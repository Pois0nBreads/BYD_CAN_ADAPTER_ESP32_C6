#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "ble_multi_conn.h"

#define TAG "BLE_MULTI"
#define DEVICE_NAME "ESP32C6_BLE"

// 固定 UUID
static const uint16_t SERVICE_UUID        = 0xAAAA;
static const uint16_t CHARACTERISTIC_UUID = 0xBBBB;
static const uint16_t CCCD_UUID           = 0x2902;

#define MANUFACTURER_ID 0x0809
#define ADV_DATA_LEN 7  
static uint8_t manufacturer_data[ADV_DATA_LEN + 2];

static uint8_t g_max_connections = 0;

typedef struct {
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
    bool in_use;
    bool notify_enabled;      // Notify 订阅标志
    bool indicate_enabled;    // Indicate 订阅标志
} ble_conn_t;
static ble_conn_t *g_connections = NULL;

static uint16_t g_service_handle = 0;
static uint16_t g_char_handle = 0;
static uint16_t g_cccd_handle = 0;
static esp_gatt_if_t g_gatts_if = 0;

static esp_ble_adv_params_t g_adv_params = {
    // .adv_int_min        = 0x20,
    // .adv_int_max        = 0x40,
    .adv_int_min        = 0x160,
    .adv_int_max        = 0x320,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr          = {0x00},
    .peer_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static ble_receive_callback_t s_receive_callback = NULL;

// 内部函数声明
static void start_advertising(void);
static int find_free_conn_slot(void);
static int find_conn_by_id(uint16_t conn_id);
static void clear_conn(int idx);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

static void start_advertising(void) {
    esp_err_t ret = esp_ble_gap_start_advertising(&g_adv_params);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Start advertising failed: %d", ret);
    else ESP_LOGI(TAG, "Advertising started");
}

static int find_free_conn_slot(void) {
    for (int i = 0; i < g_max_connections; i++) {
        if (!g_connections[i].in_use) return i;
    }
    return -1;
}

static int find_conn_by_id(uint16_t conn_id) {
    for (int i = 0; i < g_max_connections; i++) {
        if (g_connections[i].in_use && g_connections[i].conn_id == conn_id) return i;
    }
    return -1;
}

static void clear_conn(int idx) {
    if (idx >= 0 && idx < g_max_connections) {
        g_connections[idx].in_use = false;
        g_connections[idx].notify_enabled = false;
        g_connections[idx].indicate_enabled = false;
        memset(g_connections[idx].remote_bda, 0, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "Connection slot %d cleared", idx);
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "REG_EVT");
            g_gatts_if = gatts_if;
            // 创建服务
            esp_bt_uuid_t service_uuid = { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = SERVICE_UUID } };
            esp_gatt_srvc_id_t service_id = {
                .id = { .uuid = service_uuid, .inst_id = 0 },
                .is_primary = true,
            };
            esp_ble_gatts_create_service(gatts_if, &service_id, 8);
            break;

        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "CREATE_EVT");
            g_service_handle = param->create.service_handle;
            // 添加特征：支持 Read, Write, Notify, Indicate
            esp_bt_uuid_t char_uuid = { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = CHARACTERISTIC_UUID } };
            esp_gatt_char_prop_t char_prop = ESP_GATT_CHAR_PROP_BIT_READ |
                                             ESP_GATT_CHAR_PROP_BIT_WRITE_NR |
                                             ESP_GATT_CHAR_PROP_BIT_NOTIFY ;
                                            //  ESP_GATT_CHAR_PROP_BIT_INDICATE;
            esp_ble_gatts_add_char(g_service_handle, &char_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   char_prop, NULL, NULL);
            break;

        case ESP_GATTS_ADD_CHAR_EVT:
            g_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "ADD_CHAR_EVT, handle %d", g_char_handle);
            // 添加 CCCD 描述符
            esp_bt_uuid_t cccd_uuid = { .len = ESP_UUID_LEN_16, .uuid = { .uuid16 = CCCD_UUID } };
            esp_ble_gatts_add_char_descr(g_service_handle, &cccd_uuid,
                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                         NULL, NULL);
            break;

        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            if (param->add_char_descr.descr_uuid.uuid.uuid16 == CCCD_UUID) {
                g_cccd_handle = param->add_char_descr.attr_handle;
                ESP_LOGI(TAG, "CCCD handle %d", g_cccd_handle);
                esp_ble_gatts_start_service(g_service_handle);
                start_advertising();
            }
            break;

        case ESP_GATTS_CONNECT_EVT: {
                int idx = find_free_conn_slot();
                if (idx == -1) {
                    ESP_LOGE(TAG, "No free slot, reject");
                    esp_ble_gap_disconnect(param->connect.remote_bda);
                    break;
                }
                g_connections[idx].conn_id = param->connect.conn_id;
                memcpy(g_connections[idx].remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
                g_connections[idx].in_use = true;
                g_connections[idx].notify_enabled = false;
                g_connections[idx].indicate_enabled = false;
                ESP_LOGI(TAG, "CONNECT, conn_id=%d slot=%d", param->connect.conn_id, idx);
                // 更新连接参数
                esp_ble_conn_update_params_t conn_params;
                memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
                conn_params.min_int = 0x30;
                conn_params.max_int = 0x50;
                conn_params.latency = 0;
                conn_params.timeout = 500;
                esp_ble_gap_update_conn_params(&conn_params);
                start_advertising();
            }
            break;

        case ESP_GATTS_DISCONNECT_EVT: {
                int idx = find_conn_by_id(param->disconnect.conn_id);
                if (idx != -1) clear_conn(idx);
                start_advertising();
            }
            break;

        case ESP_GATTS_WRITE_EVT: {
                ESP_LOGI(TAG, "WRITE_EVT, conn_id=%d, handle=%d", param->write.conn_id, param->write.handle);
                if (param->write.handle == g_cccd_handle && param->write.len == 2) {
                    // CCCD 写入，解析订阅值
                    uint16_t cccd_value = param->write.value[0] | (param->write.value[1] << 8);
                    int idx = find_conn_by_id(param->write.conn_id);
                    if (idx != -1) {
                        g_connections[idx].notify_enabled = (cccd_value & 0x0001) != 0;
                        g_connections[idx].indicate_enabled = (cccd_value & 0x0002) != 0;
                        ESP_LOGI(TAG, "conn %d: notify=%d, indicate=%d",
                                param->write.conn_id, g_connections[idx].notify_enabled, g_connections[idx].indicate_enabled);
                    }
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                } else if (param->write.handle == g_char_handle && !param->write.is_prep) {
                    // 特征值写入（手机控制指令） 不需要手动回复ACK
                    // ESP_LOGI(TAG, "esp_ble_gatts_send_response, conn_id=%d, handle=%d", param->write.conn_id, param->write.handle);
                    // esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                    if (s_receive_callback) {
                        s_receive_callback(param->write.conn_id, param->write.value, param->write.len);
                    }
                }
            }
            break;
        case ESP_GATTS_READ_EVT: {
                ESP_LOGI(TAG, "READ_EVT, conn_id=%d, handle=%d", param->read.conn_id, param->read.handle);
                //esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, NULL);

                if (param->read.handle == g_char_handle) {
                    esp_gatt_rsp_t rsp;
                    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                    rsp.attr_value.handle = param->read.handle;
                    rsp.attr_value.len = 3;
                    rsp.attr_value.value[0] = 0x11;
                    rsp.attr_value.value[1] = 0x45;
                    rsp.attr_value.value[2] = 0x14;
                    esp_ble_gatts_send_response(g_gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
                }
            }
            break;

        case ESP_GATTS_CONF_EVT:
            // Indicate 确认事件，一般无需额外处理
            ESP_LOGD(TAG, "Indicate confirm, conn_id=%d", param->conf.conn_id);
            break;

        default:
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            start_advertising();
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
                ESP_LOGE(TAG, "Adv start failed: %d", param->adv_start_cmpl.status);
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "Update conn params status: %d", param->update_conn_params.status);
            break;
        default:
            break;
    }
}

static esp_err_t ble_init_controller_and_stack(uint8_t max_conn) {
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    // bt_cfg.ble_max_act = max_conn;
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) return ret;
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) return ret;

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) return ret;
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) return ret;
    // 设置设备名称
    esp_ble_gap_set_device_name(DEVICE_NAME);
    return ESP_OK;
}

static void setup_adv_data(void) {
    // 填充厂商数据：厂商ID + 负载
    manufacturer_data[0] = (MANUFACTURER_ID >> 8) & 0xFF;
    manufacturer_data[1] = MANUFACTURER_ID & 0xFF;
    // memcpy(manufacturer_data + 2, ble_adv_payload, ADV_DATA_LEN);
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp        = false,
        .include_name        = true,      // 广播设备名
        .include_txpower     = true,
        .flag                = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,          // 广播Flags
        .manufacturer_len    = sizeof(manufacturer_data),
        .p_manufacturer_data = manufacturer_data,
    };
    esp_ble_gap_config_adv_data(&adv_data);
}

void testADD(void) {
    manufacturer_data[0+2] += 1;
    setup_adv_data();
}
// ========== 公共 API ==========

esp_err_t ble_multi_conn_init(uint8_t max_connections) {
    if (max_connections < 1 || max_connections > 9) return ESP_ERR_INVALID_ARG;
    g_max_connections = max_connections;
    g_connections = malloc(sizeof(ble_conn_t) * g_max_connections);
    if (!g_connections) return ESP_ERR_NO_MEM;
    memset(g_connections, 0, sizeof(ble_conn_t) * g_max_connections);

    esp_err_t ret = ble_init_controller_and_stack(g_max_connections);
    if (ret != ESP_OK) {
        free(g_connections);
        return ret;
    }

    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_register_callback(gatts_event_handler);
    setup_adv_data();
    esp_ble_gatts_app_register(0x55);
    ESP_LOGI(TAG, "Init OK, max_conn=%d", g_max_connections);
    return ESP_OK;
}

// 发送 Indicate（可靠）
bool ble_multi_conn_send_indicate(uint16_t conn_id, const uint8_t *data, uint16_t len) {
    if (!g_gatts_if || g_char_handle == 0) {
        ESP_LOGE(TAG, "Not ready");
        return false;
    }
    if (len > 20) {
        ESP_LOGE(TAG, "Data too long");
        return false;
    }
    int idx = find_conn_by_id(conn_id);
    if (idx == -1 || !g_connections[idx].indicate_enabled) {
        ESP_LOGW(TAG, "Indicate not enabled for conn %d", conn_id);
        return false;
    }
    // need_confirm = true 表示 Indicate
    esp_err_t ret = esp_ble_gatts_send_indicate(g_gatts_if, conn_id, g_char_handle, len, (uint8_t*)data, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Indicate send failed, err=%d", ret);
        return false;
    }
    return true;
}

// 发送 Notify（不可靠，保留）
bool ble_multi_conn_send_notify(uint16_t conn_id, const uint8_t *data, uint16_t len) {
    if (!g_gatts_if || g_char_handle == 0) return false;
    if (len > 20) return false;
    int idx = find_conn_by_id(conn_id);
    if (idx == -1 || !g_connections[idx].notify_enabled) {
        ESP_LOGW(TAG, "Notify not enabled for conn %d", conn_id);
        return false;
    }
    esp_err_t ret = esp_ble_gatts_send_indicate(g_gatts_if, conn_id, g_char_handle, len, (uint8_t*)data, false);
    return (ret == ESP_OK);
}

int ble_multi_conn_get_connected_count(void) {
    int count = 0;
    for (int i = 0; i < g_max_connections; i++) {
        if (g_connections && g_connections[i].in_use) count++;
    }
    return count;
}

uint16_t ble_multi_conn_get_conn_id_by_index(int index) {
    if (!g_connections || index < 0 || index >= g_max_connections) return 0xFFFF;
    if (g_connections[index].in_use) return g_connections[index].conn_id;
    return 0xFFFF;
}

void ble_multi_conn_set_receive_callback(ble_receive_callback_t callback) {
    s_receive_callback = callback;
}