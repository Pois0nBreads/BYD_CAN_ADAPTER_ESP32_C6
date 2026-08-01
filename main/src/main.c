/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/twai.h"

#include "ble_multi_conn.h"


#define BLINK_GPIO GPIO_NUM_15
#define CAN_RX0 GPIO_NUM_2
#define CAN_TX0 GPIO_NUM_3
#define CAN_RX1 GPIO_NUM_4
#define CAN_TX1 GPIO_NUM_5
#define TAG "MAIN"

static twai_handle_t twai_bus_0;
static twai_handle_t twai_bus_1;

static void twai_receive_task_0(void *arg);
static void twai_receive_task_1(void *arg);
static void sendHeartPacket(void *arg);
static void on_ble_receive(uint16_t conn_id, const uint8_t *data, uint16_t len);

static esp_err_t initChip();
static esp_err_t initTwai();

void app_main(void) {
    ESP_LOGI(TAG, "Hello world!\n");
    if (initChip() != ESP_OK
        || initTwai() != ESP_OK
        || ble_multi_conn_init(4) != ESP_OK ) {
        ESP_LOGE(TAG, "Init failed, Restart ESP32");
        esp_restart();
        return;
    }

    // 设置接收回调
    ble_multi_conn_set_receive_callback(on_ble_receive);
    // 主循环：打印连接数量，并定期发送心跳（Notify）
    xTaskCreatePinnedToCore(sendHeartPacket, "BLE_HeTx", 4096, NULL, 8, NULL, tskNO_AFFINITY); 

    gpio_reset_pin(BLINK_GPIO);
    // 设置该引脚为输出模式
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    bool blink_int = false;
    while (1) {
        gpio_set_level(BLINK_GPIO, blink_int = !blink_int);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// 数据接收回调（收到手机控制指令时触发）
static void on_ble_receive(uint16_t conn_id, const uint8_t *data, uint16_t len) {
    ESP_LOG_BUFFER_HEX("MAIN: Received: ", data, len);
    
    // 示例：回复确认消息
    // uint8_t ping[] = {0xFF, 0xFF, 0xFF, 0xFF};
    // ble_multi_conn_send_notify(conn_id, ping, sizeof(ping));

    if (len != 12)
        return;
    uint8_t dlc = data[3];
    uint16_t id = (data[1] << 8) | data[2];
    uint8_t ch  = data[0];

    twai_message_t message = {
        .identifier = id,            // 消息ID, 本例中使用标准帧ID
        .extd = 0,                      // 0: 标准帧 (11位ID), 1: 扩展帧 (29位ID)
        .rtr = 0,                       // 0: 数据帧, 1: 远程帧
        .data_length_code = dlc,          // 数据长度，单位: 字节
        // .ss = 0,                      // 默认0，自动重试
    };
    memcpy(message.data, data + 4, dlc);
    
    if (ch == 0x00) {
        twai_transmit_v2(twai_bus_0, &message, 0);
    }
    if (ch == 0x01) {
        twai_transmit_v2(twai_bus_1, &message, 0);
    }

    // TODO: 解析控制指令，更新设备状态，然后调用 ble_multi_conn_send_indicate 上报新状态
    // 例如：
    // uint8_t new_state = parse_command(data, len);
    // uint8_t status_report[] = {0x01, new_state};
    // ble_multi_conn_send_indicate(conn_id, status_report, sizeof(status_report));
}

static void sendHeartPacket(void *arg)  {
    uint8_t ping[] = {0xFF, 0xFF, 0xFF, 0xFF};
    while (1) {
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        // 向所有已连接设备发送心跳indicate
        for (int i = 0; i < 4; i++) {
            uint16_t cid = ble_multi_conn_get_conn_id_by_index(i);
            if (cid != 0xFFFF) {
                // ble_multi_conn_send_indicate(cid, ping, sizeof(ping));
                ble_multi_conn_send_notify(cid, ping, sizeof(ping));
            }
        }
    }
}

static void twai_receive_task_0(void *arg) {
    twai_message_t msg;
    uint32_t alerts;
    while (1) {
        // 读取所有待处理的警报（非阻塞）
        esp_err_t ret = twai_receive_v2(twai_bus_0, &msg, pdMS_TO_TICKS(100));
        twai_read_alerts_v2(twai_bus_0, &alerts, pdMS_TO_TICKS(0));
        if (alerts & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "TWAI bus 0 bus-off! System restart...");
            esp_restart();  // 直接重启
        }
        if (ret != ESP_OK)
            continue;
        if (msg.rtr != 0)
            continue;
        if (msg.extd != 0)
            continue;
            
        uint8_t data[12] = {0};
        data[0] = 0x00;
        data[1] = (msg.identifier >> 8) & 0xFF;
        data[2] = msg.identifier & 0xFF;
        data[3] = msg.data_length_code;
        for (int j = 0; j < msg.data_length_code; j++)
            data[j+4] = msg.data[j];

        for (int i = 0; i < 4; i++) {
            uint16_t cid = ble_multi_conn_get_conn_id_by_index(i);
            if (cid != 0xFFFF) {
                ble_multi_conn_send_notify(cid, data, sizeof(data));
            }
        }
        // int j;
        // printf("ReceiveID: 0x%lX Data: ", msg.identifier);
        // for (j = 0; j < msg.data_length_code; j++)
        //     printf("%02X ", msg.data[j]);
        // printf("\n");
    }
    vTaskDelete(NULL);
}

static void twai_receive_task_1(void *arg) {
    twai_message_t msg;
    uint32_t alerts;
    while (1) {
        // 读取所有待处理的警报（非阻塞）
        esp_err_t ret = twai_receive_v2(twai_bus_1, &msg, pdMS_TO_TICKS(100));
        twai_read_alerts_v2(twai_bus_1, &alerts, pdMS_TO_TICKS(0));
        if (alerts & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "TWAI bus 1 bus-off! System restart...");
            esp_restart();  // 直接重启
        }
        if (ret != ESP_OK)
            continue;
        if (msg.rtr != 0)
            continue;
        if (msg.extd != 0)
            continue;
        uint8_t data[12] = {0};
        data[0] = 0x01;
        data[1] = (msg.identifier >> 8) & 0xFF;
        data[2] = msg.identifier & 0xFF;
        data[3] = msg.data_length_code;
        for (int j = 0; j < msg.data_length_code; j++)
            data[j+4] = msg.data[j];
            
        for (int i = 0; i < 4; i++) {
            uint16_t cid = ble_multi_conn_get_conn_id_by_index(i);
            if (cid != 0xFFFF) {
                ble_multi_conn_send_notify(cid, data, sizeof(data));
            }
        }
        // int j;
        // printf("ReceiveID: 0x%lX Data: ", msg.identifier);
        // for (j = 0; j < msg.data_length_code; j++)
        //     printf("%02X ", msg.data[j]);
        // printf("\n");
    }
    vTaskDelete(NULL);
}

static esp_err_t initTwai() {
    //CAN接口基本配置
    ESP_LOGI(TAG, "Init TWAI Driver");
    twai_general_config_t g_config0 = {
        .controller_id = 0,
        .mode = TWAI_MODE_NORMAL , //TWAI_MODE_NORMAL / TWAI_MODE_NO_ACK / TWAI_MODE_LISTEN_ONLY
        .tx_io = CAN_TX0, //IO号
        .rx_io = CAN_RX0, //IO号
        .clkout_io = TWAI_IO_UNUSED, //io号，不用为-1
        .bus_off_io = TWAI_IO_UNUSED,//io号，不用为-1
        .tx_queue_len = 5, //发送队列长度，0-禁用发送队列
        .rx_queue_len = 5,//接收队列长度
        .alerts_enabled = TWAI_ALERT_ALL,  //警告标志 TWAI_ALERT_ALL 可开启所有警告
        .clkout_divider = 0,//1 to 14 , 0-不用
        .intr_flags = ESP_INTR_FLAG_LEVEL1//中断优先级
    };
    twai_general_config_t g_config1 = {
        .controller_id = 1,
        .mode = TWAI_MODE_NORMAL , //TWAI_MODE_NORMAL / TWAI_MODE_NO_ACK / TWAI_MODE_LISTEN_ONLY
        .tx_io = CAN_TX1, //IO号
        .rx_io = CAN_RX1, //IO号
        .clkout_io = TWAI_IO_UNUSED, //io号，不用为-1
        .bus_off_io = TWAI_IO_UNUSED,//io号，不用为-1
        .tx_queue_len = 5, //发送队列长度，0-禁用发送队列
        .rx_queue_len = 5,//接收队列长度
        .alerts_enabled = TWAI_ALERT_ALL,  //警告标志 TWAI_ALERT_ALL 可开启所有警告
        .clkout_divider = 0,//1 to 14 , 0-不用
        .intr_flags = ESP_INTR_FLAG_LEVEL1//中断优先级
    };
    //过滤器配置
    twai_filter_config_t f_config0 = {
        .acceptance_code = 0, //验证代码
        .acceptance_mask = 0xFFFFFFFF, //验证掩码 0xFFFFFFFF表示全部接收
        .single_filter = true//true：单过滤器模式 false：双过滤器模式
    };
    twai_filter_config_t f_config1 = {
        .acceptance_code = 0, //验证代码
        .acceptance_mask = 0xFFFFFFFF, //验证掩码 0xFFFFFFFF表示全部接收
        .single_filter = true//true：单过滤器模式 false：双过滤器模式
    };
    //CAN接口时序配置官方提供了1K to 1Mbps的常用配置
    twai_timing_config_t t_config0 = TWAI_TIMING_CONFIG_125KBITS(); //TWAI_TIMING_CONFIG_500KBITS()
    twai_timing_config_t t_config1 = TWAI_TIMING_CONFIG_125KBITS(); //TWAI_TIMING_CONFIG_500KBITS()

    ESP_ERROR_CHECK(twai_driver_install_v2(&g_config0, &t_config0, &f_config0, &twai_bus_0));
    ESP_LOGI(TAG, "TWAI 0 Driver installed");
    ESP_ERROR_CHECK(twai_driver_install_v2(&g_config1, &t_config1, &f_config1, &twai_bus_1));
    ESP_LOGI(TAG, "TWAI 1 Driver installed");
    
    ESP_ERROR_CHECK(twai_start_v2(twai_bus_0));
    ESP_LOGI(TAG, "TWAI 0 Driver started\n");
    ESP_ERROR_CHECK(twai_start_v2(twai_bus_1));
    ESP_LOGI(TAG, "TWAI 1 Driver started\n");

    xTaskCreatePinnedToCore(twai_receive_task_0, "TWAI_rx_0", 4096, NULL, 8, NULL, tskNO_AFFINITY); 
    xTaskCreatePinnedToCore(twai_receive_task_1, "TWAI_rx_1", 4096, NULL, 8, NULL, tskNO_AFFINITY); 
    return ESP_OK;
}

static esp_err_t initChip() {
    /* Print chip information */
    ESP_LOGI(TAG, "Init Chip Info");
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");
    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG, "silicon revision v%d.%d, ", major_rev, minor_rev);
    uint32_t flash_size;
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGE(TAG, "Get flash size failed");
        return ESP_ERR_FLASH_BASE;
    }
    ESP_LOGI(TAG, "%" PRIu32 "MB %s flash", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGI(TAG, "Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
    return ESP_OK;
}