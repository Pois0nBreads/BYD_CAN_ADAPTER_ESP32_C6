#ifndef BLE_MULTI_CONN_H
#define BLE_MULTI_CONN_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void testADD(void);

/**
 * @brief 初始化 BLE 多连接服务器（支持 Notify）
 * @param max_connections 最大连接数（1~9）
 * @return ESP_OK 成功，其他失败
 */
esp_err_t ble_multi_conn_init(uint8_t max_connections);

/**
 * @brief 向指定连接发送 Notify 数据
 * @param conn_id 连接ID
 * @param data 数据指针（最大20字节）
 * @param len 数据长度
 * @return true 发送成功，false 失败（如未订阅或连接无效）
 */
bool ble_multi_conn_send_notify(uint16_t conn_id, const uint8_t *data, uint16_t len);

/**
 * @brief 获取当前已连接设备数量
 */
int ble_multi_conn_get_connected_count(void);

/**
 * @brief 获取指定索引的连接ID（用于遍历）
 * @param index 连接槽位索引（0 ~ max_connections-1）
 * @return conn_id，若槽位未使用则返回0xFFFF
 */
uint16_t ble_multi_conn_get_conn_id_by_index(int index);

/**
 * @brief 向指定连接发送 Indicate（可靠，需要对方确认）
 * @param conn_id 连接ID
 * @param data 数据指针（最大20字节）
 * @param len 数据长度
 * @return true 发送成功，false 失败（如未订阅或连接无效）
 */
bool ble_multi_conn_send_indicate(uint16_t conn_id, const uint8_t *data, uint16_t len);

/**
 * @brief 设置数据接收回调（收到客户端写入时调用）
 * @param callback 回调函数，参数为(conn_id, data, len)
 */
typedef void (*ble_receive_callback_t)(uint16_t conn_id, const uint8_t *data, uint16_t len);
void ble_multi_conn_set_receive_callback(ble_receive_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif // BLE_MULTI_CONN_H