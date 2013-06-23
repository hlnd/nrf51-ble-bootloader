#ifndef __BLE_BLS_H_
#define __BLE_BLS_H_

#include <stdint.h>

#define BLS_UUID_BASE {0x44, 0x98, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define BLS_UUID_SERVICE 0x1000
#define BLS_UUID_CMD 0x2000
#define BLS_UUID_RESPONSE 0x2001
#define BLS_UUID_DATA 0x2002

#define BLE_BLS_MAX_LINE_LEN 48

typedef enum
{
    BLE_BLS_CMD_NOP,
    BLE_BLS_CMD_ERASE_APP,
    BLE_BLS_CMD_WRITE_LINE,
    BLE_BLS_CMD_RESET_AND_RUN,
} ble_bls_cmd_t;

typedef enum
{
    BLE_BLS_RESPONSE_SUCCESS,
    BLE_BLS_RESPONSE_FAILURE,
} ble_bls_response_t;

typedef struct
{
    ble_bls_cmd_t cmd;
    uint8_t len;
    uint8_t * data;
} ble_bls_evt_t;

typedef struct ble_bls_s ble_bls_t;

typedef void (*ble_bls_evt_handler_t)(ble_bls_t * p_bls, ble_bls_evt_t * p_evt);

typedef struct
{
    ble_bls_evt_handler_t evt_handler;
} ble_bls_init_t;

typedef struct ble_bls_s
{
    uint16_t conn_handle;
    uint8_t uuid_type;
    uint16_t service_handle;
    ble_gatts_char_handles_t cmd_char_handles;
    ble_gatts_char_handles_t response_char_handles;
    ble_gatts_char_handles_t data_char_handles;
    ble_bls_evt_handler_t evt_handler;
    uint8_t buffer_index;
    uint8_t buffer[BLE_BLS_MAX_LINE_LEN];
} ble_bls_t;

void ble_bls_on_ble_evt(ble_bls_t * p_bls, ble_evt_t * p_ble_evt);
uint32_t ble_bls_init(ble_bls_t * p_bls, ble_bls_init_t * p_bls_init);
uint32_t ble_bls_response_send(ble_bls_t * p_bls, ble_bls_response_t response);

#endif
