#include <string.h>

#include "ble.h"

#include "ble_bls.h"

static uint32_t cmd_char_add(ble_bls_t * p_bls, ble_bls_init_t * p_bls_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_t    attr;
    ble_uuid_t          uuid;

    uuid.type = p_bls->uuid_type;
    uuid.uuid = BLS_UUID_CMD;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write = 1;

    memset(&attr, 0, sizeof(attr));
    attr.p_uuid = &uuid;
    attr.p_attr_md = &attr_md;
    attr.init_len = sizeof(ble_bls_cmd_t);
    attr.max_len = sizeof(ble_bls_cmd_t);

    return sd_ble_gatts_characteristic_add(p_bls->service_handle, &char_md, &attr, &p_bls->cmd_char_handles);
}

static uint32_t response_char_add(ble_bls_t * p_bls, ble_bls_init_t * p_bls_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr;
    ble_uuid_t          uuid;

    uuid.type = p_bls->uuid_type;
    uuid.uuid = BLS_UUID_RESPONSE;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;


    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.p_cccd_md = &cccd_md;

    memset(&attr, 0, sizeof(attr));
    attr.p_uuid = &uuid;
    attr.p_attr_md = &attr_md;
    attr.init_len = sizeof(ble_bls_response_t);
    attr.max_len = sizeof(ble_bls_response_t);

    return sd_ble_gatts_characteristic_add(p_bls->service_handle, &char_md, &attr, &p_bls->response_char_handles);
}

static uint32_t data_char_add(ble_bls_t * p_bls, ble_bls_init_t * p_bls_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_t    attr;
    ble_uuid_t          uuid;

    uuid.type = p_bls->uuid_type;
    uuid.uuid = BLS_UUID_DATA;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.vlen = 1;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write_wo_resp = 1;

    memset(&attr, 0, sizeof(attr));
    attr.p_uuid = &uuid;
    attr.p_attr_md = &attr_md;
    attr.init_len = 0;
    attr.max_len = 20;

    return sd_ble_gatts_characteristic_add(p_bls->service_handle, &char_md, &attr, &p_bls->data_char_handles);
}

static void on_write(ble_bls_t * p_bls, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * write_evt = &p_ble_evt->evt.gatts_evt.params.write;
    ble_bls_evt_t bls_evt;
    if (write_evt->handle == p_bls->cmd_char_handles.value_handle)
    {
        bls_evt.cmd = write_evt->data[0];

        p_bls->evt_handler(p_bls, &bls_evt);
    }
    else if (write_evt->handle == p_bls->data_char_handles.value_handle)
    {
        bls_evt.cmd = BLE_BLS_CMD_WRITE_LINE;

        for (uint8_t i = 0; i < write_evt->len; i++)
        {
            p_bls->buffer[p_bls->buffer_index++] = write_evt->data[i];
        }

        if (p_bls->buffer[p_bls->buffer_index-1] == '\n' || p_bls->buffer_index >= BLE_BLS_MAX_LINE_LEN)
        {
			bls_evt.data = (uint8_t *) p_bls->buffer;
			bls_evt.len = p_bls->buffer_index;
            p_bls->evt_handler(p_bls, &bls_evt);
			p_bls->buffer_index = 0;
        }
    }

}

void ble_bls_on_ble_evt(ble_bls_t * p_bls, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_bls->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            p_bls->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_bls, p_ble_evt);
            break;

        default:
            break;
    }
}

uint32_t ble_bls_init(ble_bls_t * p_bls, ble_bls_init_t * p_bls_init)
{
    uint32_t err_code;
    ble_uuid128_t base_uuid = {BLS_UUID_BASE};
    ble_uuid_t service_uuid;

    if (p_bls_init->evt_handler == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
	p_bls->evt_handler = p_bls_init->evt_handler;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_bls->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    service_uuid.type = p_bls->uuid_type;
    service_uuid.uuid = BLS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &p_bls->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = cmd_char_add(p_bls, p_bls_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = response_char_add(p_bls, p_bls_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = data_char_add(p_bls, p_bls_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    return NRF_SUCCESS;
}

uint32_t ble_bls_response_send(ble_bls_t * p_bls, ble_bls_response_t response)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(response);

    params.handle = p_bls->response_char_handles.value_handle;
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.offset = 0;
    params.p_data = &response;
    params.p_len = &len;
    
    return sd_ble_gatts_hvx(p_bls->conn_handle, &params);
}
