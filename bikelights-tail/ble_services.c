
/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

#include "ble_services.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "SEGGER_RTT.h"


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_BR       LED Button Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_BR_t * p_BR, ble_evt_t * p_ble_evt)
{
    p_BR->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_BR       LED Button Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_BR_t * p_BR, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_BR->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_BR       LED Button Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_BR_t * p_BR, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if ((p_evt_write->handle == p_BR->light_char_handles.value_handle) &&
        (p_evt_write->len == 1) &&
        (p_BR->light_state_handler != NULL))
    {
        p_BR->light_state_handler(p_BR, p_evt_write->data[0]);
    }

    if ((p_evt_write->handle == p_BR->light_mode_char_handles.value_handle) &&
        (p_evt_write->len == 1) &&
        (p_BR->light_mode_handler != NULL))
    {
        p_BR->light_mode_handler(p_BR, p_evt_write->data[0]);
    }

}


void ble_BR_on_ble_evt(ble_BR_t * p_BR, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_BR, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_BR, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_BR, p_ble_evt);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the LED characteristic.
 *
 */
static uint32_t light_control_char_add(ble_BR_t * p_BR, const ble_BR_init_t * p_BR_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = "light on/off control char";
    char_md.char_user_desc_size = 25;
    char_md.char_user_desc_max_size = 25;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_BR->uuid_type;
    ble_uuid.uuid = BR_UUID_LIGHT_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;
    
    return sd_ble_gatts_characteristic_add(p_BR->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_BR->light_char_handles);
}


static uint32_t light_mode_control_char_add(ble_BR_t * p_BR, const ble_BR_init_t * p_BR_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = "light mode control char";
    char_md.char_user_desc_size = 23;
    char_md.char_user_desc_max_size = 23;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_BR->uuid_type;
    ble_uuid.uuid = BR_UUID_LIGHT_MODE_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_BR->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_BR->light_mode_char_handles);
}


/**@brief Function for adding the Button characteristic.
 *
 */
static uint32_t light_state_char_add(ble_BR_t * p_BR, const ble_BR_init_t * p_BR_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t value = 0;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = "light state read-only char";
    char_md.char_user_desc_size = 26;
    char_md.char_user_desc_max_size = 26;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_BR->uuid_type;
    ble_uuid.uuid = BR_UUID_LIGHT_STATE_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = &value;
    
    return sd_ble_gatts_characteristic_add(p_BR->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_BR->light_state_char_handles);
}

uint32_t ble_BR_init(ble_BR_t * p_BR, const ble_BR_init_t * p_BR_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_BR->conn_handle       = BLE_CONN_HANDLE_INVALID;
    p_BR->light_state_handler = p_BR_init->light_state_handler;
    p_BR->light_mode_handler = p_BR_init->light_mode_handler;
    
    // Add service
    ble_uuid128_t base_uuid = {BR_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_BR->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_BR->uuid_type;
    ble_uuid.uuid = BR_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_BR->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = light_state_char_add(p_BR, p_BR_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = light_control_char_add(p_BR, p_BR_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = light_mode_control_char_add(p_BR, p_BR_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    
    return NRF_SUCCESS;
}

uint32_t ble_BR_on_light_change(ble_BR_t * p_BR, uint8_t light_state)
{
     ble_gatts_hvx_params_t params;
     ble_gatts_value_t light_state_char_attr_val;
     uint16_t len = sizeof(light_state);
   
     light_state_char_attr_val.len = sizeof(uint8_t);
     light_state_char_attr_val.offset = 0;
     light_state_char_attr_val.p_value = &light_state;

     SEGGER_RTT_printf(0, "RTT DEBUG: detected light state change to %d. Updating BLE chars and notifying.\n",light_state);

     sd_ble_gatts_value_set(p_BR->conn_handle,p_BR->light_state_char_handles.value_handle,&light_state_char_attr_val);
 
     memset(&params, 0, sizeof(params));
     params.type = BLE_GATT_HVX_NOTIFICATION;
     params.handle = p_BR->light_state_char_handles.value_handle;
     params.p_data = &light_state;
     params.p_len = &len;
    
     return sd_ble_gatts_hvx(p_BR->conn_handle, &params);
}

