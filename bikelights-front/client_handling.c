/*
 * Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

#include "client_handling.h"
#include <string.h>
#include <stdbool.h>
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
//#include "app_trace.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "boards.h"
#include "SEGGER_RTT.h"

#define BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV 5

#define LED_PIN_NO_OFFSET                 8                                                 /**< LED pin number offset. */

#define TAIL_LIGHT_PERIPHERAL_BASE_UUID    {0x23, 0xD1, 0xBC, 0xBC, 0xBC, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x14, 0x15, 0x00, 0x00, 0x00, 0x00}
#define TAIL_LIGHT_PERIPHERAL_SERVICE_UUID 0x1523                                            /**< Peripheral service UUID. */
#define TAIL_LIGHT_STATE_CHAR_UUID   0x1525                                           /**< Peripheral characterisctics UUID. */
#define TAIL_LIGHT_MODE_CHAR_UUID    0x1526                                            /**< Peripheral characterisctics UUID. */
#define TAIL_LIGHT_STATE_RO_CHAR_UUID  0x1524                                          /**< Peripheral characterisctics UUID. */
#define TAIL_LIGHT_CONFIG_BLOB_CHAR_UUID  0x1527                                       /**< Peripheral characterisctics UUID. */

#define BEACON_PERIPHERAL_BASE_UUID    {0x23, 0xD1, 0xBC, 0xBC, 0xBC, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x14, 0x15, 0x00, 0x00, 0x00, 0x10}
#define BEACON_PERIPHERAL_SERVICE_UUID 0x10
#define BEACON_PERIPHERAL_BASE_CHAR_UUID 0x10
#define BEACON_PERIPHERAL_CONFIG_BITMAP_CHAR_UUID 0x11

static const ble_gap_scan_params_t local_scan_parms =
  {
     0,                       // Active scanning not set.
     0,                       // Selective scanning not set.
     NULL,                    // White-list not set.
     (uint16_t)SCAN_INTERVAL, // Scan interval.
     (uint16_t)SCAN_WINDOW,   // Scan window.
     0                        // Never stop scanning unless explicitly asked to.
  };

/**@brief Function for finding client context information based on handle.
 *
 * @param[in] conn_handle  Connection handle.
 *
 * @return client context information or NULL upon failure.
 */
static uint32_t client_find(uint16_t conn_handle)
{
    uint32_t i;

    for (i = 0; i < MAX_CLIENTS; i++)
    {
        if (G_client[i].srv_db.conn_handle == conn_handle)
        {
            return i;
        }
    }

    return MAX_CLIENTS;
}


/**@brief Function for service discovery.
 *
 * @param[in] p_client Client context information.
 */
static void service_discover(client_t * p_client)
{
    uint32_t   err_code;

    p_client->state = STATE_SERVICE_DISC;
 
    SEGGER_RTT_printf(0, "RTT DEBUG: starting discovery for peer\n");
    
    err_code = ble_db_discovery_start(&(p_client->srv_db),
                                      p_client->srv_db.conn_handle);

    //APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling enabling notifications.
 *
 * @param[in] p_client Client context information.
 */
static void notif_enable(client_t * p_client)
{
    uint32_t                 err_code;
    ble_gattc_write_params_t write_params;
    uint8_t                  buf[BLE_CCCD_VALUE_LEN];

    p_client->state = STATE_NOTIF_ENABLE;

    buf[0] = BLE_GATT_HVX_NOTIFICATION;
    buf[1] = 0;

    write_params.write_op = BLE_GATT_OP_WRITE_REQ;
    write_params.handle   = p_client->srv_db.services[0].charateristics[p_client->char_index].cccd_handle;
    write_params.offset   = 0;
    write_params.len      = sizeof(buf);
    write_params.p_value  = buf;

    err_code = sd_ble_gattc_write(p_client->srv_db.conn_handle, &write_params);
    //APP_ERROR_CHECK(err_code);
}

uint8_t wait_for_BLE_read_response(client_t *client)
 {
   uint8_t wait_counter = 0; 
   
   while (1)
    { 
     if(G_BLE_read_response_arrived[client->srv_db.conn_handle])
      {
        G_BLE_read_response_arrived[client->srv_db.conn_handle] = 0;
        return 1;
      }
     wait_counter++;
     if(wait_counter >= 100)
      return 0;  // timeout
     nrf_delay_ms(50);
   }
 }

uint8_t beacon_config_sync(client_t *client)
 {
    SEGGER_RTT_printf(0, "RTT DEBUG: synchronizing system config from beacon\n");
    G_BLE_read_response_arrived[client->srv_db.conn_handle] = 0;
    read_gatt_char(client->beacon_handles.conn_handle, client->beacon_handles.config_bitmap_handle);
    if(!wait_for_BLE_read_response(client))
      {
        SEGGER_RTT_printf(0, "RTT DEBUG: peer read error - no response to read request,\n");
        return NRF_ERROR_TIMEOUT;
       }
     SEGGER_RTT_printf(0, "RTT DEBUG: beacon_config_sync: response from peer: %d\n", G_config_data.config_bitmap);
 }

uint8_t tail_light_sync(client_t *client)
 {
      uint8_t ble_result;

      if((G_PWM_light_state == PWM_ON_CONSTANT) || (G_PWM_light_state == PWM_ON_FLASHING) || (G_PWM_light_state == PWM_ON_ALTERNATE)) // turn on
       {
       if(client->connected)
         { 
          SEGGER_RTT_printf(0, "RTT DEBUG: tail_light_sync: we are on - checking peer...\n");
          G_BLE_read_response_arrived[client->srv_db.conn_handle] = 0;
          ble_result = read_gatt_char(client->tail_handles.conn_handle, client->tail_handles.light_state_ro_handle);
          if(!wait_for_BLE_read_response(client))
           {
            SEGGER_RTT_printf(0, "RTT DEBUG: peer read error - no response to read request,\n");
            return NRF_ERROR_TIMEOUT;
           }
          SEGGER_RTT_printf(0, "RTT DEBUG: tail_light_sync: response from peer: %d\n", client->tail_light_state);
          if(client->tail_light_state == PWM_OFF)
           {
            SEGGER_RTT_printf(0, "RTT DEBUG: tail_light_sync: switching peer on\n");
            ble_result = write_gatt_char_uint8_t(client->tail_handles.conn_handle, client->tail_handles.light_state_handle, LIGHT_ON);
           }
          else
           SEGGER_RTT_printf(0, "RTT DEBUG: tail_light_sync: peer is already on - return.\n");
         }
       }

      if(G_PWM_light_state == PWM_OFF) // turn off
       {
       if(client->connected)
        {
         SEGGER_RTT_printf(0, "RTT DEBUG: tail_light_sync: we are off - checking peer...\n");
         G_BLE_read_response_arrived[client->srv_db.conn_handle] = 0;
         ble_result = read_gatt_char(client->tail_handles.conn_handle, client->tail_handles.light_state_ro_handle);
         if(!wait_for_BLE_read_response(client))
          {
           SEGGER_RTT_printf(0, "RTT DEBUG: peer read error - no response to read request,\n");
           return NRF_ERROR_TIMEOUT;
          }
         SEGGER_RTT_printf(0, "RTT DEBUG: tail_light_sync: response from peer: %d\n", client->tail_light_state);
         if((client->tail_light_state == PWM_ON_CONSTANT) || (client->tail_light_state == PWM_ON_FLASHING) || (client->tail_light_state == PWM_ON_ALTERNATE))  
          {
           SEGGER_RTT_printf(0, "RTT DEBUG: tail_light_sync: switching peer off.\n");
           ble_result = write_gatt_char_uint8_t(client->tail_handles.conn_handle, client->tail_handles.light_state_handle, LIGHT_OFF);
          }
         else
          SEGGER_RTT_printf(0, "RTT DEBUG: tail_light_sync: peer is already off - return.\n");
        }
      }

   return ble_result;

 }


static void db_discovery_evt_handler_beacon(ble_db_discovery_evt_t * p_evt)
{
    // Find the client using the connection handle.
    client_t * p_client;
    uint32_t   index;
    uint8_t    chars_found = 0;

    index = client_find(p_evt->conn_handle);
    p_client = &G_client[index];

    if(p_evt->evt_type == BLE_DB_DISCOVERY_ERROR)
     SEGGER_RTT_printf(0, "RTT DEBUG: db_discovery_evt_handler_beacon: discovery error\n",index);

    if(p_evt->evt_type == BLE_DB_DISCOVERY_SRV_NOT_FOUND)
     SEGGER_RTT_printf(0, "RTT DEBUG: db_discovery_evt_handler_beacon: service not found\n",index);

    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
    {
        uint8_t i;

        for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            ble_db_discovery_char_t * p_characteristic;

            p_characteristic = &(p_evt->params.discovered_db.charateristics[i]);

            if ((p_characteristic->characteristic.uuid.uuid == BEACON_PERIPHERAL_BASE_CHAR_UUID)
                &&
                (p_characteristic->characteristic.uuid.type == m_base_uuid_type_beacon))
            {
              p_client->beacon_handles.conn_handle = p_evt->conn_handle;
              p_client->beacon_handles.base_handle = p_characteristic->characteristic.handle_value;
              blink_debug_led(0,1);
              chars_found++;
            }

           if ((p_characteristic->characteristic.uuid.uuid == BEACON_PERIPHERAL_CONFIG_BITMAP_CHAR_UUID)
                &&
                (p_characteristic->characteristic.uuid.type == m_base_uuid_type_beacon))
            {
              p_client->beacon_handles.conn_handle = p_evt->conn_handle;
              p_client->beacon_handles.config_bitmap_handle = p_characteristic->characteristic.handle_value;
              blink_debug_led(0,1);
              chars_found++;
            }

         }

     if(chars_found > 0)  // if true - this is our beacon
      {
        SEGGER_RTT_printf(0, "RTT DEBUG: discovered bike beacon\n");
        p_client->client_type = CLIENT_TYPE_BEACON;
        G_beacon_connected = 1;
      }

     p_client->connected = 1;
     G_config_needs_sync = 1;

   }
 
   if(G_config_data.ble_peer_count > 0)  
    {
     if(G_client_count < G_config_data.ble_peer_count)
      sd_ble_gap_scan_start(&local_scan_parms);
    }
   else
    sd_ble_gap_scan_start(&local_scan_parms);

 }


// db discovery handler for tail light peripherial

static void db_discovery_evt_handler_tail(ble_db_discovery_evt_t * p_evt)
{
    // Find the client using the connection handle.
    client_t * p_client;
    uint32_t   index;
    uint8_t    chars_found = 0;

    index = client_find(p_evt->conn_handle);
    p_client = &G_client[index];

    if(p_evt->evt_type == BLE_DB_DISCOVERY_ERROR)
      SEGGER_RTT_printf(0, "RTT DEBUG: db_discovery_evt_handler_tail: discovery error\n");

    if(p_evt->evt_type == BLE_DB_DISCOVERY_SRV_NOT_FOUND)
      SEGGER_RTT_printf(0, "RTT DEBUG: db_discovery_evt_handler_tail: service not found\n");

    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
    {
        uint8_t i;

        for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            ble_db_discovery_char_t * p_characteristic;

            p_characteristic = &(p_evt->params.discovered_db.charateristics[i]);

            if ((p_characteristic->characteristic.uuid.uuid == TAIL_LIGHT_MODE_CHAR_UUID)
                &&
                (p_characteristic->characteristic.uuid.type == m_base_uuid_type_tail))
            {
              p_client->tail_handles.conn_handle = p_evt->conn_handle;
              p_client->tail_handles.light_mode_handle = p_characteristic->characteristic.handle_value;
              blink_debug_led(0,1);
              chars_found++;
            }
           else if ((p_characteristic->characteristic.uuid.uuid == TAIL_LIGHT_STATE_CHAR_UUID)
                &&
                (p_characteristic->characteristic.uuid.type == m_base_uuid_type_tail))
            {
              p_client->tail_handles.conn_handle = p_evt->conn_handle;
              p_client->tail_handles.light_state_handle = p_characteristic->characteristic.handle_value;
              blink_debug_led(0,1);
              chars_found++;
            }
           else if ((p_characteristic->characteristic.uuid.uuid == TAIL_LIGHT_STATE_RO_CHAR_UUID)
                &&
                (p_characteristic->characteristic.uuid.type == m_base_uuid_type_tail))
            {
              p_client->tail_handles.conn_handle = p_evt->conn_handle;
              p_client->tail_handles.light_state_ro_handle = p_characteristic->characteristic.handle_value;
              blink_debug_led(0,1);
              chars_found++;
            }
           else if ((p_characteristic->characteristic.uuid.uuid == TAIL_LIGHT_CONFIG_BLOB_CHAR_UUID)
                &&
                (p_characteristic->characteristic.uuid.type == m_base_uuid_type_tail))
            {
              p_client->tail_handles.conn_handle = p_evt->conn_handle;
              p_client->tail_handles.config_blob_handle = p_characteristic->characteristic.handle_value;
              blink_debug_led(0,1);
              chars_found++;
            }

        }

      if(chars_found > 0)
       {
        p_client->client_type = CLIENT_TYPE_TAIL_LIGHT;
        SEGGER_RTT_printf(0, "RTT DEBUG: discovered tail light\n");
       }
     
      p_client->connected = 1;
      p_client->needs_sync = 1;

    }

   if(G_config_data.ble_peer_count > 0)
    {
     if(G_client_count < G_config_data.ble_peer_count)
      sd_ble_gap_scan_start(&local_scan_parms);
    }
   else
    sd_ble_gap_scan_start(&local_scan_parms);

}

uint8_t write_gatt_char_uint8_t(uint16_t conn_handle, uint16_t char_descriptor, uint8_t val)
 {

   ble_gattc_write_params_t write_params = {0};
   uint32_t write_result;

   write_params.write_op = BLE_GATT_OP_WRITE_REQ;
   write_params.handle = char_descriptor;
   write_params.offset = 0x0;
   write_params.len = sizeof(uint8_t);
   write_params.p_value = &val;
   
   blink_debug_led(1,0);

   write_result = sd_ble_gattc_write(conn_handle, &write_params);

   if(write_result == NRF_SUCCESS)
    blink_debug_led(0,1);
   if(write_result == BLE_ERROR_INVALID_CONN_HANDLE)
    blink_debug_led(0,2);
   if(write_result == NRF_ERROR_INVALID_STATE)
    blink_debug_led(0,3);
   if(write_result == NRF_ERROR_INVALID_ADDR)
    blink_debug_led(0,4);
   if(write_result == NRF_ERROR_INVALID_PARAM)
    blink_debug_led(0,5);
   if(write_result == NRF_ERROR_BUSY)
    blink_debug_led(0,6);

   return write_result;

 }

uint8_t read_gatt_char(uint16_t conn_handle, uint16_t char_descriptor)
 {
   return sd_ble_gattc_read(conn_handle, char_descriptor, 0);
 }


static void on_evt_read_rsp(ble_evt_t * p_ble_evt, client_t * p_client)
{

if (p_client != NULL)
 {
  
   if(p_ble_evt->evt.gattc_evt.params.read_rsp.handle == p_client->tail_handles.light_state_ro_handle)  // if this is a response from tail light
    { 
      p_client->tail_light_state = p_ble_evt->evt.gattc_evt.params.read_rsp.data[0];
      blink_debug_led(0,1);
    }

   if(p_ble_evt->evt.gattc_evt.params.read_rsp.handle == p_client->beacon_handles.config_bitmap_handle)  // if this is a response from tail light
    {
      G_config_data.config_bitmap  = G_config_data.config_bitmap | p_ble_evt->evt.gattc_evt.params.read_rsp.data[0];
      G_config_data.config_bitmap  = G_config_data.config_bitmap | p_ble_evt->evt.gattc_evt.params.read_rsp.data[1] << 8;
      G_config_data.config_bitmap  = G_config_data.config_bitmap | p_ble_evt->evt.gattc_evt.params.read_rsp.data[2] << 16;
      G_config_data.config_bitmap  = G_config_data.config_bitmap | p_ble_evt->evt.gattc_evt.params.read_rsp.data[3] << 24;
      blink_debug_led(0,1);
    }

 }

}
/**@brief Function for setting client to the running state once write response is received.
 *
 * @param[in] p_ble_evt Event to handle.
 */
static void on_evt_write_rsp(ble_evt_t * p_ble_evt, client_t * p_client)
{
    if ((p_client != NULL) && (p_client->state == STATE_NOTIF_ENABLE))
    {
        if (p_ble_evt->evt.gattc_evt.params.write_rsp.handle !=
            p_client->srv_db.services[0].charateristics[p_client->char_index].cccd_handle)
        {
            // Got response from unexpected handle.
            p_client->state = STATE_ERROR;
        }
        else
        {
            p_client->state = STATE_RUNNING;
        }
    }
}


/**@brief Function for toggling LEDS based on received notifications.
 *
 * @param[in] p_ble_evt Event to handle.
 */
static void on_evt_hvx(ble_evt_t * p_ble_evt, client_t * p_client, uint32_t index)
{
    if ((p_client != NULL) && (p_client->state == STATE_RUNNING))
    {
        if (
                (
                        p_ble_evt->evt.gattc_evt.params.hvx.handle
                        ==
                                p_client->srv_db.services[0].charateristics[p_client->char_index].characteristic.handle_value
                )
                &&
                (p_ble_evt->evt.gattc_evt.params.hvx.len == 1)
        )
        {
            if(index < LEDS_NUMBER)
            {
                uint8_t leds[] = LEDS_LIST;

                if (p_ble_evt->evt.gattc_evt.params.hvx.data[0] == 0)
                {
                    LEDS_OFF(1<<leds[index]);
                }
                else
                {
                    LEDS_ON(1<<leds[index]);
                }
            }
        }
    }
}


/**@brief Function for handling timeout events.
 */
static void on_evt_timeout(ble_evt_t * p_ble_evt, client_t * p_client)
{
    //APP_ERROR_CHECK_BOOL(p_ble_evt->evt.gattc_evt.params.timeout.src
    //                     == BLE_GATT_TIMEOUT_SRC_PROTOCOL);

    if (p_client != NULL)
    {
        p_client->state = STATE_ERROR;
    }
}


ret_code_t client_handling_dm_event_handler(const dm_handle_t    * p_handle,
                                              const dm_event_t     * p_event,
                                              const ret_code_t     event_result)
{
    client_t * p_client = &G_client[p_handle->connection_id];

    switch (p_event->event_id)
    {
       case DM_EVT_LINK_SECURED:
           // Attempt configuring CCCD now that bonding is established.
           if (event_result == NRF_SUCCESS)
           {
               notif_enable(p_client);
           }
           break;
       default:
           break;
    }

    return NRF_SUCCESS;
}


void client_handling_ble_evt_handler(ble_evt_t * p_ble_evt)
{
    client_t * p_client = NULL;

    uint32_t index = client_find(p_ble_evt->evt.gattc_evt.conn_handle);
    if (index != MAX_CLIENTS)
    {
       p_client = &G_client[index];
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_WRITE_RSP:
            if ((p_ble_evt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_AUTHENTICATION) ||
                (p_ble_evt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_ENCRYPTION))
            {
                uint32_t err_code = dm_security_setup_req(&p_client->handle);
                //APP_ERROR_CHECK(err_code);

            }

           switch (p_ble_evt->evt.gattc_evt.gatt_status)
            {
            case BLE_GATT_STATUS_SUCCESS:
              blink_debug_led(0,1);
              break;
            }

            on_evt_write_rsp(p_ble_evt, p_client);

            break;

        case BLE_GATTC_EVT_READ_RSP:
            //switch (p_ble_evt->evt.gattc_evt.gatt_status)
            // {
            //  case BLE_GATT_STATUS_SUCCESS:
            //   blink_debug_led(0,1);
            //   break;
            // }
            G_BLE_read_response_arrived[p_client->srv_db.conn_handle] = 1;
            on_evt_read_rsp(p_ble_evt, p_client);
            break;

        case BLE_GATTC_EVT_HVX:
            on_evt_hvx(p_ble_evt, p_client, index);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            on_evt_timeout(p_ble_evt, p_client);
            break;

        default:
            break;
    }


    if (p_client != NULL)
    {
        ble_db_discovery_on_ble_evt(&(p_client->srv_db), p_ble_evt);
    }
}


/**@brief Database discovery module initialization.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init();
    //APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the client handling.
 */
void client_handling_init(void)
{
    uint32_t err_code;
    uint32_t i;

    ble_uuid128_t base_uuid_1 = {TAIL_LIGHT_PERIPHERAL_BASE_UUID};
    ble_uuid128_t base_uuid_2 = {BEACON_PERIPHERAL_BASE_UUID};

    err_code = sd_ble_uuid_vs_add(&base_uuid_1, &m_base_uuid_type_tail);
    err_code = sd_ble_uuid_vs_add(&base_uuid_2, &m_base_uuid_type_beacon);   

    //APP_ERROR_CHECK(err_code);

    for (i = 0; i < MAX_CLIENTS; i++)
    {
        G_client[i].state  = IDLE;
        G_client[i].connected = 0;
        
    }

    G_client_count = 0;

    db_discovery_init();

    // Register with discovery module for the discovery of the service.
    ble_uuid_t tail_light_uuid;
    ble_uuid_t beacon_uuid;

    tail_light_uuid.type = m_base_uuid_type_tail;
    tail_light_uuid.uuid = TAIL_LIGHT_PERIPHERAL_SERVICE_UUID;

    beacon_uuid.type = m_base_uuid_type_beacon;
    beacon_uuid.uuid = BEACON_PERIPHERAL_SERVICE_UUID;

    
    err_code = ble_db_discovery_evt_register(&tail_light_uuid,
                                             db_discovery_evt_handler_tail);

    err_code = ble_db_discovery_evt_register(&beacon_uuid,
                                             db_discovery_evt_handler_beacon);

    //APP_ERROR_CHECK(err_code);
}

/**@brief Function for returning the current number of clients.
 */
uint8_t client_handling_count(void)
{
    return G_client_count;
}


/**@brief Function for creating a new client.
 */
uint32_t client_handling_create(const dm_handle_t * p_handle, uint16_t conn_handle)
{
    SEGGER_RTT_printf(0, "RTT DEBUG: BLE connection ID: %d\n",p_handle->connection_id);
    G_client[p_handle->connection_id].state              = STATE_SERVICE_DISC;
    G_client[p_handle->connection_id].srv_db.conn_handle = conn_handle;
    G_client_count++;
    G_client[p_handle->connection_id].handle             = (*p_handle);
    service_discover(&G_client[p_handle->connection_id]);

    return NRF_SUCCESS;
}


/**@brief Function for freeing up a client by setting its state to idle.
 */
uint32_t client_handling_destroy(const dm_handle_t * p_handle)
{
    uint32_t      err_code = NRF_SUCCESS;
    client_t    * p_client = &G_client[p_handle->connection_id];
	
    if (p_client->state != IDLE)
    {
            p_client->connected = 0;
            G_client_count--;
            p_client->state = IDLE;
            if(p_client->client_type == CLIENT_TYPE_BEACON)
             {
              G_beacon_connected = 0;
              G_beacon_disconnected = 1;
             }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
    return err_code;
}

