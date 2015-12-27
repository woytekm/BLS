/*
 * Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

 /**@file
 *
 * @defgroup XXXX
 * @{
 * @ingroup  YYYY
 *
 * @brief    ZZZZZ.
 */

#ifndef CLIENT_HANDLING_H__
#define CLIENT_HANDLING_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "device_manager.h"

#include "ble_db_discovery.h"
#include "ble_srv_common.h"

//
// max clients is 5 - 4 tail lights and a beacon
//

#define MAX_CLIENTS  DEVICE_MANAGER_MAX_CONNECTIONS  /**< Max number of clients. */

/**@brief Funtion for initializing the module.
 */
void client_handling_init(void);

/**@brief Funtion for returning the current number of clients.
 *
 * @return  The current number of clients.
 */
uint8_t client_handling_count(void);

/**@brief Funtion for creating a new client.
 *
 * @param[in] p_handle    Device Manager Handle. For link related events, this parameter
 *                        identifies the peer.
 *
 * @param[in] conn_handle Identifies link for which client is created.
 * @return NRF_SUCCESS on success, any other on failure.
 */
uint32_t client_handling_create(const dm_handle_t * p_handle, uint16_t conn_handle);

/**@brief Funtion for freeing up a client by setting its state to idle.
 *
 * @param[in] p_handle  Device Manager Handle. For link related events, this parameter
 *                      identifies the peer.
 *
 * @return NRF_SUCCESS on success, any other on failure.
 */
uint32_t client_handling_destroy(const dm_handle_t * p_handle);

/**@brief Funtion for handling client events.
 *
 * @param[in] p_ble_evt  Event to be handled.
 */
void client_handling_ble_evt_handler(ble_evt_t * p_ble_evt);
uint8_t read_gatt_char(uint16_t conn_handle, uint16_t char_descriptor);

/**@brief Funtion for handling device manager events.
 *
 * @param[in] p_handle       Identifies device with which the event is associated.
 * @param[in] p_event        Event to be handled.
 * @param[in] event_result   Event result indicating whether a procedure was successful or not.
 */
ret_code_t client_handling_dm_event_handler(const dm_handle_t    * p_handle,
                                              const dm_event_t     * p_event,
                                              const ret_code_t     event_result);


typedef enum
{
    IDLE,                                           /**< Idle state. */
    STATE_SERVICE_DISC,                             /**< Service discovery state. */
    STATE_NOTIF_ENABLE,                             /**< State where the request to enable notifications is sent to the peer. . */
    STATE_RUNNING,                                  /**< Running state. */
    STATE_ERROR                                     /**< Error state. */
} client_state_t;


typedef struct {
                uint16_t conn_handle;
                uint16_t light_state_handle;
                uint16_t light_mode_handle;
                uint16_t light_state_ro_handle;
                uint16_t config_blob_handle;
               } tail_light_handles_t;

typedef struct {
                uint16_t conn_handle;
                uint16_t base_handle;
                uint16_t config_bitmap_handle;
               } beacon_handles_t;

tail_light_handles_t G_tail_light_handles[MAX_CLIENTS];

/**@brief Client context information. */
typedef struct
{
    ble_db_discovery_t           srv_db;            /**< The DB Discovery module instance associated with this client. */
    dm_handle_t                  handle;            /**< Device manager identifier for the device. */
    uint8_t                      char_index;        /**< Client characteristics index in discovered service information. */
    uint8_t                      state;             /**< Client state. */
    tail_light_handles_t         tail_handles;    
    beacon_handles_t             beacon_handles;
    uint8_t                      tail_light_state;
    uint8_t                      needs_sync;
    uint8_t                      connected;
    uint8_t                      client_type;
} client_t;

struct  // this maps to uint32_t bitmap downloaded from beacon
 {
  uint32_t config_bitmap;
  uint8_t light_sensitivity;      // bits 0 - 2
  uint8_t motion_sensitivity;     // bits 3 - 5
  uint8_t idle_check_window;           // bits 6 - 8
  uint8_t ble_peer_count;         // bits 9 - 11
  uint8_t auto_mode;              // bit 20
  uint8_t use_beacon;             // bit 21
  uint8_t front_mode;             // bits 22 - 23
  uint8_t tail1_mode;             // bits 24 - 25
  uint8_t tail2_mode;             // bits 26 - 27
  uint8_t tail3_mode;             // bits 28 - 29
  uint8_t tail4_mode;             // bits 30 - 31
 } G_config_data;

#define CLIENT_TYPE_TAIL_LIGHT 10
#define CLIENT_TYPE_BEACON 20

// state of PWM signal controlling LED's

#define PWM_UNDEFINED     0
#define PWM_ON_CONSTANT   1
#define PWM_ON_FLASHING   2
#define PWM_ON_ALTERNATE  3
#define PWM_ON_UNKNOWN    4
#define PWM_OFF           5

#define LIGHT_ON          10
#define LIGHT_OFF         20

#define SCAN_INTERVAL                    0x02C0                                         /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                      0x0060                                         /**< Determines scan window in units of 0.625 millisecond. */

client_t                  G_client[MAX_CLIENTS];      /**< Client context information list. */
uint8_t                   G_client_count;             /**< Number of clients. */
uint8_t                   m_base_uuid_type_tail;           /**< UUID type. */
uint8_t                   m_base_uuid_type_beacon;           /**< UUID type. */

volatile uint8_t          G_PWM_light_state_prev;
volatile uint8_t          G_PWM_light_state;
volatile uint8_t          G_beacon_connected;
volatile uint8_t          G_beacon_disconnected;
volatile uint8_t          G_peer_count;
volatile uint8_t          G_config_needs_sync;
volatile uint8_t          G_BLE_read_response_arrived[MAX_CLIENTS];

uint8_t write_gatt_char_uint8_t(uint16_t conn_handle, uint16_t char_descriptor, uint8_t val);
void blink_debug_led(uint8_t b_long, uint8_t b_short);
uint8_t tail_light_sync(client_t *client);

#endif // CLIENT_HANDLING_H__

/** @} */
