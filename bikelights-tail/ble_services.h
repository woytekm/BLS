
#ifndef BLE_LBS_H__
#define BLE_LBS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_gpiote.h"
#include "app_scheduler.h"
#include "app_timer.h"


#define BR_UUID_BASE {0x23, 0xD1, 0xBC, 0xBC, 0xBC, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x14, 0x15, 0x00, 0x00, 0x00, 0x00}
#define BR_UUID_SERVICE 0x1523
#define BR_UUID_LIGHT_CHAR 0x1525
#define BR_UUID_LIGHT_MODE_CHAR 0x1526
#define BR_UUID_LIGHT_STATE_CHAR 0x1524
#define BR_UUID_CONFIG_BLOB_CHAR 0x1527

// Forward declaration of the ble_BR_t type. 
typedef struct ble_BR_s ble_BR_t;

typedef void (*ble_BR_light_state_handler_t) (ble_BR_t * p_BR, uint8_t new_state);
typedef void (*ble_BR_light_mode_handler_t) (ble_BR_t * p_BR, uint8_t new_state);

typedef struct
{
    ble_BR_light_state_handler_t light_state_handler;                    /**< Event handler to be called when LED characteristic is written. */
    ble_BR_light_mode_handler_t light_mode_handler;                    /**< Event handler to be called when LED characteristic is written. */
} ble_BR_init_t;



/**@brief LED Button Service structure. This contains various status information for the service. */
typedef struct ble_BR_s
{
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    light_char_handles;
    ble_gatts_char_handles_t    light_mode_char_handles;
    ble_gatts_char_handles_t    light_state_char_handles;
    uint8_t                     uuid_type;
    uint16_t                    conn_handle;
    ble_BR_light_state_handler_t light_state_handler;
    ble_BR_light_mode_handler_t light_mode_handler;
} ble_BR_t;

/**@brief Function for initializing the LED Button Service.
 *
 * @param[out]  p_BR       LED Button Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_BR_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_BR_init(ble_BR_t * p_BR, const ble_BR_init_t * p_BR_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the LED Button Service.
 *
 *
 * @param[in]   p_BR      LED Button Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_BR_on_ble_evt(ble_BR_t * p_BR, ble_evt_t * p_ble_evt);

/**@brief Function for sending a button state notification.
 */
uint32_t ble_BR_on_button_change(ble_BR_t * p_BR, uint8_t button_state);

#endif // BLE_LBS_H__

/** @} */

