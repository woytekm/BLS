
#ifndef BLE_LBS_H__
#define BLE_LBS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_gpiote.h"
#include "app_scheduler.h"
#include "app_timer.h"


#define BL_UUID_BASE {0x23, 0xD1, 0xBC, 0xBC, 0xBC, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x14, 0x15, 0x00, 0x00, 0x00, 0x10}
#define BL_UUID_SERVICE 0x10
#define BL_BASE_CHAR_UUID 0x10
#define BL_CONFIG_BITMAP_CHAR_UUID 0x11
#define BL_TEMP_CHAR_UUID 0x12

// Forward declaration of the ble_BL_t type. 
typedef struct ble_BL_s ble_BL_t;

typedef void (*ble_BL_config_bitmap_write_handler_t) (ble_BL_t * p_BL, uint32_t config_bitmap);

typedef struct
{
    ble_BL_config_bitmap_write_handler_t config_bitmap_write_handler;                    /**< Event handler to be called when LED characteristic is written. */
} ble_BL_init_t;


typedef struct ble_BL_s
{
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    config_bitmap_char_handles;
    ble_gatts_char_handles_t    temp_char_handles;
    uint8_t                     uuid_type;
    uint16_t                    conn_handle;
    ble_BL_config_bitmap_write_handler_t config_bitmap_write_handler;
} ble_BL_t;

/**@brief Function for initializing the LED Button Service.
 *
 * @param[out]  p_BL       LED Button Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_BL_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_BL_init(ble_BL_t * p_BL, const ble_BL_init_t * p_BL_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the LED Button Service.
 *
 *
 * @param[in]   p_BL      LED Button Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_BL_on_ble_evt(ble_BL_t * p_BL, ble_evt_t * p_ble_evt);

/**@brief Function for sending a button state notification.
 */
uint32_t ble_BL_on_button_change(ble_BL_t * p_BL, uint8_t button_state);

#endif // BLE_LBS_H__

/** @} */

