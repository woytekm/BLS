#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "twi_master.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "pstorage.h"
#include "SEGGER_RTT.h"

#include "twi_config.h"
#include "twi_wrappers.h"

#ifdef SOFTDEVICE_PRESENT

#include "my_ble.h"
#include "ble_services.h"

#endif

#include "nrf_gpiote.h"
//#include "nrf_drv_gpiote.h"
#include "app_gpiote.h"

//#include "nrf_adc.h"

#define ACCEL_INT1_PIN 01

#define ACCEL_THRESHOLD 25

#ifndef NRF_APP_PRIORITY_HIGH
#define NRF_APP_PRIORITY_HIGH 1
#endif

#ifndef NRF_APP_PRIORITY_LOW
#define NRF_APP_PRIORITY_LOW 3
#endif

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)     
#define SCHED_QUEUE_SIZE                10                      

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS             (4+BSP_APP_TIMERS_NUMBER)                  /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE          2                                         /**< Size of timer operation queues. */
#define TEMPERATURE_CHECK_INTERVAL       APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) 

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static ble_BL_t                         m_BL;
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

app_gpiote_user_id_t m_app_gpiote_my_id;
volatile uint8_t connected = 0;

app_timer_id_t  G_temp_check_timer;

pstorage_handle_t       G_pstorage_handle;
pstorage_handle_t       G_pstorage_config_bitmap_block_handle;
pstorage_module_param_t G_pstorage_param;

static volatile uint32_t     G_system_config_bitmap;
static volatile uint8_t      G_config_bitmap_needs_update;

#ifdef SOFTDEVICE_PRESENT

/* event handlers */

void config_bitmap_write_handler(ble_BL_t * p_BL, uint32_t config_bitmap)
 {
   G_system_config_bitmap = config_bitmap;
   G_config_bitmap_needs_update = 1;
 }


static void sys_evt_dispatch(uint32_t sys_evt)
{
   pstorage_sys_event_handler(sys_evt);
}

/* phase II */

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/* phase III */

static void services_init(void)
{
    uint32_t err_code;
    ble_BL_init_t init;
    
    init.config_bitmap_write_handler = config_bitmap_write_handler;
    
    err_code = ble_BL_init(&m_BL, &init);
    APP_ERROR_CHECK(err_code);
}

/* phase IV */

static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    
    ble_uuid_t adv_uuids[] = {{BL_UUID_SERVICE, m_BL.uuid_type}};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

  
    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}


/* phase V */

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/* phase VI */

static void sec_params_init(void)
{
    
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/* phasev VII  */

static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);

}



static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    static ble_gap_master_id_t p_master_id;
    static ble_gap_sec_keyset_t keys_exchanged;
    uint16_t char_len = sizeof(uint32_t);
    ble_gatts_value_t config_bitmap_char_attr_val;
    uint32_t value;


    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            SEGGER_RTT_printf(0, "RTT DEBUG: BLE connection up.\n");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            config_bitmap_char_attr_val.len = char_len;
            config_bitmap_char_attr_val.offset = 0;
            value = G_system_config_bitmap; 
            config_bitmap_char_attr_val.p_value = &value;
            sd_ble_gatts_value_set(m_conn_handle,m_BL.config_bitmap_char_handles.value_handle,&config_bitmap_char_attr_val);

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            SEGGER_RTT_printf(0, "RTT DEBUG: BLE connection disconnected - reset.\n");
            nrf_delay_ms(100);
            sd_nvic_SystemReset();
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params,&keys_exchanged);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0,BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            //p_enc_info = keys_exchanged.keys_central.p_enc_key
						
            if (p_master_id.ediv == p_ble_evt->evt.gap_evt.params.sec_info_request.master_id.ediv)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, &keys_exchanged.keys_central.p_enc_key->enc_info, &keys_exchanged.keys_central.p_id_key->id_info, NULL);
                APP_ERROR_CHECK(err_code);
 	    							p_master_id.ediv = p_ble_evt->evt.gap_evt.params.sec_info_request.master_id.ediv;
            }
           else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL,NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
                SEGGER_RTT_printf(0, "RTT DEBUG: advertising timeout - going to sleep...\n");
                nrf_delay_ms(100);
                // Go to system-off mode (this function will not return; wakeup will cause a reset)                
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_BL_on_ble_evt(&m_BL, p_ble_evt);
}

/* phase I */

static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    //SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM, NULL);

    // Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    ble_gap_addr_t addr;

    err_code = sd_ble_gap_address_get(&addr);
    APP_ERROR_CHECK(err_code);
    sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr);
    APP_ERROR_CHECK(err_code);
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

#endif

static uint16_t calc_accel_delta(uint8_t prev, uint8_t curr)
 {
  uint16_t delta = 0;

  if((prev > 245) && (curr < 15)) // seems like accelerator measurement crossed 0
   delta = (254 - prev) + curr;
  else if((prev < 15) && (curr > 245)) // crossed 0 in other direction
   delta = prev + (254 - curr);
  else if(prev > curr)
   delta = (prev - curr);
  else if(curr > prev)
   delta = curr - prev;
  else if(curr == prev)
   delta = 0;

  return delta;
 }

static void gpiote_init(void)
{ 
#define APP_GPIOTE_MAX_USERS 1  
  nrf_gpio_cfg_sense_input(ACCEL_INT1_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
}

static void pstorage_event_handler(pstorage_handle_t  * handle,
                               uint8_t              op_code,
                               uint32_t             result,
                               uint8_t            * p_data,
                               uint32_t             data_len)
{
        switch(op_code)
         {
          case PSTORAGE_UPDATE_OP_CODE:
               SEGGER_RTT_printf(0, "RTT DEBUG: pstorage callback called with op code UPDATE, result: %d\n",result);
               break;

          default:
               SEGGER_RTT_printf(0, "RTT DEBUG: pstorage callback called with op code %d, result: %d\n",op_code,result);
               break;
         }

}

static void timer2_init(void)
{
   NRF_TIMER2->MODE  = TIMER_MODE_MODE_Counter;
   NRF_TIMER2->TASKS_CLEAR = 1;        // Sets Count to 0.
   NRF_TIMER2->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
   NRF_TIMER2->TASKS_START = 1;        // Start Counting
}

void BMA250_temperature_check(void)
 {

   int8_t temperature;
   int8_t BMA250_temp_reading;
   uint8_t value;
   ble_gatts_value_t temp_char_attr_val;

   BMA250_temp_reading = readI2cReg(0x30,0x08);
   temperature = (BMA250_temp_reading/2)+24.0;

   SEGGER_RTT_printf(0, "RTT DEBUG: temperature reading: %d\n",temperature);

   value = (uint8_t)temperature;
   temp_char_attr_val.len = sizeof(uint8_t);
   temp_char_attr_val.offset = 0;
   temp_char_attr_val.p_value = &value;
   sd_ble_gatts_value_set(m_conn_handle,m_BL.temp_char_handles.value_handle,&temp_char_attr_val);

 }

/**
 * @brief Function for application main entry.
 */
int main(void)
{

  uint8_t pstorage_data[4];
  uint32_t retval;

  SEGGER_RTT_printf(0, "RTT DEBUG: system waking up (beacon)...\n");

  timer2_init();
  gpiote_init();
  twi_master_init();

  G_system_config_bitmap = 0;

  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
  app_timer_create(&G_temp_check_timer, APP_TIMER_MODE_REPEATED, BMA250_temperature_check);

#ifdef SOFTDEVICE_PRESENT

  ble_stack_init();
  scheduler_init();
  gap_params_init();
  services_init();
  advertising_init();
  //conn_params_init();
  sec_params_init();

  advertising_start();

  nrf_delay_ms(100);

  pstorage_init();

  G_pstorage_param.block_size = 20;
  G_pstorage_param.block_count = 1;
  G_pstorage_param.cb = pstorage_event_handler;

  retval = pstorage_register(&G_pstorage_param, &G_pstorage_handle);
  SEGGER_RTT_printf(0, "RTT DEBUG: pstorage_register: %d\n",retval);

  retval = pstorage_block_identifier_get(&G_pstorage_handle, 0, &G_pstorage_config_bitmap_block_handle);
  SEGGER_RTT_printf(0, "RTT DEBUG: pstorage_block_identifier_get: %d\n",retval);

  retval = pstorage_load(pstorage_data, &G_pstorage_config_bitmap_block_handle, 4, 0);
  SEGGER_RTT_printf(0, "RTT DEBUG: pstorage_load: %d \n",retval);


  G_system_config_bitmap =  pstorage_data[0]|(pstorage_data[1] << 8)|(pstorage_data[2] << 16)|(pstorage_data[3] << 24);

  SEGGER_RTT_printf(0, "RTT DEBUG: loaded system config bitmap: %d\n",G_system_config_bitmap);

#endif

  // while communicating with i2c device using these routines, one should remember that i2c address of the slave
  // should be shifted one bit left and LSB should be set to 0 or 1 for read/write

  // enable accelerometer present on beacon board (Bosch BMA250)
  writeI2cReg(0x30, 0x11, 0x58);  // program low power mode with 50ms wakeup interval
  writeI2cReg(0x30, 0x16, 0x07);  // enable all axis 
  writeI2cReg(0x30, 0x21, 0x0c);  // interrupt is temporary (12,5 ms) - not latched
  writeI2cReg(0x30, 0x1e, 0x04);  // enable interrupt on a slope (any movement)
  writeI2cReg(0x30, 0x27, 0x01);  // slope duration
  writeI2cReg(0x30, 0x28, 0x20);  // slope threshold
  writeI2cReg(0x30, 0x19, 0x04);  // enable INT1

  app_timer_start(G_temp_check_timer, TEMPERATURE_CHECK_INTERVAL, NULL);

  SEGGER_RTT_printf(0, "RTT DEBUG: system ready - entering main loop\n");  

  while(1)
    {
      sd_app_evt_wait();
      if(G_config_bitmap_needs_update)
       {
        pstorage_data[0] = G_system_config_bitmap & 0xFF;
        pstorage_data[1] = (G_system_config_bitmap >> 8) & 0xFF;
        pstorage_data[2] = (G_system_config_bitmap >> 16) & 0xFF;
        pstorage_data[3] = (G_system_config_bitmap >> 24) & 0xFF;
        
        retval = pstorage_update(&G_pstorage_config_bitmap_block_handle, pstorage_data, 4, 0);
        SEGGER_RTT_printf(0, "RTT DEBUG: wrote %d to config bitmap (%d)\n",G_system_config_bitmap, retval);
        G_config_bitmap_needs_update = 0;
       }
    }

}


