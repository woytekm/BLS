#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "twi_master.h"
#include "app_scheduler.h"
#include "app_timer.h"
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

#define DEBUG_LED_PIN 8
#define LIGHT_CONTROL_PIN 3
#define PWM_SENSE_PIN 5
#define ACCEL_I2C_ENA_PIN 18
#define ACCEL_INT1_PIN 9

#define ACCEL_THRESHOLD 25

// state of PWM signal controlling LED's

#define PWM_UNDEFINED     0
#define PWM_ON_CONSTANT   1
#define PWM_ON_FLASHING   2
#define PWM_ON_ALTERNATE  3
#define PWM_ON_UNKNOWN    4
#define PWM_OFF           5

#define LIGHT_ON          10
#define LIGHT_OFF         20

#define PWM_HIGH 650
#define PWM_LOW  300

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
#define LIGHT_PWM_CHECK_INTERVAL         APP_TIMER_TICKS(450, APP_TIMER_PRESCALER) 
#define LIGHT_MODE_TIMER_DURATION        APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static ble_BR_t                         m_BR;
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

volatile uint8_t G_PWM_light_state, G_PWM_light_state_prev, G_wakeup;
app_gpiote_user_id_t m_app_gpiote_my_id;
volatile uint32_t G_PWM_pulse_count;
volatile uint32_t G_PWM_pulse_count_prev;
app_timer_id_t  G_PWM_impulse_counter_timer;
app_timer_id_t  G_light_mode_sleep_timer;
volatile uint8_t connected = 0;
static volatile uint8_t G_light_mode_sleep_timer_flag = 0;
static volatile uint8_t G_requested_light_mode = 0;


static void light_mode_sleep_timer_handler(void * p_context)
 {
   G_light_mode_sleep_timer_flag = 0;
 }

static void PWM_impulse_counter_check(void * p_context)
 {

   if(G_PWM_pulse_count == 0)
    G_PWM_light_state =  PWM_OFF;
   else if(G_PWM_pulse_count >= 10)
    G_PWM_light_state =  PWM_ON_CONSTANT;
   else if(G_PWM_pulse_count < 10)
    G_PWM_light_state = PWM_ON_FLASHING;
 
   if(G_PWM_light_state != G_PWM_light_state_prev) 
    {

      if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
        ble_BR_on_light_change(&m_BR, G_PWM_light_state);
      G_PWM_light_state_prev = G_PWM_light_state;
      SEGGER_RTT_printf(0, "RTT DEBUG: light state change detected. Current state: %d\n",G_PWM_light_state); 
    }

   G_PWM_pulse_count = 0;

 }

#ifdef SOFTDEVICE_PRESENT

/* event handlers */

static void sys_evt_dispatch(uint32_t sys_evt)
{
    //pstorage_sys_event_handler(sys_evt);
}

static void light_state_handler(ble_BR_t * p_BR, uint8_t light_state)
{
 
   SEGGER_RTT_printf(0, "RTT DEBUG: light_state_handler: call with light_state: %d, G_PWM_light_state: %d\n",light_state,G_PWM_light_state);

   if(((light_state == LIGHT_ON) && (G_PWM_light_state == PWM_OFF)) ||
      ((light_state == LIGHT_OFF) && (G_PWM_light_state != PWM_OFF)) )
     {
      SEGGER_RTT_printf(0, "RTT DEBUG: light_state_handler: toggling light\n");
      nrf_gpio_pin_write(LIGHT_CONTROL_PIN,1);
      nrf_delay_ms(1000);
      nrf_gpio_pin_write(LIGHT_CONTROL_PIN,0);
     }
}

void pulse_light_state(uint8_t times)
 {
   uint8_t i;
   for(i=0; i < times; i++)
    {
       nrf_gpio_pin_write(LIGHT_CONTROL_PIN,1);
       nrf_delay_ms(50);
       nrf_gpio_pin_write(LIGHT_CONTROL_PIN,0);
       nrf_delay_ms(1000);
    }
 }

void static light_mode_handler(ble_BR_t * p_BR, uint8_t light_mode)
 {
   G_requested_light_mode = light_mode;
 }

//
// this function will change light mode - only if light is already lit
//
//
static void switch_light_mode(uint8_t light_mode)
{

   if(light_mode == PWM_ON_CONSTANT)
    {
     if((G_PWM_light_state ==  PWM_ON_CONSTANT) || (G_PWM_light_state == PWM_OFF))
      return;
     else // this means that we have a flashing light of some kind
      {
       pulse_light_state(1);

       G_light_mode_sleep_timer_flag = 1;
       app_timer_start(G_light_mode_sleep_timer, LIGHT_MODE_TIMER_DURATION, NULL);
       while(G_light_mode_sleep_timer_flag)
        {
         sd_app_evt_wait();
        }
       app_timer_stop(G_light_mode_sleep_timer);

       if(G_PWM_light_state ==  PWM_ON_CONSTANT)
        return;
       else
        pulse_light_state(1); // we should get constant light at this point (only three modes in basic cateye lamp)
      }
     }
    else if(light_mode == PWM_ON_ALTERNATE)
     {
     if(G_PWM_light_state == PWM_OFF)
      return;
     if(G_PWM_light_state ==  PWM_ON_CONSTANT)
      {
       pulse_light_state(1);
       return;
      }
     else // this means that we have a flashing light of some kind
      {
       pulse_light_state(1);

       G_light_mode_sleep_timer_flag = 1;
       app_timer_start(G_light_mode_sleep_timer, LIGHT_MODE_TIMER_DURATION, NULL);
       while(G_light_mode_sleep_timer_flag)
        {
         sd_app_evt_wait();
        }
       app_timer_stop(G_light_mode_sleep_timer);

       if(G_PWM_light_state ==  PWM_ON_CONSTANT)
        pulse_light_state(1);
       else
        pulse_light_state(2); // we should get alternate light at this point (only three modes in basic cateye lamp)
      }
     }
    else if(light_mode == PWM_ON_FLASHING)
     {
     if(G_PWM_light_state == PWM_OFF)
      return;
     if(G_PWM_light_state ==  PWM_ON_CONSTANT)
      {
       pulse_light_state(2);
       return;
      }
     else // this means that we have a flashing light of some kind
      {
       pulse_light_state(1);
      
       G_light_mode_sleep_timer_flag = 1;
       app_timer_start(G_light_mode_sleep_timer, LIGHT_MODE_TIMER_DURATION, NULL);
       while(G_light_mode_sleep_timer_flag)
        {
         sd_app_evt_wait();
        }
       app_timer_stop(G_light_mode_sleep_timer);

       if(G_PWM_light_state ==  PWM_ON_CONSTANT)
        pulse_light_state(2);
       else
        pulse_light_state(3); // we should get flashing light at this point (only three modes)
      }

     }
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
    ble_BR_init_t init;
    
    init.light_state_handler = light_state_handler;
    init.light_mode_handler = light_mode_handler;
    
    err_code = ble_BR_init(&m_BR, &init);
    APP_ERROR_CHECK(err_code);
}

/* phase IV */

static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    
    ble_uuid_t adv_uuids[] = {{BR_UUID_SERVICE, m_BR.uuid_type}};

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
    uint16_t char_len = sizeof(uint8_t);
    ble_gatts_value_t light_state_char_attr_val;
    uint8_t value;


    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:

            SEGGER_RTT_printf(0, "RTT DEBUG: BLE connection up. Updating light state char: %d\n",G_PWM_light_state);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            light_state_char_attr_val.len = char_len;
            light_state_char_attr_val.offset = 0;
            value = G_PWM_light_state;
            light_state_char_attr_val.p_value = &value;
            sd_ble_gatts_value_set(m_conn_handle,m_BR.light_state_char_handles.value_handle,&light_state_char_attr_val);

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
                // Configure buttons with sense level low as wakeup source.

                NRF_TIMER2->TASKS_STOP = 1;  // turn off PWM counter
                nrf_gpiote_event_disable(0);
                sd_ppi_channel_enable_clr(PPI_CHENCLR_CH0_Msk);
                app_timer_stop(G_PWM_impulse_counter_timer);

                NRF_GPIOTE->EVENTS_IN[0] = 0;
                NRF_GPIOTE->EVENTS_IN[1] = 0;

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
    ble_BR_on_ble_evt(&m_BR, p_ble_evt);
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

static void blink_debug_led(uint8_t b_long, uint8_t b_short)
 {

  uint8_t i;

  for(i = 0; i < b_long; i++)
   {
     nrf_gpio_pin_write(DEBUG_LED_PIN,1);  
     nrf_delay_ms(400);
     nrf_gpio_pin_write(DEBUG_LED_PIN,0);
     nrf_delay_ms(400);
   }

  for(i = 0; i < b_short; i++)
   {
     nrf_gpio_pin_write(DEBUG_LED_PIN,1); 
     nrf_delay_ms(200);
     nrf_gpio_pin_write(DEBUG_LED_PIN,0);
     nrf_delay_ms(200);
   }

  nrf_delay_ms(1000);

 }


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


static void timer2_init(void)    
{
   NRF_TIMER2->MODE  = TIMER_MODE_MODE_Counter;
   NRF_TIMER2->TASKS_CLEAR = 1;        // Sets Count to 0.
   NRF_TIMER2->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
   NRF_TIMER2->TASKS_START = 1;        // Start Counting             
}

static void gpiote_init(void)
{ 
#define APP_GPIOTE_MAX_USERS 1  
   //nrf_gpio_cfg_sense_input(PWM_SENSE_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);     
   nrf_gpio_cfg_sense_input(ACCEL_INT1_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
   nrf_gpiote_event_configure(0, PWM_SENSE_PIN, GPIOTE_CONFIG_POLARITY_Toggle);
   nrf_gpiote_event_enable(0);
   //nrf_gpiote_event_configure(1, ACCEL_INT1_PIN, GPIOTE_CONFIG_POLARITY_Toggle);
   //nrf_gpiote_event_enable(1);
}

static void ppi_init(void)
{
   sd_ppi_channel_assign(0, &NRF_GPIOTE->EVENTS_IN[0], &NRF_TIMER2->TASKS_COUNT);
   sd_ppi_channel_enable_set(PPI_CHENCLR_CH0_Msk);
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{

  uint8_t prev_light_state = PWM_OFF, curr_light_state = PWM_OFF;
  G_PWM_pulse_count = 0;
  G_PWM_pulse_count_prev = 0;

  SEGGER_RTT_printf(0, "RTT DEBUG: system waking up (tail)...\n");

  nrf_gpio_cfg_output(DEBUG_LED_PIN);
  nrf_gpio_cfg_output(LIGHT_CONTROL_PIN);
  nrf_gpio_cfg_output(ACCEL_I2C_ENA_PIN);

  gpiote_init();
  timer2_init();
  ppi_init();

  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
  app_timer_create(&G_PWM_impulse_counter_timer, APP_TIMER_MODE_REPEATED, PWM_impulse_counter_check);
  app_timer_create(&G_light_mode_sleep_timer, APP_TIMER_MODE_SINGLE_SHOT, light_mode_sleep_timer_handler);
  app_timer_start(G_PWM_impulse_counter_timer, LIGHT_PWM_CHECK_INTERVAL, NULL); 

 if(!twi_master_init())
  while(1)
   blink_debug_led(1,3);  // signal TWI/I2C init failure - loop forever

  nrf_delay_ms(600);

#ifdef SOFTDEVICE_PRESENT

  //NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  //NRF_CLOCK->TASKS_HFCLKSTART = 1;

  /* Wait for the external oscillator to start up */
  //while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}

  ble_stack_init();
  scheduler_init();
  gap_params_init();
  services_init();
  advertising_init();
  //conn_params_init();
  sec_params_init();

  advertising_start();

  nrf_delay_ms(500);

#endif

  nrf_gpio_pin_write(ACCEL_I2C_ENA_PIN,1);

  nrf_delay_ms(100);

  // while communicating with i2c device using these routines, one should remember that i2c address of the slave
  // should be shifted one bit left and LSB should be set to 0 or 1 for read/write

  // enable accelerometer
  writeI2cReg(0x3a, 0x20, 0x47);
  writeI2cReg(0x3a, 0x21, 0x40);

  // program interrupt on an accelerometer

  writeI2cReg(0x3a, 0x22, 0x01); // enable interrupt 1 (0x81)
  writeI2cReg(0x3a, 0x30, 0xAA); // enable interrupts on all three axis
  writeI2cReg(0x3a, 0x32, 0x10); // interrupt threshold for all three axis + AND combination of INT events
 // writeI2cReg(0x3a, 0x33, 0x0A); // interrupt duration

 SEGGER_RTT_printf(0, "RTT DEBUG: system ready - entering main loop\n");  

  while(1)
   {
      if(G_requested_light_mode  != 0)
       {
         switch_light_mode(G_requested_light_mode);
         G_requested_light_mode = 0;
       }

      sd_app_evt_wait();
      NRF_TIMER2->TASKS_CAPTURE[0] = 1;
      G_PWM_pulse_count  = NRF_TIMER2->CC[0];   
      NRF_TIMER2->TASKS_CLEAR = 1;
   }

}


