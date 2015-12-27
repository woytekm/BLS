#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "twi1_master.h"
#include "twi2_master.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nrf_gpiote.h"

#include "twi1_config.h"
#include "twi2_config.h"
#include "twi_wrappers.h"

#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_advdata.h"

#include "softdevice_handler.h"

#include "client_handling.h"

#include "pstorage.h"
#include "pstorage_platform.h"

#include "SEGGER_RTT.h"

#ifdef SOFTDEVICE_PRESENT

#define SEC_PARAM_BOND                   1                                              /**< Perform bonding. */
#define SEC_PARAM_MITM                   1                                              /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                           /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                              /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                              /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                             /**< Maximum encryption key size. */

#define MIN_CONNECTION_INTERVAL          MSEC_TO_UNITS(30, UNIT_1_25_MS)                /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL          MSEC_TO_UNITS(60, UNIT_1_25_MS)                /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY                    0                                              /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT              MSEC_TO_UNITS(4000, UNIT_10_MS)                /**< Determines supervision time-out in units of 10 millisecond. */

#define TAIL_LIGHT_DEV_NAME                  "BLTailLight"                                    /**< Target device name that application is looking for. */
#define BEACON_DEV_NAME                      "BLBeacon"

#define MAX_PEER_COUNT                   DEVICE_MANAGER_MAX_CONNECTIONS                 /**< Maximum number of peer's application intends to manage. */




#endif

//#include "nrf_gpiote.h"
//#include "nrf_drv_gpiote.h"
#include "app_gpiote.h"

//#include "nrf_adc.h"

#define DEBUG_LED_PIN 29
#define LIGHT_CONTROL_PIN 11
#define PWM_SENSE_PIN 9
#define ACCEL_I2C_ENA_PIN 3
#define ACCEL_INT1_PIN 17
#define ACCEL_INT2_PIN 5
#define ACCEL_ADDR_PIN 1

#define ACCEL_THRESHOLD 25

#define PWM_HIGH 650
#define PWM_LOW  300

#define STATE_AUTO   1
#define STATE_MANUAL 2

#define AUTO_ON 1
#define AUTO_OFF 2

#ifndef NRF_APP_PRIORITY_HIGH
#define NRF_APP_PRIORITY_HIGH 1
#endif

#ifndef NRF_APP_PRIORITY_LOW
#define NRF_APP_PRIORITY_LOW 3
#endif

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)     
#define SCHED_QUEUE_SIZE                10                      

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS             4                                          /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE          2                                         /**< Size of timer operation queues. */
#define MAINLOOP_WAKEUP_INTERVAL         APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)
#define BLE_DISCOVERY_INTERVAL           APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define LIGHT_MODE_TIMER_DURATION        APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)

volatile uint8_t g_wakeup;
app_gpiote_user_id_t m_app_gpiote_my_id;
static volatile uint32_t G_PWM_pulse_count;
static volatile uint32_t G_PWM_pulse_count_prev;
app_timer_id_t  G_mainloop_timer;
app_timer_id_t  G_ble_discovery_timer;
app_timer_id_t  G_light_mode_sleep_timer;
static volatile uint8_t G_light_mode_sleep_timer_flag = 0;
volatile uint32_t G_accel_int_1_fired_up;
uint8_t  G_current_light_state_cause;
uint8_t  G_prev_light_state_cause;
uint8_t  G_auto_triggered;
uint8_t  G_automatic_state;
uint8_t  G_main_loop_go;
uint8_t  G_scan_stopped_by_discovery;

void blink_debug_led(uint8_t b_long, uint8_t b_short)
 {

  uint8_t i;

  for(i = 0; i < b_long; i++)
   {
     nrf_gpio_pin_write(DEBUG_LED_PIN,1);
     nrf_delay_ms(200);
     nrf_gpio_pin_write(DEBUG_LED_PIN,0);
     nrf_delay_ms(250);
   }

  for(i = 0; i < b_short; i++)
   {
     nrf_gpio_pin_write(DEBUG_LED_PIN,1);
     nrf_delay_ms(50);
     nrf_gpio_pin_write(DEBUG_LED_PIN,0);
     nrf_delay_ms(100);
   }

  //nrf_delay_ms(1000);

 }

static void light_mode_sleep_timer_handler(void * p_context)
 {
   G_light_mode_sleep_timer_flag = 0;
 }

static void mainloop_timer_handler(void)
 {
   G_main_loop_go = 1;
 }

static void PWM_impulse_counter_reset(void)
 {
    NRF_TIMER2->TASKS_CLEAR = 1;
 }

static void PWM_impulse_counter_check(void)
 {

   NRF_TIMER2->TASKS_CAPTURE[0] = 1;
   G_PWM_pulse_count  = NRF_TIMER2->CC[0];
   //SEGGER_RTT_printf(0, "RTT DEBUG: G_PWM_pulse_count: %d\n",G_PWM_pulse_count);

   G_PWM_light_state_prev = G_PWM_light_state;

   if(G_PWM_pulse_count == 0)
    G_PWM_light_state =  PWM_OFF;
   else if(G_PWM_pulse_count >= 10)
    G_PWM_light_state =  PWM_ON_CONSTANT;
   else if(G_PWM_pulse_count < 10)
    G_PWM_light_state = PWM_ON_FLASHING;

   if(G_PWM_light_state != G_PWM_light_state_prev)
    {
      SEGGER_RTT_printf(0, "RTT DEBUG: light state change detected. Current state: %d\n",G_PWM_light_state);
    }

   G_PWM_pulse_count = 0;

 }


#ifdef SOFTDEVICE_PRESENT

typedef struct
{
    uint8_t     * p_data;                                                      /**< Pointer to data. */
    uint16_t      data_len;                                                    /**< Length of data. */
}data_t;

static dm_application_instance_t          m_dm_app_id;                         /**< Application identifier. */

static bool                               m_memory_access_in_progress = false; /**< Flag to keep track of ongoing operations on persistent memory. */

/**
 * @brief Scan parameters requested for scanning and connection.
 */

static const ble_gap_scan_params_t m_scan_param =
  {
     0,                       // Active scanning not set.
     0,                       // Selective scanning not set.
     NULL,                    // White-list not set.
     (uint16_t)SCAN_INTERVAL, // Scan interval.
     (uint16_t)SCAN_WINDOW,   // Scan window.
     0                        // Never stop scanning unless explicitly asked to.
  };

static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,   // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,   // Maximum connection
    0,                                   // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT        // Supervision time-out
};

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


static void scan_start(void)
{
    uint32_t err_code;
    uint32_t count;
    // Verify if there is any flash access pending, if yes delay starting scanning until
    // it's complete.
    err_code = pstorage_access_status_get(&count);
    
    //APP_ERROR_CHECK(err_code);

    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }
    err_code = sd_ble_gap_scan_start(&m_scan_param);
    //APP_ERROR_CHECK(err_code);
}

static ret_code_t device_manager_event_handler(const dm_handle_t    * p_handle,
                                                 const dm_event_t     * p_event,
                                                 const ret_code_t     event_result)
{
    uint32_t       err_code;
    //ble_gap_addr_t *p_peer_addr;

    switch(p_event->event_id)
    {
        case DM_EVT_CONNECTION:
            //p_peer_addr = &p_event->event_param.p_gap_param->params.connected.peer_addr;
            G_peer_count++;
            err_code = client_handling_create(p_handle, p_event->event_param.p_gap_param->conn_handle);

            //APP_ERROR_CHECK(err_code);
             
            if (G_peer_count < MAX_PEER_COUNT)
              G_scan_stopped_by_discovery = 1;

            SEGGER_RTT_printf(0, "RTT DEBUG: BLE peer conected. peer_count now %d\n",G_peer_count);
            break;
        case DM_EVT_DISCONNECTION:
            err_code = client_handling_destroy(p_handle);
            //APP_ERROR_CHECK(err_code);
            G_peer_count--;
            if (G_peer_count < MAX_PEER_COUNT)
            {
             if(G_config_data.ble_peer_count > 0)
               {
                if(G_peer_count < G_config_data.ble_peer_count)
                 scan_start();
               }
              else
               scan_start();
            }
            SEGGER_RTT_printf(0, "RTT DEBUG: BLE peer disconnected. peer_count now %d\n",G_peer_count);
            break;
        case DM_EVT_SECURITY_SETUP:
        {
            dm_handle_t handle = (*p_handle);
            err_code = dm_security_setup_req(&handle);
            //APP_ERROR_CHECK(err_code);
            break;
        }
        case DM_EVT_SECURITY_SETUP_COMPLETE:
            break;
        case DM_EVT_LINK_SECURED:
            break;
        case DM_EVT_DEVICE_CONTEXT_LOADED:
            break;
        case DM_EVT_DEVICE_CONTEXT_STORED:
            break;
        case DM_EVT_DEVICE_CONTEXT_DELETED:
            break;
        default:
            break;
    }

    // Relay the event to client handling module.
    err_code = client_handling_dm_event_handler(p_handle, p_event, event_result);
    //APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index+1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index+2];
            p_typedata->data_len = field_length-1;
            return NRF_SUCCESS;
        }
        index += field_length+1;
    }
    return NRF_ERROR_NOT_FOUND;
}


static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t        err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            data_t adv_data;
            data_t type_data;

            // Initialize advertisement report for parsing.
            adv_data.p_data = p_ble_evt->evt.gap_evt.params.adv_report.data;
            adv_data.data_len = p_ble_evt->evt.gap_evt.params.adv_report.dlen;

            err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                      &adv_data,
                                      &type_data);
            if (err_code != NRF_SUCCESS)
            {
                // Compare short local name in case complete name does not match.
                err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
                                          &adv_data,
                                          &type_data);
            }

            // Verify if short or complete name matches target.
            if ((err_code == NRF_SUCCESS) && (
               (memcmp(TAIL_LIGHT_DEV_NAME,type_data.p_data,type_data.data_len) == 0)|| 
               (memcmp(BEACON_DEV_NAME,type_data.p_data,type_data.data_len) == 0) ))
            {
                err_code = sd_ble_gap_scan_stop();
                if (err_code != NRF_SUCCESS)
                {
                    //APPL_LOG("[APPL]: Scan stop failed, reason %d\r\n", err_code);
                }

                err_code = sd_ble_gap_connect(&p_ble_evt->evt.gap_evt.params.adv_report.\
                                              peer_addr,
                                              &m_scan_param,
                                              &m_connection_param);

                if (err_code != NRF_SUCCESS)
                {
                    //APPL_LOG("[APPL]: Connection Request Failed, reason %d\r\n", err_code);
                }
            }
            break;
        }
        case BLE_GAP_EVT_TIMEOUT:
            if(p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                //APPL_LOG("[APPL]: Scan Timedout.\r\n");
            }
            else if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                //APPL_LOG("[APPL]: Connection Request Timedout.\r\n");
            }
            break;
        default:
            break;
    }
}


static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    //SEGGER_RTT_printf(0, "RTT DEBUG: BLE EVT code: %d\n",p_ble_evt->header.evt_id);
    dm_ble_evt_handler(p_ble_evt);
    client_handling_ble_evt_handler(p_ble_evt);
    on_ble_evt(p_ble_evt);
}

static void on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        default:
            // No implementation needed.
            break;
    }
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}


static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#ifdef S130
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = false;
#ifdef S120
    ble_enable_params.gap_enable_params.role              = BLE_GAP_ROLE_CENTRAL;
#endif

    err_code = sd_ble_enable(&ble_enable_params);
    //APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    //APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    //APP_ERROR_CHECK(err_code);
}


static void device_manager_init(void)
{
    dm_application_param_t param;
    dm_init_param_t        init_param;

    uint32_t err_code;

    err_code = pstorage_init();
    //APP_ERROR_CHECK(err_code);

    init_param.clear_persistent_data = false;

#ifdef BOND_DELETE_ALL_BUTTON_PIN
    // Clear all bonded devices if user requests to.
    init_param.clear_persistent_data =
        ((nrf_gpio_pin_read(BOND_DELETE_ALL_BUTTON_PIN) == 0)? true: false);
#endif

    err_code = dm_init(&init_param);
    //APP_ERROR_CHECK(err_code);

    memset(&param.sec_param, 0, sizeof (ble_gap_sec_params_t));

    // Event handler to be registered with the module.
    param.evt_handler            = device_manager_event_handler;

    // Service or protocol context for device manager to load, store and apply on behalf of application.
    // Here set to client as application is a GATT client.
    param.service_type           = DM_PROTOCOL_CNTXT_GATT_CLI_ID;

    // Secuirty parameters to be used for security procedures.
    param.sec_param.bond         = SEC_PARAM_BOND;
    param.sec_param.mitm         = SEC_PARAM_MITM;
    param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    param.sec_param.oob          = SEC_PARAM_OOB;
    param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    param.sec_param.kdist_periph.enc = 1;
    param.sec_param.kdist_periph.id  = 1;

    err_code = dm_register(&m_dm_app_id,&param);
    //APP_ERROR_CHECK(err_code);
}


#endif

static void pulse_light_state(void)
 {
   nrf_gpio_pin_write(LIGHT_CONTROL_PIN,1);
   nrf_delay_ms(1000);
   nrf_gpio_pin_write(LIGHT_CONTROL_PIN,0);
 }

void pulse_light_mode(uint8_t times)
 {
   uint8_t i;
   for(i=0; i < times; i++)
    {
       nrf_gpio_pin_write(LIGHT_CONTROL_PIN,1);
       nrf_delay_ms(100);
       nrf_gpio_pin_write(LIGHT_CONTROL_PIN,0);
    }
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

#define BUS1 1
#define BUS2 2


static void timer2_init(void)
{
   NRF_TIMER1->MODE  = TIMER_MODE_MODE_Counter;
   NRF_TIMER1->TASKS_CLEAR = 1;        // Sets Count to 0.
   NRF_TIMER1->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
   NRF_TIMER1->TASKS_START = 1;        // Start Counting

   NRF_TIMER2->MODE  = TIMER_MODE_MODE_Counter;
   NRF_TIMER2->TASKS_CLEAR = 1;        // Sets Count to 0.
   NRF_TIMER2->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
   NRF_TIMER2->TASKS_START = 1;        // Start Counting
}

static void gpiote_init(void)
{
#define APP_GPIOTE_MAX_USERS 1
   nrf_gpio_cfg_sense_input(ACCEL_INT1_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
   nrf_gpiote_event_configure(0, PWM_SENSE_PIN, GPIOTE_CONFIG_POLARITY_Toggle);
   nrf_gpiote_event_enable(0);
   nrf_gpiote_event_configure(1, ACCEL_INT1_PIN, GPIOTE_CONFIG_POLARITY_Toggle);
   nrf_gpiote_event_enable(1);
}

static void ppi_init(void)
{
   sd_ppi_channel_assign(0, &NRF_GPIOTE->EVENTS_IN[0], &NRF_TIMER2->TASKS_COUNT);
   sd_ppi_channel_enable_set(PPI_CHENCLR_CH0_Msk);
   sd_ppi_channel_assign(1, &NRF_GPIOTE->EVENTS_IN[1], &NRF_TIMER1->TASKS_COUNT);
   sd_ppi_channel_enable_set(PPI_CHENCLR_CH0_Msk | PPI_CHENCLR_CH1_Msk);
}

static void power_manage(void)
{
  sd_app_evt_wait();
}

uint16_t G_light_sens_lvls_low[] = {80,100,120,150,180,200};
uint16_t G_light_sens_lvls_high[] = {200,300,350,400,450,600};
uint8_t G_mot_sens_lvls[] = {12,15,18,20,25,30};
uint16_t G_idle_lvls[] = {100,130,180,210,250,300};

void map_config_bitmap(void)
 {
   G_config_data.light_sensitivity = G_config_data.config_bitmap & 7;
   G_config_data.motion_sensitivity = (G_config_data.config_bitmap >> 3) & 7;
   G_config_data.idle_check_window = (G_config_data.config_bitmap >> 6) & 7;
   G_config_data.ble_peer_count = (G_config_data.config_bitmap >> 9) & 7;
   G_config_data.auto_mode = (G_config_data.config_bitmap >> 20) & 1;
   G_config_data.use_beacon = (G_config_data.config_bitmap >> 21) & 1;
   G_config_data.front_mode = (G_config_data.config_bitmap >> 22) & 3;
   G_config_data.tail1_mode = (G_config_data.config_bitmap >> 24) & 3;  
   G_config_data.tail2_mode = (G_config_data.config_bitmap >> 26) & 3;         
   G_config_data.tail3_mode = (G_config_data.config_bitmap >> 28) & 3;         
   G_config_data.tail4_mode = (G_config_data.config_bitmap >> 30) & 3;        

   SEGGER_RTT_printf(0, "RTT DEBUG: mapped settings: light sens: %d, motion sens: %d, idle: %d, ble_peers: %d\n",G_config_data.light_sensitivity, 
                             G_config_data.motion_sensitivity, G_config_data.idle_check_window, G_config_data.ble_peer_count);

 }


void power_off(void)
 {

   uint8_t i;

   SEGGER_RTT_printf(0, "RTT DEBUG: powering system off\n");

   if(G_PWM_light_state != PWM_OFF)
    {

               pulse_light_state();  // my light off
               PWM_impulse_counter_reset();
               nrf_delay_ms(550);
               PWM_impulse_counter_check();

               for(i = 0; i < MAX_CLIENTS; i++)
                {
                  if((G_client[i].connected) && (G_client[i].client_type == CLIENT_TYPE_TAIL_LIGHT))
                   {
                    SEGGER_RTT_printf(0, "RTT DEBUG: syncing BLE peer id %d (tail light)\n",i);
                    if(tail_light_sync(&G_client[i]) == NRF_SUCCESS)
                     G_client[i].needs_sync = 0;
                   }
                }
     }


   NRF_TIMER2->TASKS_STOP = 1;  // turn off PWM counter
   nrf_gpiote_event_disable(0);
   nrf_gpiote_event_disable(1);
   sd_ppi_channel_enable_clr(PPI_CHENCLR_CH0_Msk | PPI_CHENCLR_CH1_Msk);

   NRF_GPIOTE->EVENTS_IN[0] = 0;
   NRF_GPIOTE->EVENTS_IN[1] = 0;

   nrf_gpio_cfg_sense_input(ACCEL_INT1_PIN,
                            NRF_GPIO_PIN_PULLDOWN,
                            NRF_GPIO_PIN_SENSE_HIGH);

   writeI2cReg(BUS2,0x72,0x80,0x00);  // disable ALS

   // Go to system-off mode (this function will not return; wakeup will cause a reset)
   blink_debug_led(1,0);
   sd_power_system_off();

}

#define ALS_AVG_SEC 10
#define ALS_DEFAULT_DARK_THRESHOLD 120
#define ALS_DEFAULT_BRIGHT_THRESHOLD 400

#define DEFAULT_MOVEMENT_CHECK_WINDOW 120
#define BLE_CONNECTION_INDICATOR_WINDOW 20

#define INITIAL_BLE_SCAN_TIME 60
#define BLE_SCAN_STOP 4
#define BLE_SCAN_RESTART 8

/**
 * @brief Function for application main entry.
 */
int main(void)
{

  uint16_t loop_cntr = 0;
  uint8_t first_loop;
  uint8_t ALS_byte;
  uint16_t ALS_data;
  uint16_t ALS_data_avg = 0;
  uint16_t ALS_dark_threshold;
  uint16_t ALS_bright_threshold;
  uint16_t Xdat;
  uint16_t Ydat;
  uint16_t Zdat;
  uint32_t Xdat_avg_sum;
  uint16_t Xdat_avg;
  uint8_t accel_reg_dat;
  uint32_t ALS_avg_sum = 0;
  uint32_t movement_check_loop = 0;
  uint16_t movement_check_window = DEFAULT_MOVEMENT_CHECK_WINDOW;
  uint32_t ble_connection_indicator_loop = 0;
  uint32_t ble_scan_break_loop_1 = 0;
  uint32_t ble_scan_break_loop_2 = 0;
  uint32_t error_code;
  uint8_t i;

  SEGGER_RTT_printf(0, "RTT DEBUG: system waking up (front) ...\n");

  first_loop = 1;
  G_scan_stopped_by_discovery = 0;
  G_beacon_connected = 0;
  G_beacon_disconnected = 0;
  G_peer_count = 0;
  G_accel_int_1_fired_up = 0;
  G_PWM_light_state_prev = PWM_OFF;
  G_PWM_light_state = PWM_OFF;

  nrf_gpio_cfg_output(DEBUG_LED_PIN);
  nrf_gpio_cfg_output(LIGHT_CONTROL_PIN);
  nrf_gpio_cfg_output(ACCEL_I2C_ENA_PIN);
  nrf_gpio_cfg_output(ACCEL_ADDR_PIN);

  gpiote_init();
  timer2_init();
  ppi_init();

  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
  app_timer_create(&G_mainloop_timer, APP_TIMER_MODE_SINGLE_SHOT, mainloop_timer_handler);
  app_timer_create(&G_light_mode_sleep_timer, APP_TIMER_MODE_SINGLE_SHOT, light_mode_sleep_timer_handler);

 if(!twi1_master_init())  // accelerometer on BUS1
  while(1)
   {
    blink_debug_led(1,3);  // signal TWI/I2C init failure - loop forever
    nrf_delay_ms(1000);
   }

 if(!twi2_master_init())  // ambient light sensor on BUS2
  while(1)
   {
    blink_debug_led(1,3);  // signal TWI/I2C init failure - loop forever
    nrf_delay_ms(1000);
   }

  nrf_gpio_pin_write(ACCEL_I2C_ENA_PIN,1);
  nrf_gpio_pin_write(ACCEL_ADDR_PIN,1);

  nrf_delay_ms(50);

  // while communicating with i2c device using these routines, one should remember that i2c address of the slave
  // should be shifted one bit left and LSB should be set to 0 or 1 for read/write

  // enable LIS3DH accelerometer
  writeI2cReg(BUS1,0x32,0x20,0x3F);

  // program interrupt on an accelerometer

  writeI2cReg(BUS1,0x32,0x22,0x41); // enable interrupt 1
  writeI2cReg(BUS1,0x32,0x25,0x00);
  writeI2cReg(BUS1,0x32,0x30,0xAA); // enable interrupts on all three axis
  writeI2cReg(BUS1,0x32,0x32,0x14); // interrupt threshold for all three axis + AND combination of INT events

  // initialize TMG3993 ambient light sensor

  writeI2cReg(BUS2,0x72,0x80,0x07);  // power on, ALS on, WAIT feature enabled
  writeI2cReg(BUS2,0x72,0x83,0x00);  // WAIT time 712ms
  writeI2cReg(BUS2,0x72,0x8F,0x02);  // ALS gain
  writeI2cReg(BUS2,0x72,0x81,0xDB);  // integration time 103 ms

  blink_debug_led(0,2); // signal system ready
 
//
// if system will crash and then reset with light still on, and G_auto_triggered would be initially 0 - system would think that light is manually turned on and
// enter manual mode. We don't want this, so we will set G_auto_trigger initially to 1 and after first loop we will reset it to 0
//

G_auto_triggered = 1; 

G_current_light_state_cause = STATE_AUTO;
G_prev_light_state_cause = STATE_AUTO;
G_automatic_state = AUTO_ON;

ALS_dark_threshold = ALS_DEFAULT_DARK_THRESHOLD;
ALS_bright_threshold = ALS_DEFAULT_BRIGHT_THRESHOLD;

ble_stack_init();
client_handling_init();
device_manager_init();
// Start scanning for devices.
scan_start();

PWM_impulse_counter_reset();
nrf_delay_ms(600);
PWM_impulse_counter_check();

SEGGER_RTT_printf(0, "RTT DEBUG: system ready - entering main loop\n");

  while(1)
   {
  
    loop_cntr++;

    if(G_config_needs_sync)
     {

      for(i = 0; i < MAX_CLIENTS; i++)
       {
        if((G_client[i].connected) && (G_client[i].client_type == CLIENT_TYPE_BEACON))
         beacon_config_sync(&G_client[i]);
       }

      SEGGER_RTT_printf(0, "RTT DEBUG: new config bitmap: %d\n",G_config_data.config_bitmap);

      map_config_bitmap();
     
      if(G_config_data.light_sensitivity <= 5)
       {
        SEGGER_RTT_printf(0, "RTT DEBUG: setting ALS sensitivity to %d (low), %d (high)\n",
                          G_light_sens_lvls_low[G_config_data.light_sensitivity], G_light_sens_lvls_high[G_config_data.light_sensitivity]);
        ALS_dark_threshold =  G_light_sens_lvls_low[G_config_data.light_sensitivity];
        ALS_bright_threshold = G_light_sens_lvls_high[G_config_data.light_sensitivity];
       }

      if(G_config_data.idle_check_window <= 5)
       {
        SEGGER_RTT_printf(0, "RTT DEBUG: setting idle timeout to %d seconds\n",
                          G_idle_lvls[G_config_data.idle_check_window]);
        movement_check_window =  G_idle_lvls[G_config_data.idle_check_window];
       }

 
      G_config_needs_sync = 0;
     }

    if( ((G_PWM_light_state_prev != PWM_OFF) && (G_PWM_light_state == PWM_OFF)) || ((G_PWM_light_state_prev == PWM_OFF) && (G_PWM_light_state != PWM_OFF)) )
     {

      SEGGER_RTT_printf(0, "RTT DEBUG: light state changed from %d to %d\n",G_PWM_light_state_prev, G_PWM_light_state);

      if(!G_auto_triggered) 
       {
        SEGGER_RTT_printf(0, "RTT DEBUG: manual switch detected\n");
        G_prev_light_state_cause = G_current_light_state_cause;
        G_current_light_state_cause = STATE_MANUAL;  // manual switch on/off

        if((G_current_light_state_cause == STATE_MANUAL) && (G_prev_light_state_cause == STATE_AUTO))
         {
          G_automatic_state = AUTO_OFF;
          SEGGER_RTT_printf(0, "RTT DEBUG: automatic operation off\n");
         }
        else if(G_prev_light_state_cause == STATE_MANUAL)
         {
          G_automatic_state = AUTO_ON;
          SEGGER_RTT_printf(0, "RTT DEBUG: automatic operation on\n");
         }
       }
      else
       {
        G_auto_triggered = 0;  // clear auto triggered flag, G_current_light_state_cause is already set in previous loop to AUTO
        SEGGER_RTT_printf(0, "RTT DEBUG: automatic switch detected\n");
       }

      for(i = 0; i < MAX_CLIENTS; i++)
       {
        if((G_client[i].connected) && (G_client[i].client_type == CLIENT_TYPE_TAIL_LIGHT))
         G_client[i].needs_sync = 1;
       }

     }

    // synchronize state of all tail lights
      
    for(i = 0; i < MAX_CLIENTS; i++)
     {
      if((G_client[i].needs_sync) && (G_client[i].connected) && (G_client[i].client_type == CLIENT_TYPE_TAIL_LIGHT))
       {
         SEGGER_RTT_printf(0, "RTT DEBUG: syncing BLE peer id %d (tail light)\n",i);
         if(tail_light_sync(&G_client[i]) == NRF_SUCCESS)
          G_client[i].needs_sync = 0;
       }
     } 

    ALS_data = 0;

    // read ALS
    ALS_byte = readI2cReg(BUS2,0x72,0x94);
    ALS_data = ALS_data | ALS_byte;
    ALS_byte = readI2cReg(BUS2,0x72,0x95);
    ALS_data = ALS_data | (ALS_byte<<8);

    ALS_avg_sum += ALS_data;

    accel_reg_dat = readI2cReg(BUS1,0x32,0x28);
    Xdat = accel_reg_dat;
    accel_reg_dat = readI2cReg(BUS1,0x32,0x29);
    Xdat = Xdat | (accel_reg_dat<<8);

    //accel_reg_dat = readI2cReg(BUS1,0x32,0x2A);
    //Ydat = accel_reg_dat;
    //accel_reg_dat = readI2cReg(BUS1,0x32,0x2B);
    //Ydat = Ydat | (accel_reg_dat<<8);

    //accel_reg_dat = readI2cReg(BUS1,0x32,0x2C);
    //Zdat = accel_reg_dat;
    //accel_reg_dat = readI2cReg(BUS1,0x32,0x2D);
    //Zdat = Zdat | (accel_reg_dat<<8);

    Xdat_avg_sum += Xdat;

    if(loop_cntr == ALS_AVG_SEC)
      {
       ALS_data_avg = ALS_avg_sum/ALS_AVG_SEC;
       loop_cntr = 0;
       ALS_avg_sum = 0;

       Xdat_avg = Xdat_avg_sum / ALS_AVG_SEC;
       Xdat_avg_sum = 0;

       //SEGGER_RTT_printf(0, "RTT DEBUG: ALS average: %d\n",ALS_data_avg);  
       //SEGGER_RTT_printf(0, "RTT DEBUG: X: %d, Y: %d, Z: %d\n",Xdat, Ydat, Zdat);

       if(Xdat_avg < 3000)
        power_off();
 
       if(G_beacon_connected) 
       {
       if(G_automatic_state == AUTO_ON)
        {
         if(ALS_data_avg < ALS_dark_threshold)
          {
           if(G_PWM_light_state == PWM_OFF) 
            {
             SEGGER_RTT_printf(0, "RTT DEBUG: it's dark! Light will be automatically turned on now.\n");
              pulse_light_state();  // light on
              G_prev_light_state_cause = G_current_light_state_cause;
              G_current_light_state_cause = STATE_AUTO;
              G_auto_triggered = 1;
              for(i = 0; i < MAX_CLIENTS; i++)
               {
                if(G_client[i].connected)
                 G_client[i].needs_sync = 1;
               }
            } 
          }
         else if(ALS_data_avg > ALS_bright_threshold)
          {
           if(G_PWM_light_state != PWM_OFF) 
            {
             SEGGER_RTT_printf(0, "RTT DEBUG: it's bright! Light will be automatically turned off now.\n");
             pulse_light_state();  // light off
             G_prev_light_state_cause = G_current_light_state_cause;
             G_current_light_state_cause = STATE_AUTO;
             G_auto_triggered = 1;
             for(i = 0; i < MAX_CLIENTS; i++)
              {
               if(G_client[i].connected)
                G_client[i].needs_sync = 1;
              }
            }
          }
         }
        } // if(G_beacon_connected)

      }

    movement_check_loop++;

    if(movement_check_loop > movement_check_window)
     {

       NRF_TIMER1->TASKS_CAPTURE[0] = 1;
       G_accel_int_1_fired_up  = NRF_TIMER1->CC[0];
       NRF_TIMER1->TASKS_CLEAR = 1;

       if(G_accel_int_1_fired_up > 0)
        {
         SEGGER_RTT_printf(0, "RTT DEBUG: movement check positive (%d). Resetting counter.\n",G_accel_int_1_fired_up);
         G_accel_int_1_fired_up = 0;
         movement_check_loop = 0;
        }
       else
        {
          SEGGER_RTT_printf(0, "RTT DEBUG: no movement - going to sleep\n");
          power_off();
        }
     }

    ble_connection_indicator_loop++;
    ble_scan_break_loop_1++;

    if(ble_scan_break_loop_1 > INITIAL_BLE_SCAN_TIME)
     {
      ble_scan_break_loop_2++;
      if(ble_scan_break_loop_2 == BLE_SCAN_STOP)
        sd_ble_gap_scan_stop();
      if(ble_scan_break_loop_2 == BLE_SCAN_RESTART)
       {
        sd_ble_gap_scan_start(&m_scan_param);
        ble_scan_break_loop_2 = 0;      
       }
      }

    if(G_beacon_disconnected)  // event flag
     {
           if(G_PWM_light_state != PWM_OFF)
             {
               SEGGER_RTT_printf(0, "RTT DEBUG: beacon disconnected - switching lights off\n");
               pulse_light_state();  // my light off
               PWM_impulse_counter_reset();
               nrf_delay_ms(550);
               PWM_impulse_counter_check();

               for(i = 0; i < MAX_CLIENTS; i++)
                {
                  if((G_client[i].connected) && (G_client[i].client_type == CLIENT_TYPE_TAIL_LIGHT))
                   {
                    SEGGER_RTT_printf(0, "RTT DEBUG: syncing BLE peer id %d (tail light)\n",i);
                    if(tail_light_sync(&G_client[i]) == NRF_SUCCESS)
                     G_client[i].needs_sync = 0;
                   }
                }
             }
        G_beacon_disconnected = 0;
     }

    if(ble_connection_indicator_loop > BLE_CONNECTION_INDICATOR_WINDOW)
     {
      
       for(i = 0; i < MAX_CLIENTS; i++)
        {
          if(G_client[i].connected)
           blink_debug_led(0,1);
        }
      ble_connection_indicator_loop = 0;
     }

      PWM_impulse_counter_reset();
      G_main_loop_go = 0;
      app_timer_start(G_mainloop_timer, MAINLOOP_WAKEUP_INTERVAL, NULL);
      while(!G_main_loop_go)
       {
        power_manage();
       }
      PWM_impulse_counter_check();

      if(first_loop)  // reset auto trigger flag to 0 after first loop
       {
        first_loop = 0;
        G_auto_triggered = 0;
       }

   }

}


