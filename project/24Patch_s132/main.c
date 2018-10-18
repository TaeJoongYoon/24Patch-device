
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_state.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//include ppg service 
#include "ble_ppg.h"

//max30102 i2c
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "heartRate.h"
#include "spo2_algorithm.h"


#define DEVICE_NAME                         "24Patch"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "Signalus"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                    18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define PPG_RAW_INTERVAL           			APP_TIMER_TICKS(20)
#define PPG_HRS_INTERVAL           			APP_TIMER_TICKS(1000)
#define PPG_SPO2_INTERVAL          			APP_TIMER_TICKS(1000)

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(10, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(30, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

//#define LESC_DEBUG_MODE                     0                                       /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//max30102 define
#define TWI_INSTANCE_ID     0				/* TWI instance ID. 180821*/

/* Common addresses definition for MAX30102 */
#define MAX30102_ADDR 0x57

/* MAX30102 register addresses */
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF

//gpio pin config
#define PPG_INT 0

//define ppg service instance
BLE_PPG_DEF(m_ppg);

NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

APP_TIMER_DEF(m_ppg_raw_timer_id);
APP_TIMER_DEF(m_ppg_hrs_timer_id);
APP_TIMER_DEF(m_ppg_spo2_timer_id);

static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
	{PPG_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}
};

static uint32_t m_ppg_raw = 0;
static uint8_t m_ppg_hrs = 0;
static uint8_t m_ppg_spo2 = 0;

//max30102 variable
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static uint8_t m_sample; 					/* Buffer for samples read from resistor. 180821*/
static uint8_t m_sample_FIFO[6]; 	/* Buffer for samples read from FIFO resistor. 180821*/

static volatile bool m_xfer_done = false; /* Indicates if operation on TWI has ended. 180821*/

static uint32_t RED_LED = 0; /* RES LED value 180821*/
static uint32_t IR_LED = 0; /* RES IR value 180821*/

//PBA Algorithm
static const uint8_t RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
static uint8_t rates[RATE_SIZE]; //Array of heart rates
static uint8_t rateSpot = 0;
static uint32_t lastBeat = 0; //Time at which the last beat occurred
static float beatsPerMinute;
static int beatAvg;
static uint16_t dataPPG_test[7] = {117, 111, 278, 248, 184, 97, 92};
static uint8_t dataPPG_count = 0;
extern int16_t IR_AC_Signal_Current;

//spo2 Algorithm
static uint32_t irBuffer[100]; //infrared LED sensor data
static uint32_t redBuffer[100];  //red LED sensor data

static int32_t bufferLength; //data length
static int32_t spo2; //SPO2 value
static int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
static int32_t heartRate; //heart rate value
static int8_t validHeartRate; //indicator to show if the heart rate calculation is valid


static void on_ppg_evt(ble_ppg_t * p_ppg_service, ble_ppg_evt_t * p_evt)
{
	ret_code_t err_code;
	
    switch(p_evt->evt_type)
    {
		case BLE_PPG_RAW_EVT_NOTI_ENABLED:
			err_code = app_timer_start(m_ppg_raw_timer_id, PPG_RAW_INTERVAL, NULL);
			APP_ERROR_CHECK(err_code);
            break;

        case BLE_PPG_RAW_EVT_NOTI_DISABLED:
			err_code = app_timer_stop(m_ppg_raw_timer_id);
			APP_ERROR_CHECK(err_code);
            break;
		
		case BLE_PPG_HRS_EVT_NOTI_ENABLED:
			err_code = app_timer_start(m_ppg_hrs_timer_id, PPG_HRS_INTERVAL, NULL);
			APP_ERROR_CHECK(err_code);
            break;

        case BLE_PPG_HRS_EVT_NOTI_DISABLED:
			err_code = app_timer_stop(m_ppg_hrs_timer_id);
			APP_ERROR_CHECK(err_code);
            break;
		
		case BLE_PPG_SPO2_EVT_NOTI_ENABLED:
			err_code = app_timer_start(m_ppg_spo2_timer_id, PPG_SPO2_INTERVAL, NULL);
			APP_ERROR_CHECK(err_code);
            break;

        case BLE_PPG_SPO2_EVT_NOTI_DISABLED:
			err_code = app_timer_stop(m_ppg_spo2_timer_id);
			APP_ERROR_CHECK(err_code);
            break;
		
        case BLE_PPG_EVT_CONNECTED:
            break;

        case BLE_PPG_EVT_DISCONNECTED:
			break;

        default:
			// No implementation needed.
			break;
    }
}

static void ppg_raw_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    
    m_ppg_raw = IR_LED;
	
	NRF_LOG_INFO("ppg_raw_timeout_handler after");
    err_code = ble_ppg_raw_update(&m_ppg, m_ppg_raw);
	NRF_LOG_INFO("ppg_raw_timeout_handler before: %d", err_code);
    APP_ERROR_CHECK(err_code);
}

static void ppg_hrs_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    
    // Increment the value of m_custom_value before nortifing it.
    m_ppg_hrs++;
    
    err_code = ble_ppg_hrs_update(&m_ppg, m_ppg_hrs);
    APP_ERROR_CHECK(err_code);
}

static void ppg_spo2_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    
    // Increment the value of m_custom_value before nortifing it.
    m_ppg_spo2++;
    
    err_code = ble_ppg_spo2_update(&m_ppg, m_ppg_spo2);
    APP_ERROR_CHECK(err_code);
}

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code;

        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
	NRF_LOG_INFO("advertising start");
}


static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}


static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_create(&m_ppg_raw_timer_id, APP_TIMER_MODE_REPEATED, ppg_raw_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_create(&m_ppg_hrs_timer_id, APP_TIMER_MODE_REPEATED, ppg_hrs_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_create(&m_ppg_spo2_timer_id, APP_TIMER_MODE_REPEATED, ppg_spo2_timeout_handler);
    APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("timers_init");
}


static void gap_params_init(void)
{
    ret_code_t              err_code;
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
	NRF_LOG_INFO("gap_params_init");
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
	ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("gatt_init");
}


static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


static void services_init(void)
{
    ret_code_t 		err_code;
	
    // Initialize Queued Write Module.
	nrf_ble_qwr_init_t	qwr_init = {0};
    qwr_init.error_handler = nrf_qwr_error_handler;
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
	
	// Initialize PPG Service.
	ble_ppg_init_t		ppg_init = {0};//Initialize the ppg services
	memset(&ppg_init, 0, sizeof(ppg_init));
	ppg_init.evt_handler = on_ppg_evt;
	err_code = ble_ppg_init(&m_ppg, &ppg_init);
    APP_ERROR_CHECK(err_code);
	
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ppg_init.ppg_value_char_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ppg_init.ppg_value_char_attr_md.write_perm);
	NRF_LOG_INFO("services_init");
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
//    ret_code_t err_code;

    // Start application timers.
//    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
//    APP_ERROR_CHECK(err_code);

//    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
//    APP_ERROR_CHECK(err_code);

//    err_code = app_timer_start(m_rr_interval_timer_id, RR_INTERVAL_INTERVAL, NULL);
//    APP_ERROR_CHECK(err_code);

//    err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
//    APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("application timer start");
}


static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
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
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;//m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("conn_params_init");
}


static void sleep_mode_enter(void)
{
    ret_code_t err_code;
	
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}



static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
    
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;
        
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        default:
            // No implementation needed.
            break;
    }
}


static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
	NRF_LOG_INFO("ble_stack_init");
}


static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("peer_manager_init");
}


static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = false;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
	NRF_LOG_INFO("advertising_init");
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
	NRF_LOG_INFO("log_init");
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("power_management_init");
}

__STATIC_INLINE void data_handler(uint8_t *temp)
{
	int i;
	RED_LED = 0;
	IR_LED = 0;
	
//		if(dataPPG_count > 99)
//		{
//				dataPPG_count = 0;
//		}
	//0x00 0x00 0x00 0x00 0x00 0x00 |
	for(i = 0; i < 2; i++)
		RED_LED = (RED_LED | temp[i]) << 8;
	RED_LED = (RED_LED | temp[i]);
	//redBuffer[dataPPG_count] = RED_LED;
	
	for(i = 3; i < 5; i++)
		IR_LED = (IR_LED | temp[i]) << 8;
	IR_LED = (IR_LED | temp[i]);
	//irBuffer[dataPPG_count] = IR_LED;

	//dataPPG_count++;
	
	//NRF_LOG_INFO("data_handler[RED]: %d ", RED_LED);
	//NRF_LOG_INFO("data_handler[IR]: %d ", IR_LED);
	//NRF_LOG_INFO("data_handler[millis]: %d ", millis());

	//Logging each bytes of FIFO
	//for(i =0; i < 6; i++)	
	//	NRF_LOG_INFO("data_handler[%d]: %d ",i, temp[i]);
}

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample_FIFO);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);

    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void twi_init(void)
{
	ret_code_t err_code;

    const nrf_drv_twi_config_t twi_max30102_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_max30102_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
	NRF_LOG_INFO("twi_init");
}

static bool writeRegister(uint8_t regNumber, uint8_t value){
	uint8_t valueBuffer[2];
	valueBuffer[0] = regNumber;
	valueBuffer[1] = value;
	//int32_t toSend = (int32_t)value;
	uint32_t err_code = nrf_drv_twi_tx(&m_twi, MAX30102_ADDR, valueBuffer, sizeof(valueBuffer), false);
	nrf_delay_ms(10);
	return true;
}

static uint8_t readRegister(uint8_t regNumber){
	m_xfer_done = false;

	//uint8_t resultsWhoAmI;
	uint8_t whoAmIPointer = regNumber;
	nrf_drv_twi_tx(&m_twi, MAX30102_ADDR, &whoAmIPointer, 1, true);
	nrf_delay_ms(10);
	nrf_drv_twi_rx(&m_twi, MAX30102_ADDR, &m_sample, 1);
	nrf_delay_ms(10);
	NRF_LOG_INFO("TWI 0x%x: 0x%x.", regNumber, m_sample);
	return m_sample;
}


static uint8_t readRegister_FIFO(uint8_t regNumber){
	m_xfer_done = false;

	//uint8_t resultsWhoAmI;
	uint8_t whoAmIPointer = regNumber;
	nrf_drv_twi_tx(&m_twi, MAX30102_ADDR, &whoAmIPointer, 1, true);
	nrf_delay_ms(10);
	nrf_drv_twi_rx(&m_twi, MAX30102_ADDR, m_sample_FIFO, 1);
	nrf_delay_ms(10);
	NRF_LOG_INFO("TWI 0x%x: 0x%x.", regNumber, m_sample_FIFO);
	
	return 1;
}


static void max30102_init(void)
{
	writeRegister(REG_MODE_CONFIG, 0x40);//reset
	
	writeRegister(REG_INTR_ENABLE_1, 0xc0);// INTR setting [0x02 1100 0000] A_FULL_EN, PPG_RDY_EN
	writeRegister(REG_INTR_ENABLE_2, 0x00);
	writeRegister(REG_FIFO_WR_PTR, 0x00);//FIFO_WR_PTR[4:0] 0x04
	writeRegister(REG_OVF_COUNTER, 0x00);//OVF_COUNTER[4:0] 0x05
	writeRegister(REG_FIFO_RD_PTR, 0x00);//FIFO_RD_PTR[4:0] 0x06
	writeRegister(REG_FIFO_CONFIG, 0x0f);//sample avg = 1, fifo rollover=false, fifo almost full = 17 [0x08 0000 1111] 
	writeRegister(REG_MODE_CONFIG, 0x03);//0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED [0x09 0000 0011]
	writeRegister(REG_SPO2_CONFIG, 0x27);// SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)  [0x0A 0010 0111]
	writeRegister(REG_LED1_PA, 0x24);//Choose value for ~ 7mA for LED1 [0x0C 0010 0100]
	writeRegister(REG_LED2_PA, 0x24);// Choose value for ~ 7mA for LED2 [0x0D 0010 0100]
	writeRegister(REG_PILOT_PA, 0x7f);// Choose value for ~ 25mA for Pilot LED [0x10 0111 1111]

	readRegister(REG_INTR_STATUS_1);
	readRegister(REG_INTR_STATUS_2);
	readRegister_FIFO(REG_FIFO_DATA);
	NRF_LOG_INFO("max30102_init");
}

static void read_max_data()
{
    m_xfer_done = false;

    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, MAX30102_ADDR, m_sample_FIFO, 6);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    log_init();
    timers_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
	advertising_init();
    conn_params_init();
    peer_manager_init();
	
	twi_init();//nrf_drv_twi_init(), nrf_drv_twi_enable()
	max30102_init(); //init max30102 resistor

    // Start execution.
    NRF_LOG_INFO("24Patch v1.0.0 started.");
    application_timers_start();
    advertising_start(erase_bonds);

    // Enter main loop.
    for (;;)
    {
		nrf_delay_ms(20);
		
		do
		{
			idle_state_handle();
		}while (m_xfer_done == false);
				
		read_max_data();
				
		//calHR();

//		NRF_LOG_INFO("IR_LED : %d", IR_LED);
//		NRF_LOG_INFO("Heart Rate : %d", beatsPerMinute);
//		NRF_LOG_INFO("Heart Rate Average : %d", beatAvg);
				
        NRF_LOG_FLUSH();
    }
	
	// Enter main loop.
    //for (uint8_t i = 0; i < 100; i++)
//	for (dataPPG_count = 0; dataPPG_count < 100; dataPPG_count++)
//    {
//		nrf_delay_ms(100);

//        do
//		{
//			idle_state_handle();
//		}while (m_xfer_done == false);
//			
//		read_max_data();
//		
//		//calHR();
//		NRF_LOG_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
//        NRF_LOG_FLUSH();
//    }
		
	maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
		
	while(1)
	{
		for(uint8_t i = 25; i < 100; i++)
		{
			redBuffer[i - 25] = redBuffer[i];
			irBuffer[i - 25] = irBuffer[i];
		}
		
		//dataPPG_count = 75;
		//for(uint8_t i = 75; i < 100; i++)
		for(dataPPG_count = 75; dataPPG_count < 100; dataPPG_count++)
		{
			nrf_delay_ms(100);
		
			do
			{
				idle_state_handle();
			}while (m_xfer_done == false);
		
			read_max_data();
			
			//calHR();
			
			//NRF_LOG_INFO("red: %d", redBuffer[i]);
			//NRF_LOG_INFO("ir: %d", irBuffer[i]);
			NRF_LOG_INFO("HR: %d", heartRate);
			NRF_LOG_INFO("HRvalid: %d", validHeartRate);
			
			NRF_LOG_INFO("dataPPG_count: %d", dataPPG_count);
			NRF_LOG_INFO("SPO2: %d", spo2);
			NRF_LOG_INFO("SPO2valid: %d", validSPO2);
			//NRF_LOG_INFO("PPG_INT : %d", drv_gpio_inpin_get(PPG_INT, &PPG_INT_value));
			
			NRF_LOG_FLUSH();
		}
		
		maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
		

	}
}


