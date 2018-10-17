
//PPG service

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

//996badf7-aeea-4a83-b23a-e4ce62bde7f0
#define PPG_SERVICE_UUID_BASE         {0xF0, 0xE7, 0xBD, 0x62, 0xCE, 0xE4, 0x3A, 0xB2, \
                                          0x83, 0x4A, 0xEA, 0xAE, 0xF7, 0xAD, 0x6B, 0x99}

#define PPG_SERVICE_UUID		0x1400
#define PPG_RAW_CHAR_UUID		0x1401
#define PPG_HRS_CHAR_UUID		0x1402
#define PPG_SPO2_CHAR_UUID		0x1403

#define BLE_PPG_DEF(_name)                                                                          \
static ble_ppg_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_ppg_on_ble_evt, &_name)

typedef struct ble_ppg_s ble_ppg_t;		

typedef enum
{
	BLE_PPG_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_PPG_EVT_NOTIFICATION_DISABLED,                            /**< Custom value notification disabled event. */
    BLE_PPG_EVT_DISCONNECTED,
    BLE_PPG_EVT_CONNECTED
} ble_ppg_evt_type_t;
										  
typedef struct
{
    ble_ppg_evt_type_t evt_type;                                  /**< Type of event. */
} ble_ppg_evt_t;										  

typedef void (*ble_ppg_evt_handler_t) (ble_ppg_t * p_bas, ble_ppg_evt_t * p_evt);

typedef struct
{
	ble_ppg_evt_handler_t         evt_handler;
    uint8_t                       initial_ppg_value;          /**< Initial ppg value */
    ble_srv_cccd_security_mode_t  ppg_value_char_attr_md;     /**< Initial security level for ppg characteristics attribute */
} ble_ppg_init_t;

struct ble_ppg_s
{
	ble_ppg_evt_handler_t         evt_handler;
    uint16_t                      service_handle;                 /**< Handle of ppg Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      ppg_raw_handles;           /**< Handles related to the ppg Value characteristic. */
	ble_gatts_char_handles_t      ppg_hrs_handles;           
	ble_gatts_char_handles_t      ppg_spo2_handles;           
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};

uint32_t ble_ppg_init(ble_ppg_t * p_ppg, const ble_ppg_init_t * p_ppg_init);

void ble_ppg_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);

uint32_t ble_ppg_ppg_value_update(ble_ppg_t * p_ppg, uint8_t ppg_value);
