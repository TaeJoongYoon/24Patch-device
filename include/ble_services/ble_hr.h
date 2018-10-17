
//hr service

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

////996badf7-aeea-4a83-b23a-e4ce62bde7f0
#define HR_SERVICE_UUID_BASE         {0xF0, 0xE7, 0xBD, 0x62, 0xCE, 0xE4, 0x3A, 0xB2, \
                                          0x83, 0x4A, 0xEA, 0xAE, 0xF7, 0xAD, 0x6B, 0x99}

//588abbde-d492-4ec4-9842-d0c2209da1bc
//#define HR_SERVICE_UUID_BASE         {0xBC, 0xA1, 0x9D, 0x20, 0xC2, 0xD0, 0x42, 0x98, \
//                                          0xC4, 0x4E, 0x92, 0xD4, 0xDE, 0xBB, 0x8A, 0x58}


#define HR_SERVICE_UUID		0x1500
#define HR_CHAR_UUID		0x1501

#define BLE_HR_DEF(_name)                                                                          \
static ble_hr_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_hr_on_ble_evt, &_name)

typedef struct ble_hr_s ble_hr_t;		

typedef enum
{
	BLE_HR_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_HR_EVT_NOTIFICATION_DISABLED,                            /**< Custom value notification disabled event. */
    BLE_HR_EVT_DISCONNECTED,
    BLE_HR_EVT_CONNECTED
} ble_hr_evt_type_t;
										  
typedef struct
{
    ble_hr_evt_type_t evt_type;                                  /**< Type of event. */
} ble_hr_evt_t;										  

typedef void (*ble_hr_evt_handler_t) (ble_hr_t * p_bas, ble_hr_evt_t * p_evt);

typedef struct
{
	ble_hr_evt_handler_t         evt_handler;
    uint8_t                       initial_hr_value;          /**< Initial hr value */
    ble_srv_cccd_security_mode_t  hr_value_char_attr_md;     /**< Initial security level for hr characteristics attribute */
} ble_hr_init_t;

struct ble_hr_s
{
	ble_hr_evt_handler_t         evt_handler;
    uint16_t                      service_handle;                 /**< Handle of hr Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      hr_raw_handles;           /**< Handles related to the hr Value characteristic. */       
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};

uint32_t ble_hr_init(ble_hr_t * p_hr, const ble_hr_init_t * p_hr_init);

void ble_hr_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);

uint32_t ble_hr_hr_value_update(ble_hr_t * p_hr, uint8_t hr_value);
