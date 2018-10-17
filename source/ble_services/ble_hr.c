
//hr service

#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_hr.h"
#include <string.h>
#include "nrf_gpio.h"
#include "24Patch_board.h"
#include "nrf_log.h"

static void on_write(ble_hr_t * p_hr, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
	// Custom Value Characteristic Written to.
    if (p_evt_write->handle == p_hr->hr_raw_handles.value_handle)
    {
        NRF_LOG_INFO("on Write");
    }
	
    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_hr->hr_raw_handles.cccd_handle)
        && (p_evt_write->len == 2)
       )
    {

		// CCCD written, call application event handler
		if (p_hr->evt_handler != NULL)
		{
			ble_hr_evt_t evt;

			if (ble_srv_is_notification_enabled(p_evt_write->data))
			{
				evt.evt_type = BLE_HR_EVT_NOTIFICATION_ENABLED;
			}
			else
			{
				evt.evt_type = BLE_HR_EVT_NOTIFICATION_DISABLED;
			}
			// Call the application event handler.
			p_hr->evt_handler(p_hr, &evt);
		}
	}

}

static void on_connect(ble_hr_t * p_hr, ble_evt_t const * p_ble_evt)
{
    p_hr->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
	
	ble_hr_evt_t evt;

    evt.evt_type = BLE_HR_EVT_CONNECTED;

    p_hr->evt_handler(p_hr, &evt);
}

static void on_disconnect(ble_hr_t * p_hr, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_hr->conn_handle = BLE_CONN_HANDLE_INVALID;
}

void ble_hr_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_hr_t * p_hr = (ble_hr_t *) p_context;
    
    if (p_hr == NULL || p_ble_evt == NULL)
    {
        return;
    }
	
	switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
			on_connect(p_hr, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
			on_disconnect(p_hr, p_ble_evt);
            break;
		case BLE_GATTS_EVT_WRITE:
		   on_write(p_hr, p_ble_evt);
		   break;
        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_hr_hr_value_update(ble_hr_t * p_hr, uint8_t hr_value){
    if (p_hr == NULL)
    {
        return NRF_ERROR_NULL;
    }
	
	uint32_t err_code = NRF_SUCCESS;
	ble_gatts_value_t gatts_value;

	// Initialize value struct.
	memset(&gatts_value, 0, sizeof(gatts_value));

	gatts_value.len     = sizeof(uint8_t);
	gatts_value.offset  = 0;
	gatts_value.p_value = &hr_value;

	// Update database.
	err_code = sd_ble_gatts_value_set(p_hr->conn_handle,
										p_hr->hr_raw_handles.value_handle,
										&gatts_value);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Send value if connected and notifying.
	if ((p_hr->conn_handle != BLE_CONN_HANDLE_INVALID)) 
	{
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = p_hr->hr_raw_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = gatts_value.offset;
		hvx_params.p_len  = &gatts_value.len;
		hvx_params.p_data = gatts_value.p_value;

		err_code = sd_ble_gatts_hvx(p_hr->conn_handle, &hvx_params);
	}
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;
	}

	return err_code;
}

static uint32_t hr_value_char_add(ble_hr_t * p_hr, const ble_hr_init_t * p_hr_init)
{
    uint32_t            err_code;
	
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
	
	memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
	
	memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 0;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
	
	memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_hr_init->hr_value_char_attr_md.read_perm;
    attr_md.write_perm = p_hr_init->hr_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
	
	ble_uuid.type = p_hr->uuid_type;
    ble_uuid.uuid = HR_CHAR_UUID;
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
	
	//Add hr characteristic
	err_code = sd_ble_gatts_characteristic_add(p_hr->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_hr->hr_raw_handles);
											   
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_hr_init(ble_hr_t * p_hr, const ble_hr_init_t * p_hr_init)
{
    if (p_hr == NULL || p_hr_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;
   
	// Initialize service structure
	p_hr->evt_handler               = p_hr_init->evt_handler;
	p_hr->conn_handle               = BLE_CONN_HANDLE_INVALID;
   
	// Add hr Service UUID
	ble_uuid128_t base_uuid = {HR_SERVICE_UUID_BASE};
	err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_hr->uuid_type);
	VERIFY_SUCCESS(err_code);

	ble_uuid.type = p_hr->uuid_type;
	ble_uuid.uuid = HR_SERVICE_UUID;

	// Add the HR Service
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_hr->service_handle);
	if (err_code != NRF_SUCCESS)
	{
	  return err_code;
	}

	// Add Custom Value characteristic
	return hr_value_char_add(p_hr, p_hr_init);
}
