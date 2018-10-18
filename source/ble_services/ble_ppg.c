
//PPG service

#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_ppg.h"
#include <string.h>
#include "nrf_gpio.h"
#include "24Patch_board.h"
#include "nrf_log.h"

static void on_write(ble_ppg_t * p_ppg, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
	// Custom Value Characteristic Written to.
    if (p_evt_write->handle == p_ppg->ppg_raw_handles.value_handle)
    {
        NRF_LOG_INFO("on Write");
    }
	
    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_ppg->ppg_raw_handles.cccd_handle)
        && (p_evt_write->len == 2)
       )
    {

		// CCCD written, call application event handler
		if (p_ppg->evt_handler != NULL)
		{
			ble_ppg_evt_t evt;

			if (ble_srv_is_notification_enabled(p_evt_write->data))
			{
				evt.evt_type = BLE_PPG_RAW_EVT_NOTI_ENABLED;
			}
			else
			{
				evt.evt_type = BLE_PPG_RAW_EVT_NOTI_DISABLED;
			}
			// Call the application event handler.
			p_ppg->evt_handler(p_ppg, &evt);
		}
	}
}

static void on_connect(ble_ppg_t * p_ppg, ble_evt_t const * p_ble_evt)
{
    p_ppg->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
	
	ble_ppg_evt_t evt;

    evt.evt_type = BLE_PPG_EVT_CONNECTED;

    p_ppg->evt_handler(p_ppg, &evt);
}

static void on_disconnect(ble_ppg_t * p_ppg, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_ppg->conn_handle = BLE_CONN_HANDLE_INVALID;
}

void ble_ppg_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_ppg_t * p_ppg = (ble_ppg_t *) p_context;
    
    if (p_ppg == NULL || p_ble_evt == NULL)
    {
        return;
    }
	
	switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
			on_connect(p_ppg, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
			on_disconnect(p_ppg, p_ble_evt);
            break;
		case BLE_GATTS_EVT_WRITE:
		   on_write(p_ppg, p_ble_evt);
		   break;
        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_ppg_raw_update(ble_ppg_t * p_ppg, uint32_t ppg_value)
{
    if (p_ppg == NULL)
    {
        return NRF_ERROR_NULL;
    }
	
	uint32_t err_code = NRF_SUCCESS;
	
	ble_gatts_value_t	gatts_value;
	uint8_t				encoded_hrm[4];
	encoded_hrm[3] = (uint8_t) ((ppg_value & 0x000000FF) >> 0);
	encoded_hrm[2] = (uint8_t) ((ppg_value & 0x0000FF00) >> 8);
	encoded_hrm[1] = (uint8_t) ((ppg_value & 0x00FF0000) >> 16);
	encoded_hrm[0] = (uint8_t) ((ppg_value & 0xFF000000) >> 24);

	// Initialize value struct.
	memset(&gatts_value, 0, sizeof(gatts_value));

	gatts_value.len     = sizeof(uint32_t);
	gatts_value.offset  = 0;
	gatts_value.p_value = encoded_hrm;

	// Update database.
//	err_code = sd_ble_gatts_value_set(p_ppg->conn_handle,
//										p_ppg->ppg_raw_handles.value_handle,
//										&gatts_value);
//	if (err_code != NRF_SUCCESS)
//	{
//		NRF_LOG_INFO("err_code: %d", err_code);
//		return err_code;
//	}
	
	// Send value if connected and notifying.
	if ((p_ppg->conn_handle != BLE_CONN_HANDLE_INVALID)) 
	{
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = p_ppg->ppg_raw_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = gatts_value.offset;
		hvx_params.p_len  = &gatts_value.len;
		hvx_params.p_data = encoded_hrm;//gatts_value.p_value;

		err_code = sd_ble_gatts_hvx(p_ppg->conn_handle, &hvx_params);
		NRF_LOG_INFO("ble_ppg_raw_update::err_code:%d", err_code);
	}
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;
		NRF_LOG_INFO("ble_ppg_raw_update::err_code:%d", err_code);
	}

	return err_code;
}

uint32_t ble_ppg_hrs_update(ble_ppg_t * p_ppg, uint8_t ppg_hrs){
    if (p_ppg == NULL)
    {
        return NRF_ERROR_NULL;
    }
	
	uint32_t err_code = NRF_SUCCESS;
	ble_gatts_value_t gatts_value;

	// Initialize value struct.
	memset(&gatts_value, 0, sizeof(gatts_value));

	gatts_value.len     = sizeof(uint8_t);
	gatts_value.offset  = 0;
	gatts_value.p_value = &ppg_hrs;

	// Update database.
	err_code = sd_ble_gatts_value_set(p_ppg->conn_handle,
										p_ppg->ppg_hrs_handles.value_handle,
										&gatts_value);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Send value if connected and notifying.
	if ((p_ppg->conn_handle != BLE_CONN_HANDLE_INVALID)) 
	{
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = p_ppg->ppg_hrs_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = gatts_value.offset;
		hvx_params.p_len  = &gatts_value.len;
		hvx_params.p_data = gatts_value.p_value;

		err_code = sd_ble_gatts_hvx(p_ppg->conn_handle, &hvx_params);
	}
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;
		NRF_LOG_INFO("ble_ppg_hrs_update::err_code:%d", err_code);
	}

	return err_code;
}

uint32_t ble_ppg_spo2_update(ble_ppg_t * p_ppg, uint8_t ppg_spo2){
    if (p_ppg == NULL)
    {
        return NRF_ERROR_NULL;
    }
	
	uint32_t err_code = NRF_SUCCESS;
	ble_gatts_value_t gatts_value;

	// Initialize value struct.
	memset(&gatts_value, 0, sizeof(gatts_value));

	gatts_value.len     = sizeof(uint8_t);
	gatts_value.offset  = 0;
	gatts_value.p_value = &ppg_spo2;

	// Update database.
	err_code = sd_ble_gatts_value_set(p_ppg->conn_handle,
										p_ppg->ppg_spo2_handles.value_handle,
										&gatts_value);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Send value if connected and notifying.
	if ((p_ppg->conn_handle != BLE_CONN_HANDLE_INVALID)) 
	{
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = p_ppg->ppg_spo2_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = gatts_value.offset;
		hvx_params.p_len  = &gatts_value.len;
		hvx_params.p_data = gatts_value.p_value;

		err_code = sd_ble_gatts_hvx(p_ppg->conn_handle, &hvx_params);
	}
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;
		NRF_LOG_INFO("ble_ppg_spo2_update::err_code:%d", err_code);
	}

	return err_code;
}

static uint32_t ppg_value_char_add(ble_ppg_t * p_ppg, const ble_ppg_init_t * p_ppg_init)
{
    uint32_t            err_code;
	
    ble_gatts_char_md_t ppg_raw_char_md;
	ble_gatts_char_md_t ppg_hrs_char_md;
	ble_gatts_char_md_t ppg_spo2_char_md;
    ble_gatts_attr_md_t ppg_raw_cccd_md;
	ble_gatts_attr_md_t ppg_hrs_cccd_md;
	ble_gatts_attr_md_t ppg_spo2_cccd_md;
    ble_gatts_attr_t    attr_char_ppg_raw;
	ble_gatts_attr_t    attr_char_ppg_hrs;
	ble_gatts_attr_t    attr_char_ppg_spo2;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
	
	//Setting PPG RAW Characteristic-----------------------------------------------------
	memset(&ppg_raw_cccd_md, 0, sizeof(ppg_raw_cccd_md));

    //  Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ppg_raw_cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ppg_raw_cccd_md.write_perm);
    
    ppg_raw_cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
	
	memset(&ppg_raw_char_md, 0, sizeof(ppg_raw_char_md));

    ppg_raw_char_md.char_props.read   = 0;
    ppg_raw_char_md.char_props.write  = 0;
    ppg_raw_char_md.char_props.notify = 1; 
    ppg_raw_char_md.p_char_user_desc  = NULL;
    ppg_raw_char_md.p_char_pf         = NULL;
    ppg_raw_char_md.p_user_desc_md    = NULL;
    ppg_raw_char_md.p_cccd_md         = &ppg_raw_cccd_md; 
    ppg_raw_char_md.p_sccd_md         = NULL;
	
	memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_ppg_init->ppg_value_char_attr_md.read_perm;
    attr_md.write_perm = p_ppg_init->ppg_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
	
	ble_uuid.type = p_ppg->uuid_type;
    ble_uuid.uuid = PPG_RAW_CHAR_UUID;
	
	memset(&attr_char_ppg_raw, 0, sizeof(attr_char_ppg_raw));

    attr_char_ppg_raw.p_uuid    = &ble_uuid;
    attr_char_ppg_raw.p_attr_md = &attr_md;
    attr_char_ppg_raw.init_len  = sizeof(uint8_t);
    attr_char_ppg_raw.init_offs = 0;
    attr_char_ppg_raw.max_len   = sizeof(uint8_t);
	
	//Add PPG RAW characteristic
	err_code = sd_ble_gatts_characteristic_add(p_ppg->service_handle, &ppg_raw_char_md,
                                               &attr_char_ppg_raw,
                                               &p_ppg->ppg_raw_handles);
	
	//Setting PPG HRS Characteristic-----------------------------------------------------
//	memset(&ppg_hrs_cccd_md, 0, sizeof(ppg_hrs_cccd_md));

//    //  Read  operation on Cccd should be possible without authentication.
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ppg_hrs_cccd_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ppg_hrs_cccd_md.write_perm);
//    
//    ppg_hrs_cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
//	
//	memset(&ppg_hrs_char_md, 0, sizeof(ppg_hrs_char_md));

//    ppg_hrs_char_md.char_props.read   = 0;
//    ppg_hrs_char_md.char_props.write  = 0;
//    ppg_hrs_char_md.char_props.notify = 1;
//    ppg_hrs_char_md.p_char_user_desc  = NULL;
//    ppg_hrs_char_md.p_char_pf         = NULL;
//    ppg_hrs_char_md.p_user_desc_md    = NULL;
//    ppg_hrs_char_md.p_cccd_md         = &ppg_hrs_cccd_md; 
//    ppg_hrs_char_md.p_sccd_md         = NULL;
//	
//	memset(&attr_md, 0, sizeof(attr_md));

//    attr_md.read_perm  = p_ppg_init->ppg_value_char_attr_md.read_perm;
//    attr_md.write_perm = p_ppg_init->ppg_value_char_attr_md.write_perm;
//    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
//    attr_md.rd_auth    = 0;
//    attr_md.wr_auth    = 0;
//    attr_md.vlen       = 0;
//	
//	ble_uuid.type = p_ppg->uuid_type;
//    ble_uuid.uuid = PPG_HRS_CHAR_UUID;
//	
//	memset(&attr_char_ppg_hrs, 0, sizeof(attr_char_ppg_hrs));

//    attr_char_ppg_hrs.p_uuid    = &ble_uuid;
//    attr_char_ppg_hrs.p_attr_md = &attr_md;
//    attr_char_ppg_hrs.init_len  = sizeof(uint8_t);
//    attr_char_ppg_hrs.init_offs = 0;
//    attr_char_ppg_hrs.max_len   = sizeof(uint8_t);
//	
//	//Add PPG HRS characteristic
//	err_code = sd_ble_gatts_characteristic_add(p_ppg->service_handle, &ppg_hrs_char_md,
//                                               &attr_char_ppg_hrs,
//                                               &p_ppg->ppg_hrs_handles);
	
	
	//Setting PPG SPO2 Characteristic-----------------------------------------------------
//	memset(&ppg_spo2_cccd_md, 0, sizeof(ppg_spo2_cccd_md));

//    //  Read  operation on Cccd should be possible without authentication.
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ppg_spo2_cccd_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ppg_spo2_cccd_md.write_perm);
//    
//    ppg_spo2_cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
//	
//	memset(&ppg_spo2_char_md, 0, sizeof(ppg_spo2_char_md));

//    ppg_spo2_char_md.char_props.read   = 0;
//    ppg_spo2_char_md.char_props.write  = 0;
//    ppg_spo2_char_md.char_props.notify = 1; 
//    ppg_spo2_char_md.p_char_user_desc  = NULL;
//    ppg_spo2_char_md.p_char_pf         = NULL;
//    ppg_spo2_char_md.p_user_desc_md    = NULL;
//    ppg_spo2_char_md.p_cccd_md         = &ppg_spo2_cccd_md; 
//    ppg_spo2_char_md.p_sccd_md         = NULL;
//	
//	memset(&attr_md, 0, sizeof(attr_md));

//    attr_md.read_perm  = p_ppg_init->ppg_value_char_attr_md.read_perm;
//    attr_md.write_perm = p_ppg_init->ppg_value_char_attr_md.write_perm;
//    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
//    attr_md.rd_auth    = 0;
//    attr_md.wr_auth    = 0;
//    attr_md.vlen       = 0;
//	
//	ble_uuid.type = p_ppg->uuid_type;
//    ble_uuid.uuid = PPG_SPO2_CHAR_UUID;
//	
//	memset(&attr_char_ppg_spo2, 0, sizeof(attr_char_ppg_spo2));

//    attr_char_ppg_spo2.p_uuid    = &ble_uuid;
//    attr_char_ppg_spo2.p_attr_md = &attr_md;
//    attr_char_ppg_spo2.init_len  = sizeof(uint8_t);
//    attr_char_ppg_spo2.init_offs = 0;
//    attr_char_ppg_spo2.max_len   = sizeof(uint8_t);
//	
//	//Add PPG SPO2 characteristic
//	err_code = sd_ble_gatts_characteristic_add(p_ppg->service_handle, &ppg_spo2_char_md,
//                                               &attr_char_ppg_spo2,
//                                               &p_ppg->ppg_spo2_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_ppg_init(ble_ppg_t * p_ppg, const ble_ppg_init_t * p_ppg_init)
{
    if (p_ppg == NULL || p_ppg_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;
   
	// Initialize service structure
	p_ppg->evt_handler               = p_ppg_init->evt_handler;
	p_ppg->conn_handle               = BLE_CONN_HANDLE_INVALID;
   
	// Add PPG Service UUID
	ble_uuid128_t base_uuid = {PPG_SERVICE_UUID_BASE};
	err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_ppg->uuid_type);
	VERIFY_SUCCESS(err_code);

	ble_uuid.type = p_ppg->uuid_type;
	ble_uuid.uuid = PPG_SERVICE_UUID;

	// Add the PPG Service
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_ppg->service_handle);
	if (err_code != NRF_SUCCESS)
	{
	  return err_code;
	}

	// Add Custom Value characteristic
	return ppg_value_char_add(p_ppg, p_ppg_init);
}
