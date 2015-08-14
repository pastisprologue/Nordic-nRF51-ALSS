/**
 ******************************************************************************
 * File Name          : ble_alss.c
 * Date               : 15/07/24 
 * Author             : Michael Chiasson
 * Description        : Ambient Light Sensor Service Structure
 ******************************************************************************
 *
 */
#include "ble_alss.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_alss      Ambient Light Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_alss_t * p_alss, ble_evt_t * p_ble_evt)
{
    p_alss->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_alss      Ambient Light Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_alss_t * p_alss, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_alss->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_alss      Ambient Light Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_alss_t * p_alss, ble_evt_t * p_ble_evt)
{
    /* Need to define */
}


/**@brief Function for handling the Read/Write Authorize Request evemt.
 *
 * @param[in]   p_alss    	Ambient Light Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_rw_authorize(ble_alss_t * p_alss, ble_evt_t * p_ble_evt)
{
	/* Need to define */
}


void ble_alss_on_ble_evt(ble_alss_t * p_alss, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_alss, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_alss, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_alss, p_ble_evt);
            break;
		
		case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
			on_rw_authorize(p_alss, p_ble_evt);
			break;
            
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the Infrared Value characteristic.
 *
 */
static uint32_t irval_char_add(ble_alss_t * p_alss)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

#ifdef ALSS_UDD	
	uint8_t irval_UDD[] = "ALSS_IRVAL";
#endif
	
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
	char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

#ifdef ALSS_UDD	
	char_md.p_char_user_desc 		= irval_UDD;
	char_md.char_user_desc_max_size = sizeof(irval_UDD);
	char_md.char_user_desc_size 	= sizeof(irval_UDD);
#endif    
	
    ble_uuid.type = p_alss->uuid_type;
    ble_uuid.uuid = ALSS_UUID_IRVAL_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint16_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint16_t);
    attr_char_value.p_value      = NULL;
    
    return sd_ble_gatts_characteristic_add(p_alss->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_alss->irval_char_handles);
}


/**@brief Function for adding the Visual/Infrared Value characteristic.
 *
 */
static uint32_t virval_char_add(ble_alss_t * p_alss)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
	
#ifdef ALSS_UDD		
	uint8_t virval_UDD[] = "ALSS_VIRVAL";
#endif
	    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
	
#ifdef ALSS_UDD
	char_md.p_char_user_desc 		= virval_UDD;
	char_md.char_user_desc_max_size = sizeof(virval_UDD);
	char_md.char_user_desc_size 	= sizeof(virval_UDD);
#endif    

    ble_uuid.type = p_alss->uuid_type;
    ble_uuid.uuid = ALSS_UUID_VIRVAL_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint16_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint16_t);
    attr_char_value.p_value      = NULL;
    
    return sd_ble_gatts_characteristic_add(p_alss->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_alss->virval_char_handles);
}


/**@brief Function for adding the LUX characteristic.
 *
 */
static uint32_t lux_char_add(ble_alss_t * p_alss)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
	
#ifdef ALSS_UDD		
	uint8_t lux_UDD[] = "ALSS_LUX";
#endif
	
    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
	
#ifdef ALSS_UDD
	char_md.p_char_user_desc 		= lux_UDD;
	char_md.char_user_desc_max_size = sizeof(lux_UDD);
	char_md.char_user_desc_size 	= sizeof(lux_UDD);
#endif    

    ble_uuid.type = p_alss->uuid_type;
    ble_uuid.uuid = ALSS_UUID_LUX_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint32_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint32_t);
    attr_char_value.p_value      = NULL;
    
    return sd_ble_gatts_characteristic_add(p_alss->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_alss->lux_char_handles);
}


uint32_t ble_alss_init(ble_alss_t * p_alss)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_alss->conn_handle       = BLE_CONN_HANDLE_INVALID;
        
    ble_uuid.type = p_alss->uuid_type;
    ble_uuid.uuid = ALSS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_alss->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
	err_code = irval_char_add(p_alss);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	
    err_code = virval_char_add(p_alss);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	
	err_code = lux_char_add(p_alss);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
	return NRF_SUCCESS;
}


uint32_t ble_alss_on_lux_change(ble_alss_t * p_alss, uint32_t lux)
{
    ble_gatts_hvx_params_t params;
	uint32_t swap = ((lux>>24)&0xff) | ((lux<<8)&0xff0000)  | ((lux>>8)&0xff00)  | ((lux<<24)&0xff000000);
    uint16_t len = sizeof(swap);
    
    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_alss->lux_char_handles.value_handle;
    params.p_data = (uint8_t*)&swap;
    params.p_len = &len;
    
    return sd_ble_gatts_hvx(p_alss->conn_handle, &params);
}

/**
 * @}
 */
