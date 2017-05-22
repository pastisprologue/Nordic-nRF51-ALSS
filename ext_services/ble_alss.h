/**
 ******************************************************************************
 * File Name          : ble_alss.h
 * Date               : 15/07/24 
 * Author             : Michael Chiasson
 * Description        : Bluetooth Low Energy Ambient Light Sensor Service
 ******************************************************************************
 *
 */
#ifndef BLE_ALSS_H__
#define BLE_ALSS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define LMP_UUID_BASE 			{0xC5, 0xA4, 0xAB, 0x52, 0x98, 0x54, 0x16, 0x8D, 0x71, 0x4C, 0x03, 0x4A, 0x00, 0x00, 0x09, 0xF3}
#define ALSS_UUID_SERVICE 		0x0010
#define ALSS_UUID_IRVAL_CHAR 	0x0011
#define ALSS_UUID_VIRVAL_CHAR 	0x0012
#define ALSS_UUID_LUX_CHAR		0x0013


// Forward declaration of the ble_alss_t type. 
typedef struct ble_alss_s ble_alss_t;


/**@brief Ambient Light Sensor Service structure. This contains various status information for the service. */
typedef struct ble_alss_s
{
    uint16_t                    	service_handle;
    ble_gatts_char_handles_t    	irval_char_handles;
    ble_gatts_char_handles_t    	virval_char_handles;
    ble_gatts_char_handles_t    	lux_char_handles;
    uint8_t                     	uuid_type;
    uint16_t                    	conn_handle;
} ble_alss_t;


/**@brief Function for initializing the Ambient Light Sensor Service.
 *
 * @param[out]  p_alss       Ambient Light Sensor Service structure. This structure will have to be supplied by
 *                           the application. It will be initialized by this function, and will later
 *                           be used to identify this particular service instance.
 * @param[in]   p_alss_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_alss_init(ble_alss_t * p_alss);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Ambient Light Sensor Service.
 *
 *
 * @param[in]   p_alss      Ambient Light Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
void ble_alss_on_ble_evt(ble_alss_t * p_alss, ble_evt_t * p_ble_evt);


/**@brief Function for sending a LUX Value change notification.
 */
uint32_t ble_alss_on_lux_change(ble_alss_t * p_alss, uint32_t lux);

#endif // BLE_ALSS_H__

/**
 * @}
 */

