/**
 ******************************************************************************
 * File Name          : tsl2561.c
 * Date               : 15/07/24 
 * Author             : Michael Chiasson
 * Description        : Driver for TAOS TSL2561 I2C Ambient Light Sensor 
 ******************************************************************************
 *
 */

#ifndef TSL2561_H
#define TSL2561_H


#include <stdbool.h>
#include <stdint.h>


/*
 * Possible values for the TSL2561 ADC Gain
 */
typedef enum 
{
	TSL2561_ADC_GAIN_LOW = 0,			/** Low gain setting of 1x 		*/
	TSL2561_ADC_GAIN_HIGH,				/** High gain setting of 16x	*/
} tsl2561_adc_gain_value_t;


/*
 * Possible values for the TSL2561 Integration Conversion Timing
 */
typedef enum 
{
	TSL2561_INT_TIME_13_7_MS = 0,		/** 13.7ms Nominal Integration Time, 0.034 scaling factor 	*/
	TSL2561_INT_TIME_101_MS, 			/** 101ms Nominal Integration Time, 0.034 scaling factor	*/
	TSL2561_INT_TIME_402_MS,			/** 402ms Nominal Integration Time, 0.252 scaling factor	*/
	TSL2561_INT_TIME_MANUAL,			/** Manual Timing Control									*/
} tsl2561_integration_time_t;


/*
 * Possible values for the TSL2561 Interrupt Control Select
 */
typedef enum 
{
	TSL2561_INTR_CTRL_DISABLED = 0,		/** Interrupt output disabled			*/
	TSL2561_INTR_CTRL_LEVEL,			/** Level Interrupt						*/
	TSL2561_INTR_CTRL_SMBALERT,			/** SMBAlert Compliant					*/
	TSL2561_INTR_CTRL_TEST				/** Test Mode							*/
} tsl2561_intr_ctrl_val_t;


/*
 * Possible values for the TSL2561 Interrupt Persistence Select
 */
typedef enum 
{
	TSL2561_PERSIST_VAL_EVERY = 0,		/** Every ADC conversion generates interrupt 	*/
	TSL2561_PERSIST_VAL_ANY,			/** Any value outside of threshold range		*/
	TSL2561_PERSIST_VAL_2CONV,			/** 2 conversions outside of threshold range	*/
	TSL2561_PERSIST_VAL_3CONV,			/** 3 conversions outside of threshold range	*/
	TSL2561_PERSIST_VAL_4CONV,			/** 4 conversions outside of threshold range	*/
	TSL2561_PERSIST_VAL_5CONV,			/** 5 conversions outside of threshold range	*/
	TSL2561_PERSIST_VAL_6CONV,			/** 6 conversions outside of threshold range	*/		
	TSL2561_PERSIST_VAL_7CONV,			/** 7 conversions outside of threshold range	*/
	TSL2561_PERSIST_VAL_8CONV,			/** 8 conversions outside of threshold range	*/
	TSL2561_PERSIST_VAL_9CONV,			/** 9 conversions outside of threshold range	*/
	TSL2561_PERSIST_VAL_10CONV,			/** 10 conversions outside of threshold range	*/
	TSL2561_PERSIST_VAL_11CONV,			/** 11 conversions outside of threshold range	*/
	TSL2561_PERSIST_VAL_12CONV,			/** 12 conversions outside of threshold range	*/
	TSL2561_PERSIST_VAL_13CONV,			/** 13 conversions outside of threshold range	*/
	TSL2561_PERSIST_VAL_14CONV,			/** 14 conversions outside of threshold range	*/
	TSL2561_PERSIST_VAL_15CONV			/** 15 conversions outside of threshold range	*/
} tsl2561_intr_persist_val_t;

/** Instance type of TSL2561 Ambient Light Sensor*/
typedef struct
{	
	uint8_t 					device_address;			/**	7-bit I2C Address								*/
	
	tsl2561_adc_gain_value_t 	gain;					/** Value from a TSL2561_ADC_GAIN_VALUE series.		*/
	tsl2561_integration_time_t 	integrate_time;			/** Value from a TSL2561_INTEGRATION_TIME series. 	*/
	
	uint16_t 					lower_threshold;		/** Interrupt lower threshold						*/	
	uint16_t					upper_threshold;		/**	Interrupt upper threshold						*/
	
	tsl2561_intr_ctrl_val_t		interrupt_enable;		/** Value from a TSL2561_INTR_CTRL_VAL series.		*/
	tsl2561_intr_persist_val_t 	interrupt_persistence;	/** Value from a TSL2561_INTR_PERSIST_VAL series.	*/
} TSL2561_t;


/*
 * 	tsl2561_init
 * 	Initialize a TSL2561 Ambient Light Sensor
 *	PARAMETERS: Point to a TSL2561_t with preferred parameters
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tsl2561_init(TSL2561_t* als_init);


/*
 * 	tsl2561_set_gain
 * 	Set the gain of your pre-initialized TSL2561 Ambient Light Sensor
 *	PARAMETERS: new gain value, must be of type tsl2561_adc_gain_value_t 
*	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tsl2561_set_gain(tsl2561_adc_gain_value_t new_gain);


/*
 * 	tsl2561_set_integration_time
 * 	Set the integration time of your pre-initialized TSL2561 Ambient Light Sensor
 *	PARAMETERS: new integration time value, must be of type tsl2561_integration_time_t 
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tsl2561_set_integration_time(tsl2561_integration_time_t new_integration_time);


/*
 * 	tsl2561_set_lower_threshold
 * 	Set the lower threshold of your pre-initialized TSL2561 Ambient Light Sensor
 *	PARAMETERS: new threshold value, must be of range 0x0000 to 0xFFFF (unsigned int) 
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tsl2561_set_lower_threshold(uint16_t new_threshold);


/*
 * 	tsl2561_set_upper_threshold
 * 	Set the upper threshold of your pre-initialized TSL2561 Ambient Light Sensor
 *	PARAMETERS: new threshold value, must be of range 0x0000 to 0xFFFF (unsigned int) 
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tsl2561_set_upper_threshold(uint16_t new_threshold);


/*
 * 	tsl2561_set_interrupt_enable
 * 	Set the interrupt type of your pre-initialized TSL2561 Ambient Light Sensor
 *	PARAMETERS: new interrupt type, must be of type tsl2561_intr_ctrl_val_t
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tsl2561_set_interrupt_enable(tsl2561_intr_ctrl_val_t new_intr_ctrl);


/*
 * 	tsl2561_set_interrupt_persistence
 * 	Set the interrupt persistence of your pre-initialized TSL2561 Ambient Light Sensor
 *	PARAMETERS: new interrupt persistence, must be of type tsl2561_intr_persist_val_t 
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tsl2561_set_interrupt_persistence(tsl2561_intr_persist_val_t new_persistence);


/*
 * 	tsl2561_power_on
 * 	Power on the pre-initialized TSL2561 Ambient Light Sensor
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tsl2561_power_on(void);


/*
 * 	tsl2561_power_off
 * 	Power down the pre-initialized TSL2561 Ambient Light Sensor
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tsl2561_power_off(void);


/*
 * 	tsl2561_read_raw_ADC
 * 	Read the analog measurements pre-initialized TSL2561 Ambient Light Sensor
 *	PARAMETERS: pointer to where channel0 (Visible and IR) will be stored
 *				pointer to where channel1 (IR only) will be stored
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tsl2561_read_raw_ADC(uint16_t* channel0, uint16_t* channel1);


/*
 * 	CalculateLux
 * 	Given the analog measurements of both Visible and IR, and IR only,
 *		estimate the Lux value at the sensor location.
 *	PARAMETERS: channel0 (Visible and IR) input value
 *				channel1 (IR only) input value
 *	RETURN:		32-bit unsigned Lux value 
 */
uint32_t CalculateLux(uint16_t ch0, uint16_t ch1);


#endif
