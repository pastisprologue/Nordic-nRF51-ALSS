/**
 ******************************************************************************
 * File Name          : tcs34725.c
 * Date               : 15/07/24 
 * Author             : Michael Chiasson
 * Description        : Driver for TAOS TCS34725 I2C Color Sensor 
 ******************************************************************************
 *
 */

#ifndef TCS34725_H
#define TCS34725_H


#include <stdbool.h>
#include <stdint.h>


/*
 * Possible values for the TCS34725 ADC Gain
 */
typedef enum 
{
	TCS34725_ADC_GAIN_1X = 0,			/** Gain setting of 1x 		*/
	TCS34725_ADC_GAIN_4X,				/** Gain setting of 4x		*/
	TCS34725_ADC_GAIN_16X,				/** Gain setting of 16x		*/
	TCS34725_ADC_GAIN_60X,				/** Gain setting of 60x		*/
} tcs34725_adc_gain_value_t;


/*
 * Possible values for the TCS34725 Wait Multiplier
 */
typedef enum
{
	TCS34725_WAIT_MULTIPLIER_1X 	= 0x00,  	/** Wait Multiplier of 1x  	*/
	TCS34725_WAIT_MULTIPLIER_12X 	= 0x02,		/** Wait Multiplier of 12x 	*/
} tcs34725_wait_multiplier_t;	

/*
 * Possible values for the TCS34725 Interrupt Control Select
 */
typedef enum 
{
	TCS34725_INTR_CTRL_DISABLED = 0,		/** Interrupt output disabled			*/
	TCS34725_INTR_CTRL_LEVEL,			/** Level Interrupt						*/
	TCS34725_INTR_CTRL_SMBALERT,			/** SMBAlert Compliant					*/
	TCS34725_INTR_CTRL_TEST				/** Test Mode							*/
} tcs34725_intr_ctrl_val_t;


/*
 * Possible values for the TCS34725 Interrupt Persistence Select
 */
typedef enum 
{
	TCS34725_PERSIST_VAL_EVERY = 0,		/** Every ADC conversion generates interrupt 	*/
	TCS34725_PERSIST_VAL_1CONV,			/** 1 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_2CONV,			/** 2 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_3CONV,			/** 3 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_5CONV,			/** 5 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_10CONV,		/** 10 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_15CONV,		/** 15 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_20CONV,		/** 20 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_25CONV,		/** 25 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_30CONV,		/** 30 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_35CONV,		/** 35 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_40CONV,		/** 40 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_45CONV,		/** 45 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_50CONV,		/** 50 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_55CONV,		/** 55 conversions outside of threshold range	*/
	TCS34725_PERSIST_VAL_60CONV,		/** 60 conversions outside of threshold range	*/
} tcs34725_intr_persist_val_t;

/** Instance type of TCS34725 Color Sensor*/
typedef struct
{	
	uint8_t 						device_address;			/**	7-bit I2C Address								*/
	
	tcs34725_adc_gain_value_t 		gain;					/** Value from a TCS34725_ADC_GAIN_VALUE series.	*/
	uint8_t		 					integration_cycles;		/** Amount of integration cycles (MAX 255)		 	*/
	uint8_t							wait_cycles;			/** Amount of wait cycles (MAX 256)					*/
	tcs34725_wait_multiplier_t		wait_multiplier;		/** Value from a TCS34725_WAIT_MULTIPLIER series	*/
	
	uint16_t 						lower_threshold;		/** Interrupt lower threshold						*/	
	uint16_t						upper_threshold;		/**	Interrupt upper threshold						*/
	
	tcs34725_intr_persist_val_t 	interrupt_persistence;	/** Value from a TCS34725_INTR_PERSIST_VAL series.	*/
	
} TCS34725_t;

typedef struct
{
	uint16_t	clear_data;
	uint16_t	red_data;
	uint16_t	green_data;
	uint16_t	blue_data;
} TCS34725_data_t;


/**************************************************************************************
 * 	tcs34725_init
 * 	Initialize a TCS34725 Color Sensor
 *	PARAMETERS: Point to a TCS34725_t with preferred parameters
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tcs34725_init(TCS34725_t* als_init);
/**************************************************************************************/

/**************************************************************************************
 * 	tcs34725_set_gain
 * 	Set the gain of your pre-initialized TCS34725 Color Sensor
 *	PARAMETERS: new gain value, must be of type tcs34725_adc_gain_value_t 
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tcs34725_set_gain(tcs34725_adc_gain_value_t new_gain);


/**************************************************************************************
 * 	tcs34725_set_integration_time
 * 	Set the integration time of your pre-initialized TCS34725 Color Sensor
 *	PARAMETERS: new integration time value, must be of type tcs34725_integration_time_t 
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tcs34725_set_integration_time(uint8_t cycles);


/**************************************************************************************
 * 	tcs34725_set_lower_threshold
 * 	Set the lower threshold of your pre-initialized TCS34725 Color Sensor
 *	PARAMETERS: new threshold value, must be of range 0x0000 to 0xFFFF (unsigned int) 
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tcs34725_set_lower_threshold(uint16_t new_threshold);


/**************************************************************************************
 * 	tcs34725_set_upper_threshold
 * 	Set the upper threshold of your pre-initialized TCS34725 Color Sensor
 *	PARAMETERS: new threshold value, must be of range 0x0000 to 0xFFFF (unsigned int) 
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tcs34725_set_upper_threshold(uint16_t new_threshold);


/**************************************************************************************
 * 	tcs34725_set_interrupt_enable
 * 	Set the interrupt type of your pre-initialized TCS34725 Color Sensor
 *	PARAMETERS: new interrupt type, must be of type tcs34725_intr_ctrl_val_t
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tcs34725_set_interrupt_enable(tcs34725_intr_ctrl_val_t new_intr_ctrl);


/**************************************************************************************
 * 	tcs34725_set_interrupt_persistence
 * 	Set the interrupt persistence of your pre-initialized TCS34725 Color Sensor
 *	PARAMETERS: new interrupt persistence, must be of type tcs34725_intr_persist_val_t 
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tcs34725_set_interrupt_persistence(tcs34725_intr_persist_val_t new_persistence);


/**************************************************************************************
 * 	tcs34725_power_on
 * 	Power on the pre-initialized TCS34725 Color Sensor
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tcs34725_power_on(void);


/**************************************************************************************
 * 	tcs34725_power_off
 * 	Power down the pre-initialized TCS34725 Color Sensor
 *	RETURN:		NRF_SUCCESS or Error Code
 */
uint32_t tcs34725_power_off(void);






#endif
