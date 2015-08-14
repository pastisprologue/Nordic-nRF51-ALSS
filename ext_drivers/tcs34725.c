/**
 ******************************************************************************
 * File Name          : tcs34725.c
 * Date               : 15/07/24 
 * Author             : Michael Chiasson
 * Description        : Driver for TAOS TCS34725 I2C Color Sensor 
 ******************************************************************************
 *
 */
 
#include "tcs34725.h"
#include "twi_master.h"
#include "nrf_delay.h"
#include "nrf_error.h"

static TCS34725_t m_tcs34725;

/*
 * TCS34725_REG_ADDR Register Address 
 */
#define TCS34725_ENABLE_REG_ADDR				0x00
#define TCS34725_ATIME_REG_ADDR					0x01
#define TCS34725_WTIME_REG_ADDR					0x02
#define TCS34725_AILTL_REG_ADDR					0x03
#define TCS34725_AILTH_REG_ADDR					0x04
#define TCS34725_AIHTL_REG_ADDR					0x05
#define TCS34725_AIHTH_REG_ADDR					0x06
#define TCS34725_PERS_REG_ADDR					0x0C
#define TCS34725_CONFIG_REG_ADDR				0x0D
#define TCS34725_CTRL_REG_ADDR					0x0F
#define TCS34725_ID_REG_ADDR					0x12
#define TCS34725_STATUS_REG_ADDR				0x13
#define TCS34725_CDATAL_REG_ADDR				0x14
#define TCS34725_CDATAH_REG_ADDR				0x15
#define TCS34725_RDATAL_REG_ADDR				0x16
#define TCS34725_RDATAH_REG_ADDR				0x17
#define TCS34725_GDATAL_REG_ADDR				0x18
#define TCS34725_GDATAH_REG_ADDR				0x19
#define TCS34725_BDATAL_REG_ADDR				0x1A
#define TCS34725_BDATAH_REG_ADDR				0x1B
#define TCS34725_CMD_REG_ADDR					0x80
/********<end> TCS34725_REG_ADDR <end>**********/

/*
 * TCS34725 ENABLE REGISTER OFFSETS
 */
#define TCS34725_ENABLE_AIEN_OFFSET				4
#define TCS34725_ENABLE_WEN_OFFSET				3
#define TCS34725_ENABLE_AEN_OFFSET				1
#define TCS34725_ENABLE_PON_OFFSET				0
/***<end> TCS34725 ENABLE REGISTER OFFSETS <end>***/

/*
 * TCS34725 WAIT TIME CONFIGURATION
 */
 #define TCS34725_WAIT_TIME_SHORT				0x00
 #define TCS34725_WAIT_TIME_LONG				0x02
 /***<end> TCS34725 WAIT TIME CONFIGURATION <end>***/
 
 
static uint32_t tcs34725_read_reg(uint8_t addr, uint8_t* value)
{
	uint8_t command_code;
	
	command_code = (uint8_t)(TCS34725_CMD_REG_ADDR | addr);
	
	if (twi_master_transfer(m_tcs34725.device_address, &command_code, 1, TWI_DONT_ISSUE_STOP))
	{
		if (twi_master_transfer(m_tcs34725.device_address | TWI_READ_BIT, value, 1, TWI_ISSUE_STOP))
		{
			return NRF_SUCCESS;
		}
	}
	
	return NRF_ERROR_FORBIDDEN;
}


static uint32_t tcs34725_write_reg(uint8_t addr, uint8_t* value)
{
	uint8_t data_buffer[2];
	
	data_buffer[0] = (uint8_t)(TCS34725_CMD_REG_ADDR | addr);
	data_buffer[1] = *value;
	
	if(twi_master_transfer(m_tcs34725.device_address, data_buffer, sizeof(data_buffer), TWI_ISSUE_STOP))
	{
		return NRF_SUCCESS;
	}
	
	return NRF_ERROR_FORBIDDEN;
}


uint32_t tcs34725_set_gain(tcs34725_adc_gain_value_t new_gain)
{
	uint32_t err_code;
	
	// Update the Register with a new value
	err_code = tcs34725_write_reg(TCS34725_CTRL_REG_ADDR, (uint8_t*)&new_gain);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	m_tcs34725.gain = new_gain;
	
	return NRF_SUCCESS;
}

uint32_t tcs34725_set_integration_time(uint8_t cycles)
{
	uint32_t err_code;
	uint8_t value;
	
	value = (uint8_t)(256 - cycles);
	
	// Update the Register with a new value
	err_code = tcs34725_write_reg(TCS34725_ATIME_REG_ADDR, &value);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	m_tcs34725.integration_cycles = cycles;
	
	return NRF_SUCCESS;
}

uint32_t tcs34725_set_wait_time(uint8_t cycles)
{
	uint32_t err_code;
	uint8_t value;
	
	value = (uint8_t)(256 - cycles);
	
	// Set Wait Time
	err_code = tcs34725_write_reg(TCS34725_WTIME_REG_ADDR, &value);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
		
	m_tcs34725.wait_cycles = cycles;
	
	return NRF_SUCCESS;
}

uint32_t tcs34725_set_wait_multiplier(tcs34725_wait_multiplier_t new_wait_multiplier)
{
	uint32_t err_code;
	
	err_code = tcs34725_write_reg(TCS34725_CONFIG_REG_ADDR, (uint8_t*)&new_wait_multiplier);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
		
	m_tcs34725.wait_multiplier = new_wait_multiplier;
	
	return NRF_SUCCESS;
}

uint32_t tcs34725_set_lower_threshold(uint16_t new_threshold)
{
	uint32_t err_code;
	uint8_t data_buffer;
	
	// Set LSB
	data_buffer = (uint8_t)(new_threshold);
	err_code = tcs34725_write_reg(TCS34725_AILTL_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Set MSB
	data_buffer = (uint8_t)(new_threshold >> 8);
	err_code = tcs34725_write_reg(TCS34725_AILTH_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	m_tcs34725.lower_threshold = new_threshold;
	
	return NRF_SUCCESS;
}

uint32_t tcs34725_set_upper_threshold(uint16_t new_threshold)
{
	uint32_t err_code;
	uint8_t data_buffer;
	
	// Set LSB
	data_buffer = (uint8_t)(new_threshold);
	err_code = tcs34725_write_reg(TCS34725_AIHTL_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Set MSB
	data_buffer = (uint8_t)(new_threshold >> 8);
	err_code = tcs34725_write_reg(TCS34725_AIHTH_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	m_tcs34725.upper_threshold = new_threshold;
	
	return NRF_SUCCESS;
}

uint32_t tcs34725_set_interrupt_persistence(tcs34725_intr_persist_val_t new_persistence)
{
	uint32_t err_code;
		
	// Update the Register with a new value
	err_code = tcs34725_write_reg(TCS34725_PERS_REG_ADDR, (uint8_t*)&new_persistence);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	m_tcs34725.interrupt_persistence = new_persistence;
	
	return NRF_SUCCESS;
}


uint32_t tcs34725_init(TCS34725_t* cs_init)
{
	uint32_t err_code;
	
	m_tcs34725.device_address = cs_init->device_address;
	
	nrf_delay_ms(15);
		
    // Set Timing Register
	// Set Gain
	err_code = tcs34725_set_gain(cs_init->gain);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Set Integration Time
	err_code = tcs34725_set_integration_time(cs_init->integration_cycles);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Set Lower Threshold
	err_code = tcs34725_set_lower_threshold(cs_init->lower_threshold);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Set Upper Threshold
	err_code = tcs34725_set_upper_threshold(cs_init->upper_threshold);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Set Interrupt Persistence
	err_code = tcs34725_set_interrupt_persistence(cs_init->interrupt_persistence);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Set Wait Time
	err_code = tcs34725_set_wait_time(cs_init->wait_cycles);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Set Wait Multiplier
	err_code = tcs34725_set_wait_multiplier(cs_init->wait_multiplier);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	return NRF_SUCCESS;
}

uint32_t tcs34725_power_on(void)
{
	uint32_t err_code;
	uint8_t data_buffer;
	
	// Read Enable Register
	err_code = tcs34725_read_reg(TCS34725_ENABLE_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Set the Power State
	data_buffer |= (uint8_t)(0x1 << TCS34725_ENABLE_PON_OFFSET);
	
	// Re-Write the Enable Register
	err_code = tcs34725_write_reg(TCS34725_ENABLE_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}	
	
	return NRF_SUCCESS;
}


uint32_t tcs34725_power_off(void)
{
	uint32_t err_code;
	uint8_t data_buffer;
	
	// Read Enable Register
	err_code = tcs34725_read_reg(TCS34725_ENABLE_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	// Clear the Power State
	data_buffer &= (uint8_t)(~(0x1 << TCS34725_ENABLE_PON_OFFSET));
	
	// Re-Write the Enable Register
	err_code = tcs34725_write_reg(TCS34725_ENABLE_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}	
	
	return NRF_SUCCESS;
}


uint32_t tcs34725_read_raw_ADC(TCS34725_data_t* crgb_data)
{
	uint32_t err_code;
	uint8_t data_buffer;
	// Read clear data LSB
	err_code = tcs34725_read_reg(TCS34725_CDATAL_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}	
	
	crgb_data->clear_data = (uint16_t)(data_buffer);
	
	// Read clear data MSB
	err_code = tcs34725_read_reg(TCS34725_CDATAH_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}	
	
	crgb_data->clear_data = (uint16_t)(data_buffer);
	
	// Read red data LSB
	err_code = tcs34725_read_reg(TCS34725_RDATAL_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}	
	
	crgb_data->red_data = (uint16_t)(data_buffer);
	
	// Read red data MSB
	err_code = tcs34725_read_reg(TCS34725_RDATAH_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}	
	
	crgb_data->clear_data = (uint16_t)(data_buffer);
	
	// Read green data LSB
	err_code = tcs34725_read_reg(TCS34725_GDATAL_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}	
	
	crgb_data->green_data = (uint16_t)(data_buffer);
	
	// Read green data MSB
	err_code = tcs34725_read_reg(TCS34725_GDATAH_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}	
	
	crgb_data->green_data = (uint16_t)(data_buffer);
	
	// Read blue data LSB
	err_code = tcs34725_read_reg(TCS34725_BDATAL_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}	
	
	crgb_data->blue_data = (uint16_t)(data_buffer);
	
	// Read blue data MSB
	err_code = tcs34725_read_reg(TCS34725_BDATAH_REG_ADDR, &data_buffer);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}	
	
	crgb_data->blue_data = (uint16_t)(data_buffer);
	
	
	return NRF_SUCCESS;
}





