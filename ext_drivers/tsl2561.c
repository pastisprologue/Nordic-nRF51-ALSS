/**
 ******************************************************************************
 * File Name          : tsl2561.c
 * Date               : 15/07/24 
 * Author             : Michael Chiasson
 * Description        : Driver for TAOS TSL2561 I2C Ambient Light Sensor 
 ******************************************************************************
 *
 */
 
#include "tsl2561.h"
#include "twi_master.h"
#include "nrf_delay.h"
#include "nrf_error.h"

static TSL2561_t m_tsl2561;

/*
 * TSL2561_REG_ADDR Register Address 
 */
#define TSL2561_CMD_REG_ADDR                0x80
#define TSL2561_CTRL_REG_ADDR               0x0
#define TSL2561_TIME_REG_ADDR               0x1
#define TSL2561_THRESLL_REG_ADDR            0x2
#define TSL2561_THRESLH_REG_ADDR            0x3
#define TSL2561_THRESHL_REG_ADDR            0x4
#define TSL2561_THRESHH_REG_ADDR            0x5
#define TSL2561_INTCTRL_REG_ADDR            0x6
#define TSL2561_DATA0L_REG_ADDR             0xC
#define TSL2561_DATA0H_REG_ADDR             0xD
#define TSL2561_DATA1L_REG_ADDR             0xE
#define TSL2561_DATA1H_REG_ADDR             0xF
/********<end> TSL2561_REG_ADDR <end>**********/

/*
 * TSL2561_REG_VALUE_OFFSETS Register Value Offsets
 */
#define TSL2561_CTRL_REG_POWER_OFFSET       0x00

#define TSL2561_TIME_REG_GAIN_OFFSET        0x04
#define TSL2561_TIME_REG_MAN_OFFSET         0x03
#define TSL2561_TIME_REG_INTEG_OFFSET       0x00

#define TSL2561_INT_CTRL_INTR_OFFSET        0x04
#define TSL2561_INT_CTRL_PERSIST_OFFSET     0x00
/****<end> TSL2561_REG_VALUE_OFFSETS <end>******/

/*
 * LUX Calculation Constants
 */
#define LUX_SCALE 14 // scale by 2^14
#define RATIO_SCALE 9 // scale ratio by 2^9
#define CH_SCALE 10 // scale channel values by 2^10
#define CHSCALE_TINT0 0x7517 // 322/11 * 2^CH_SCALE
#define CHSCALE_TINT1 0x0fe7 // 322/81 * 2^CH_SCALE

#define K1T 0x0040 // 0.125 * 2^RATIO_SCALE
#define B1T 0x01f2 // 0.0304 * 2^LUX_SCALE
#define M1T 0x01be // 0.0272 * 2^LUX_SCALE

#define K2T 0x0080 // 0.250 * 2^RATIO_SCALE
#define B2T 0x0214 // 0.0325 * 2^LUX_SCALE
#define M2T 0x02d1 // 0.0440 * 2^LUX_SCALE

#define K3T 0x00c0 // 0.375 * 2^RATIO_SCALE
#define B3T 0x023f // 0.0351 * 2^LUX_SCALE
#define M3T 0x037b // 0.0544 * 2^LUX_SCALE

#define K4T 0x0100 // 0.50 * 2^RATIO_SCALE
#define B4T 0x0270 // 0.0381 * 2^LUX_SCALE
#define M4T 0x03fe // 0.0624 * 2^LUX_SCALE
#define K5T 0x0138 // 0.61 * 2^RATIO_SCALE
#define B5T 0x016f // 0.0224 * 2^LUX_SCALE
#define M5T 0x01fc // 0.0310 * 2^LUX_SCALE

#define K6T 0x019a // 0.80 * 2^RATIO_SCALE
#define B6T 0x00d2 // 0.0128 * 2^LUX_SCALE
#define M6T 0x00fb // 0.0153 * 2^LUX_SCALE

#define K7T 0x029a // 1.3 * 2^RATIO_SCALE
#define B7T 0x0018 // 0.00146 * 2^LUX_SCALE
#define M7T 0x0012 // 0.00112 * 2^LUX_SCALE

#define K8T 0x029a // 1.3 * 2^RATIO_SCALE
#define B8T 0x0000 // 0.000 * 2^LUX_SCALE
#define M8T 0x0000 // 0.000 * 2^LUX_SCALE
/**<end> LUX Calculation Constants <end>**/



static uint32_t tsl2561_read_reg(uint8_t addr, uint8_t* value)
{
    uint8_t command_code;
    
    command_code = (uint8_t)(TSL2561_CMD_REG_ADDR | addr);
    
    if (twi_master_transfer(m_tsl2561.device_address, &command_code, 1, TWI_DONT_ISSUE_STOP))
    {
        if (twi_master_transfer(m_tsl2561.device_address | TWI_READ_BIT, value, 1, TWI_ISSUE_STOP))
        {
            return NRF_SUCCESS;
        }
    }
    
    return NRF_ERROR_FORBIDDEN;
}


static uint32_t tsl2561_write_reg(uint8_t addr, uint8_t* value)
{
    uint8_t data_buffer[2];
    
    data_buffer[0] = (uint8_t)(TSL2561_CMD_REG_ADDR | addr);
    data_buffer[1] = *value;
    
    if(twi_master_transfer(m_tsl2561.device_address, data_buffer, sizeof(data_buffer), TWI_ISSUE_STOP))
    {
        return NRF_SUCCESS;
    }
    
    return NRF_ERROR_FORBIDDEN;
}


uint32_t tsl2561_set_gain(tsl2561_adc_gain_value_t new_gain)
{
    uint32_t err_code;
    uint8_t data_buffer;
    
    // Get the current status of the register 
    err_code = tsl2561_read_reg(TSL2561_TIME_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Clear data_buffer of current gain state
    data_buffer &= (uint8_t)(~(0x1 << TSL2561_TIME_REG_GAIN_OFFSET));
    
    // Fill in appropriate gain value in data_buffer
    data_buffer |= (uint8_t)(new_gain << TSL2561_TIME_REG_GAIN_OFFSET);
    
    // Update the Register with a new value
    err_code = tsl2561_write_reg(TSL2561_TIME_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    m_tsl2561.gain = new_gain;
    
    return NRF_SUCCESS;
}

uint32_t tsl2561_set_integration_time(tsl2561_integration_time_t new_integration_time)
{
    uint32_t err_code;
    uint8_t data_buffer;
    
    // Get the current status of the register 
    err_code = tsl2561_read_reg(TSL2561_TIME_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Clear data_buffer of current gain state
    data_buffer &= (uint8_t)(~(0x3 << TSL2561_TIME_REG_INTEG_OFFSET));
    
    // Fill in appropriate gain value in data_buffer
    data_buffer |= (uint8_t)(new_integration_time << TSL2561_TIME_REG_INTEG_OFFSET);
    
    // Update the Register with a new value
    err_code = tsl2561_write_reg(TSL2561_TIME_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    m_tsl2561.integrate_time = new_integration_time;
    
    return NRF_SUCCESS;
}

uint32_t tsl2561_set_lower_threshold(uint16_t new_threshold)
{
    uint32_t err_code;
    uint8_t data_buffer;
    
    // Set LSB
    data_buffer = (uint8_t)(new_threshold);
    err_code = tsl2561_write_reg(TSL2561_THRESLL_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Set MSB
    data_buffer = (uint8_t)(new_threshold >> 8);
    err_code = tsl2561_write_reg(TSL2561_THRESLH_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    m_tsl2561.lower_threshold = new_threshold;
    
    return NRF_SUCCESS;
}

uint32_t tsl2561_set_upper_threshold(uint16_t new_threshold)
{
    uint32_t err_code;
    uint8_t data_buffer;
    
    // Set LSB
    data_buffer = (uint8_t)(new_threshold);
    err_code = tsl2561_write_reg(TSL2561_THRESHL_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Set MSB
    data_buffer = (uint8_t)(new_threshold >> 8);
    err_code = tsl2561_write_reg(TSL2561_THRESHH_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    m_tsl2561.upper_threshold = new_threshold;
    
    return NRF_SUCCESS;
}

uint32_t tsl2561_set_interrupt_enable(tsl2561_intr_ctrl_val_t new_intr_ctrl)
{
    uint32_t err_code;
    uint8_t data_buffer;
    
    // Get the current status of the register 
    err_code = tsl2561_read_reg(TSL2561_INTCTRL_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Clear data_buffer of current gain state
    data_buffer &= (uint8_t)(~(0x3 << TSL2561_INT_CTRL_INTR_OFFSET));
    
    // Fill in appropriate gain value in data_buffer
    data_buffer |= (uint8_t)(new_intr_ctrl << TSL2561_INT_CTRL_INTR_OFFSET);
    
    // Update the Register with a new value
    err_code = tsl2561_write_reg(TSL2561_INTCTRL_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    m_tsl2561.interrupt_enable = new_intr_ctrl;
    
    return NRF_SUCCESS;
}

uint32_t tsl2561_set_interrupt_persistence(tsl2561_intr_persist_val_t new_persistence)
{
    uint32_t err_code;
    uint8_t data_buffer;
    
    // Get the current status of the register 
    err_code = tsl2561_read_reg(TSL2561_INTCTRL_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Clear data_buffer of current gain state
    data_buffer &= (uint8_t)(~(0xF << TSL2561_INT_CTRL_PERSIST_OFFSET));
    
    // Fill in appropriate gain value in data_buffer
    data_buffer |= (uint8_t)(new_persistence << TSL2561_INT_CTRL_PERSIST_OFFSET);
    
    // Update the Register with a new value
    err_code = tsl2561_write_reg(TSL2561_INTCTRL_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    m_tsl2561.interrupt_persistence = new_persistence;
    
    return NRF_SUCCESS;
}


uint32_t tsl2561_init(TSL2561_t* als_init)
{
    uint32_t err_code;
    
    m_tsl2561.device_address = als_init->device_address;
    
    nrf_delay_ms(15);
        
    // Set Timing Register
    // Set Gain
    err_code = tsl2561_set_gain(als_init->gain);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Set Integration Time
    err_code = tsl2561_set_integration_time(als_init->integrate_time);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Set Lower Threshold
    err_code = tsl2561_set_lower_threshold(als_init->lower_threshold);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Set Upper Threshold
    err_code = tsl2561_set_upper_threshold(als_init->upper_threshold);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Set Interrupt Control Register
    // Set Interrupt Control
    err_code = tsl2561_set_interrupt_enable(als_init->interrupt_enable);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Set Interrupt Persistence
    err_code = tsl2561_set_interrupt_persistence(als_init->interrupt_persistence);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;
}

uint32_t tsl2561_power_on(void)
{
    uint32_t err_code;
    uint8_t data_buffer;
    
    // Set LSB
    data_buffer = (uint8_t)(0x03 << TSL2561_CTRL_REG_POWER_OFFSET);
    err_code = tsl2561_write_reg(TSL2561_CTRL_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    return NRF_SUCCESS;
}


uint32_t tsl2561_power_off(void)
{
    uint32_t err_code;
    uint8_t data_buffer;
    
    // Set LSB
    data_buffer = (uint8_t)(0x00 << TSL2561_CTRL_REG_POWER_OFFSET);
    err_code = tsl2561_write_reg(TSL2561_CTRL_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    return NRF_SUCCESS;
}


uint32_t tsl2561_read_raw_ADC(uint16_t* channel0, uint16_t* channel1)
{
    uint32_t err_code;
    uint8_t data_buffer;
    
    //Read Channel 0 LSB
    err_code = tsl2561_read_reg(TSL2561_DATA0L_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    *channel0 = (uint16_t)(data_buffer);
    
    //Read Channel 0 MSB
    err_code = tsl2561_read_reg(TSL2561_DATA0H_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    *channel0 |= (uint16_t)((data_buffer << 8) & 0xFF00);
    
    //Read Channel 1 LSB
    err_code = tsl2561_read_reg(TSL2561_DATA1L_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    *channel1 = (uint16_t)(data_buffer);
    
    //Read Channel 1 MSB
    err_code = tsl2561_read_reg(TSL2561_DATA1H_REG_ADDR, &data_buffer);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    *channel1 |= (uint16_t)((data_buffer << 8) & 0xFF00);   
    
    return NRF_SUCCESS;
}


uint32_t CalculateLux(uint16_t ch0, uint16_t ch1)
{
    //------------------------------------------------------------------------
    // first, scale the channel values depending on the gain and integration time
    // 16X, 402mS is nominal.
    // scale if integration time is NOT 402 msec
    uint32_t chScale;
    uint32_t channel1;
    uint32_t channel0;
    
    switch (m_tsl2561.integrate_time)
    {
        case TSL2561_INT_TIME_13_7_MS: // 13.7 msec
            chScale = CHSCALE_TINT0;
            break;
        case TSL2561_INT_TIME_101_MS: // 101 msec
            chScale = CHSCALE_TINT1;
            break;
        default: // assume no scaling
            chScale = (1 << CH_SCALE);
        break;
    }
    
    // scale if gain is NOT 16X
    if (!m_tsl2561.gain) chScale = chScale << 4; // scale 1X to 16X
    
    // scale the channel values
    channel0 = (ch0 * chScale) >> CH_SCALE;
    channel1 = (ch1 * chScale) >> CH_SCALE;
    
    // find the ratio of the channel values (Channel1/Channel0)
    // protect against divide by zero
    uint32_t ratio1 = 0;
    if (channel0 != 0) ratio1 = (channel1 << (RATIO_SCALE+1)) / channel0;
    
    // round the ratio value
    int32_t ratio = (ratio1 + 1) >> 1;
    
    // is ratio <= eachBreak ?
    uint16_t b, m;
    
    if ((ratio >= 0) && (ratio <= K1T))
        {b=B1T; m=M1T;}
    else if (ratio <= K2T)
        {b=B2T; m=M2T;}
    else if (ratio <= K3T)
        {b=B3T; m=M3T;}
    else if (ratio <= K4T)
        {b=B4T; m=M4T;}
    else if (ratio <= K5T)
        {b=B5T; m=M5T;}
    else if (ratio <= K6T)
        {b=B6T; m=M6T;}
    else if (ratio <= K7T)
        {b=B7T; m=M7T;}
    else if (ratio > K8T)
        {b=B8T; m=M8T;}
        
    int32_t temp;
    temp = ((channel0 * b) - (channel1 * m));
    // do not allow negative lux value
    if (temp < 0) temp = 0;
    // round lsb (2^(LUX_SCALE-1))
    temp += (1 << (LUX_SCALE-1));
    // strip off fractional portion
    unsigned long lux = temp >> LUX_SCALE;
    return(lux);
}


