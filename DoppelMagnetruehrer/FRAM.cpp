/*
 * FRAM.cpp
 *
 * Created: 03.03.2019 22:22:13
 *  Author: JackFrost
 */ 

#include "FRAM.h"
#include "lcd.h"
#include "twi.h"
#include "SD1306.h"
#include "main.h"
#include "SPI.h"
#include "FM25CL64B.h"
#include <avr/interrupt.h>
#include <util/delay.h>

void set_WREN()
{
	PORTC.OUTCLR = PIN4_bm;
	SPIC_Write(WREN);
	PORTC.OUTSET = PIN4_bm;
}

void Write_to_SPI_FRAM(uint8_t *writeData,uint16_t Adress,uint8_t bytes)
{
	
	SPIC_Write(WRITE);
	SPIC_Write_16bit_address(Adress);
	SPIC_Write_array(writeData,bytes);
	
}

void Read_from_SPI_FRAM(uint8_t *readData,uint16_t Adress,uint8_t bytes)
{
	
	SPIC_Write(READ);
	SPIC_Write_16bit_address(Adress);
	SPIC_Read_array(readData,bytes);
	
}

void read_profile(p_Speed_profile_t proile,uint8_t ID,bool use_twi,uint16_t offset)
{
	if(use_twi)
		twi_read(&TWIE,(uint8_t *)proile,FRAM_I2C_ADDR,offset + sizeof(Speed_profile_t) * (uint16_t) ID,sizeof(Speed_profile_t),0);
	else
	{
		PORTC.OUTCLR = PIN4_bm;
		Read_from_SPI_FRAM((uint8_t *)proile,offset + sizeof(Speed_profile_t) * (uint16_t) ID,sizeof(Speed_profile_t));
		PORTC.OUTSET = PIN4_bm;
	}
	
	
}

void read_param_profile(p_regulator_parameters_t proile,uint8_t ID,bool use_twi,uint16_t offset)
{
	if(use_twi)
	twi_read(&TWIE,(uint8_t *)proile,FRAM_I2C_ADDR,offset + sizeof(p_regulator_parameters_t) * (uint16_t) ID,sizeof(p_regulator_parameters_t),0);
	else
	{
		PORTC.OUTCLR = PIN4_bm;
		Read_from_SPI_FRAM((uint8_t *)proile,offset + sizeof(p_regulator_parameters_t) * (uint16_t) ID,sizeof(p_regulator_parameters_t));
		PORTC.OUTSET = PIN4_bm;
	}
	
	
}

void write_profile(p_Speed_profile_t proile,uint8_t ID,bool use_twi,uint16_t offset)
{
	if(use_twi)
		twi_write(&TWIE,(uint8_t *)proile,FRAM_I2C_ADDR,offset + sizeof(Speed_profile_t) * (uint16_t) ID,sizeof(Speed_profile_t),0,0,0);
	else
	{
		set_WREN();
		_delay_ms(2);
		PORTC.OUTCLR = PIN4_bm;
		Write_to_SPI_FRAM((uint8_t *)proile,offset + sizeof(Speed_profile_t) * (uint16_t) ID,sizeof(Speed_profile_t));
		PORTC.OUTSET = PIN4_bm;
	}
	
}

void write_param_profile(p_regulator_parameters_t proile,uint8_t ID,bool use_twi,uint16_t offset)
{
	if(use_twi)
	twi_write(&TWIE,(uint8_t *)proile,FRAM_I2C_ADDR,offset + sizeof(p_regulator_parameters_t) * (uint16_t) ID,sizeof(p_regulator_parameters_t),0,0,0);
	else
	{
		set_WREN();
		_delay_ms(2);
		PORTC.OUTCLR = PIN4_bm;
		Write_to_SPI_FRAM((uint8_t *)proile,offset + sizeof(p_regulator_parameters_t) * (uint16_t) ID,sizeof(p_regulator_parameters_t));
		PORTC.OUTSET = PIN4_bm;
	}
	
}