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
#include <avr/interrupt.h>
#include <util/delay.h>

void read_profile(p_Speed_profile_t proile,uint8_t ID)
{
	twi_read(&TWIE,(uint8_t *)proile,FRAM_I2C_ADDR,sizeof(Speed_profile_t) * (uint16_t) ID,sizeof(Speed_profile_t),0);
	
	
}

void write_profile(p_Speed_profile_t proile,uint8_t ID)
{
	
	twi_write(&TWIE,(uint8_t *)proile,FRAM_I2C_ADDR,sizeof(Speed_profile_t) * (uint16_t) ID,sizeof(Speed_profile_t),0,0,0);
	
}