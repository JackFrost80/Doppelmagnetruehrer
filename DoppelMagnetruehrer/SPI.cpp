/*
 * SPI.cpp
 *
 * Created: 05.04.2019 23:50:14
 *  Author: JackFrost
 */ 

#include "SPI.h"
#include "FM25CL64B.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>


void SPIC_Init(void)
{
	
	// PC4 output (SS FRAM)
	PORTC.DIRSET = PIN4_bm;
	// PF5 output (MOSI SPIC)
	PORTC.DIRSET = PIN5_bm;
	// PF6 input (MISO SPIC)
	PORTC.DIRCLR = PIN6_bm;
	// PF7 output (SCK SPIC)
	PORTC.DIRSET = PIN7_bm;
	
	//SS disabled (high)
	PORTC.OUTSET = PIN4_bm;
		
	//Prescaler at 16 and CLK2X off  --> CLK = 1MHz
	//Mode 0 (CPOL=0 CPHA=0)
	//Master mode
	SPIC.CTRL = SPI_MODE_3_gc | SPI_PRESCALER_DIV4_gc | SPI_ENABLE_bm | SPI_MASTER_bm | SPI_CLK2X_bm;
	
}

unsigned char SPIC_Read_Write(unsigned char data)
{
	SPIC.DATA = data; //send
	while(!(SPIC.STATUS & (1<<SPI_IF_bp))); //wait completion
	return SPIC.DATA;
	
}

void SPIC_Write(unsigned char data)
{
	SPIC.DATA = data; //send
	while(!(SPIC.STATUS & (1<<SPI_IF_bp))); //wait completion
	uint8_t dummy = SPIC.DATA;

	
}

void SPIC_Write_16bit_address(uint16_t address)
{
	SPIC_Write((uint8_t)(address >> 8));
	SPIC_Write((uint8_t)address);
}


void SPIC_Write_array(uint8_t *data, uint8_t length)
{
	for(uint8_t i = 0; i<length;i++)
	{
		SPIC.DATA = data[i]; //send
		while(!(SPIC.STATUS & (1<<SPI_IF_bp))); //wait completion
		uint8_t dummy = SPIC.DATA;
	}
}

void SPIC_Read_array(uint8_t *data, uint8_t length)
{
	for(uint8_t i = 0; i<length;i++)
	{
		SPIC.DATA = 0xFF; //send
		while(!(SPIC.STATUS & (1<<SPI_IF_bp))); //wait completion
		data[i] = SPIC.DATA;
	}
}