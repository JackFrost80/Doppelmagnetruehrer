/*
 * twi.cpp
 *
 * Created: 14.05.2017 21:10:33
 *  Author: JackFrost
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


void twi_write(TWI_t *twiname, uint8_t *writeData,uint8_t page_adress,uint16_t Adress, uint8_t bytes,bool fixed,bool skip_address,bool skip_high_address){

	uint16_t i;
	TWIE_MASTER_CTRLC &= ~((1<<TWI_MASTER_ACKACT_bp));
	twiname->MASTER.ADDR = page_adress;
	while (!(TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm));
	if(TWIE_MASTER_STATUS & (1<<TWI_MASTER_ARBLOST_bp))
	{
		twiname->MASTER.CTRLA = 0;
		PORTE.DIRSET = PIN1_bm;
		for(uint8_t i=0;i<9;i++)
		{
			PORTE.OUTSET = PIN1_bm;
			_delay_us(20);
			PORTE.OUTCLR = PIN1_bm;
			_delay_us(20);
		}
		PORTE.DIRCLR = PIN1_bm;
		twiname->MASTER.CTRLA = TWI_MASTER_ENABLE_bm;
		twiname->MASTER.STATUS = TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm;
		twiname->MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
		TWIE.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		twiname->MASTER.ADDR = page_adress;
		if(TWIE_MASTER_STATUS & (1<<TWI_MASTER_ARBLOST_bp));
		
	}
	if(!skip_address)
	{
		if(!skip_high_address)
		{
			twiname->MASTER.DATA = Adress>>8;
			while (!(twiname->MASTER.STATUS & TWI_MASTER_WIF_bm));
		}
		
		twiname->MASTER.DATA = Adress;
		while (!(twiname->MASTER.STATUS & TWI_MASTER_WIF_bm));
	}
	for(i=0;i<bytes;i++){
		if(!fixed)             // write data 
		twiname->MASTER.DATA =writeData[i];
		else
		twiname->MASTER.DATA =writeData[0];
		while(!(twiname->MASTER.STATUS&TWI_MASTER_WIF_bm));
	}
	TWIE.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
}

void twi_read(TWI_t *twiname, uint8_t *readData,uint8_t page_adress, uint16_t Adress, uint8_t bytes,bool skip_address)
{
	twiname->MASTER.CTRLC &= ~((1<<TWI_MASTER_ACKACT_bp));
	twiname->MASTER.ADDR = page_adress;
	while (!(twiname->MASTER.STATUS & TWI_MASTER_WIF_bm));
	if(twiname->MASTER.STATUS & (1<<TWI_MASTER_ARBLOST_bp))
	{
		twiname->MASTER.CTRLA = 0;
		PORTC.DIRSET = PIN1_bm;
		for(uint8_t i=0;i<9;i++)
		{
			PORTC.OUTSET = PIN1_bm;
			_delay_us(20);
			PORTC.OUTCLR = PIN1_bm;
			_delay_us(20);
		}
		PORTC.DIRCLR = PIN1_bm;
		twiname->MASTER.CTRLA = TWI_MASTER_ENABLE_bm;
		twiname->MASTER.STATUS = TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm;
		twiname->MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
		twiname->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		twiname->MASTER.ADDR = page_adress;
		while (!(twiname->MASTER.STATUS & TWI_MASTER_WIF_bm));
	}
	if(!skip_address)
	{
		twiname->MASTER.DATA = Adress >> 8;
		while (!(twiname->MASTER.STATUS & TWI_MASTER_WIF_bm));
		twiname->MASTER.DATA = Adress ;
		while (!(twiname->MASTER.STATUS & TWI_MASTER_WIF_bm));
	}
	page_adress |= 0x01;
	twiname->MASTER.ADDR = page_adress;
	for(volatile uint8_t j=0 ;j<bytes ;j++){
		
		while(!(twiname->MASTER.STATUS&TWI_MASTER_RIF_bm));
		if(j == (bytes-1))
		{
			twiname->MASTER.CTRLC = (1<<TWI_MASTER_ACKACT_bp) | TWI_MASTER_CMD_STOP_gc;
		}
		readData[j] = twiname->MASTER.DATA;
	}
}


