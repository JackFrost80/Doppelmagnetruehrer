/*
 * SPI.h
 *
 * Created: 05.04.2019 23:50:30
 *  Author: JackFrost
 */ 


#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>



void SPIC_Init(void);
unsigned char SPIC_Read_Write(unsigned char data);
void SPIC_Write(unsigned char data);
void SPIC_Write_array(uint8_t *data, uint8_t length);
void SPIC_Read_array(uint8_t *data, uint8_t length);
void SPIC_Write_16bit_address(uint16_t address);


#endif /* SPI_H_ */