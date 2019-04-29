/*
 * FRAM.h
 *
 * Created: 03.03.2019 22:22:20
 *  Author: JackFrost
 */ 


#ifndef FRAM_H_
#define FRAM_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "main.h"

#define FRAM_I2C_ADDR	0xA0

void set_WREN();
void read_profile(p_Speed_profile_t proile,uint8_t ID,bool use_twi,uint16_t offset);
void write_profile(p_Speed_profile_t proile,uint8_t ID,bool use_twi,uint16_t offset);
void read_param_profile(p_regulator_parameters_t proile,uint8_t ID,bool use_twi,uint16_t offset);
void write_param_profile(p_regulator_parameters_t proile,uint8_t ID,bool use_twi,uint16_t offset);
void Write_to_SPI_FRAM(uint8_t *writeData,uint16_t Adress,uint8_t bytes);
void Read_from_SPI_FRAM(uint8_t *readData,uint16_t Adress,uint8_t bytes);




#endif /* FRAM_H_ */