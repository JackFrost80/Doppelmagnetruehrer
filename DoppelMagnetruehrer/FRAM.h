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

void read_profile(p_Speed_profile_t proile,uint8_t ID);
void write_profile(p_Speed_profile_t proile,uint8_t ID);



#endif /* FRAM_H_ */