/*
 * main.h
 *
 * Created: 06.02.2019 23:27:11
 *  Author: JackFrost
 */ 


#ifndef MAIN_H_
#define MAIN_H_
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define prepared 0
#define running_fast 1
#define running_slow 2

#define type_normal 0
#define type_switch 1
#define type_stop	2

#define auto_aus	0
#define auto_an		1
#define manuel		3
typedef struct Speed_profile {
	
	uint8_t ID;
	uint8_t status;
	uint8_t type;
	bool switchoff;
	bool slowdown;
	bool speedup;
	uint16_t speed_slow;
	uint16_t speed_fast;
	uint32_t time_fast_init;
	uint32_t total_time;
	uint32_t time_slow;
	uint32_t speed_time;
	uint32_t CRC_value;
} Speed_profile_t, *p_Speed_profile_t;

typedef struct Menu {
	uint8_t sub_menu;
	uint8_t menu_point;
}Menu_t, *p_Menu_t;

void AddTotAvg(uint16_t * io_pFloatAvgFilter,uint8_t * Index ,uint16_t  i_NewValue);
int8_t encode_read2( void );
void display_time_menu(volatile uint32_t *timestamp,bool invert,uint8_t start, uint8_t ende);
void update_proile_time(uint32_t *timevalue, int16_t change,uint32_t max_value);

#endif /* MAIN_H_ */