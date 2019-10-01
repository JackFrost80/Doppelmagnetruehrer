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


#define offset_settings_regulator 0x00
#define offset_settings_system  sizeof(regulator_parameters_t)*2 + offset_settings_regulator
#define offset_profile_a sizeof(system_settings_t) + offset_settings_system
#define offset_profile_b sizeof(Speed_profile_t) + offset_profile_a
#define offset_profiles sizeof(Speed_profile_t) + offset_profile_b


typedef struct system_settings {
	
	uint8_t show_debug;
	uint8_t show_profile;
	uint8_t type_of_encoder;
	uint32_t CRC_value;

} system_settings_t,*p_system_settings_t;

typedef struct regulator_parameters {
	
	uint32_t I_speed_0;
	uint32_t I_voltage_0;
	uint32_t I_speed_1;
	uint32_t I_voltage_1;
	uint32_t CRC_value;

} regulator_parameters_t,*p_regulator_parameters_t;

typedef struct Speed_profile {
	
	uint8_t ID;
	uint8_t status;
	uint8_t type;
	uint8_t erlenmayer_size;
	bool switchoff;
	bool slowdown;
	bool speedup;
	bool slowdown_switch;
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
void init_profile_mod();
void change_parameter_settings(p_regulator_parameters_t paramters_,bool seite, uint8_t *position_,char *int_buffer,bool *screen_clear_,uint8_t *old_position_);
void init_profile_(p_Speed_profile_t p_profile_);

#endif /* MAIN_H_ */