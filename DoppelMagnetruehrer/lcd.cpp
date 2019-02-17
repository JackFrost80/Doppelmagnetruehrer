/*
 * lcd.cpp
 *
 * Created: 02.02.2019 22:44:05
 *  Author: JackFrost
 */ 

/*
 * This file is part of lcd library for ssd1306 oled-display.
 *
 * lcd library for ssd1306 oled-display is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or any later version.
 *
 * lcd library for ssd1306 oled-display is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Diese Datei ist Teil von lcd library for ssd1306 oled-display.
 *
 * lcd library for ssd1306 oled-display ist Freie Software: Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder jeder späteren
 * veröffentlichten Version, weiterverbreiten und/oder modifizieren.
 *
 * lcd library for ssd1306 oled-display wird in der Hoffnung, dass es nützlich sein wird, aber
 * OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite
 * Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License für weitere Details.
 *
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.
 *
 *  lcd.c
 *
 *  Created by Michael Köhler on 22.12.16.
 *  Copyright 2016 Skie-Systems. All rights reserved.
 *
 */
/* Standard ASCII 6x8 font */

#include "lcd.h"
#include "twi.h"
#include "SD1306.h"
#include <avr/interrupt.h>

static const uint8_t ssd1306oled_font6x8 [] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sp
    0x00, 0x00, 0x00, 0x2f, 0x00, 0x00, // !
    0x00, 0x00, 0x07, 0x00, 0x07, 0x00, // "
    0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14, // #
    0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12, // $
    0x00, 0x62, 0x64, 0x08, 0x13, 0x23, // %
    0x00, 0x36, 0x49, 0x55, 0x22, 0x50, // &
    0x00, 0x00, 0x05, 0x03, 0x00, 0x00, // '
    0x00, 0x00, 0x1c, 0x22, 0x41, 0x00, // (
    0x00, 0x00, 0x41, 0x22, 0x1c, 0x00, // )
    0x00, 0x14, 0x08, 0x3E, 0x08, 0x14, // *
    0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, // +
    0x00, 0x00, 0x00, 0xA0, 0x60, 0x00, // ,
    0x00, 0x08, 0x08, 0x08, 0x08, 0x08, // -
    0x00, 0x00, 0x60, 0x60, 0x00, 0x00, // .
    0x00, 0x20, 0x10, 0x08, 0x04, 0x02, // /
    0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
    0x00, 0x00, 0x42, 0x7F, 0x40, 0x00, // 1
    0x00, 0x42, 0x61, 0x51, 0x49, 0x46, // 2
    0x00, 0x21, 0x41, 0x45, 0x4B, 0x31, // 3
    0x00, 0x18, 0x14, 0x12, 0x7F, 0x10, // 4
    0x00, 0x27, 0x45, 0x45, 0x45, 0x39, // 5
    0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30, // 6
    0x00, 0x01, 0x71, 0x09, 0x05, 0x03, // 7
    0x00, 0x36, 0x49, 0x49, 0x49, 0x36, // 8
    0x00, 0x06, 0x49, 0x49, 0x29, 0x1E, // 9
    0x00, 0x00, 0x36, 0x36, 0x00, 0x00, // :
    0x00, 0x00, 0x56, 0x36, 0x00, 0x00, // ;
    0x00, 0x08, 0x14, 0x22, 0x41, 0x00, // <
    0x00, 0x14, 0x14, 0x14, 0x14, 0x14, // =
    0x00, 0x00, 0x41, 0x22, 0x14, 0x08, // >
    0x00, 0x02, 0x01, 0x51, 0x09, 0x06, // ?
    0x00, 0x32, 0x49, 0x59, 0x51, 0x3E, // @
    0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C, // A
    0x00, 0x7F, 0x49, 0x49, 0x49, 0x36, // B
    0x00, 0x3E, 0x41, 0x41, 0x41, 0x22, // C
    0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C, // D
    0x00, 0x7F, 0x49, 0x49, 0x49, 0x41, // E
    0x00, 0x7F, 0x09, 0x09, 0x09, 0x01, // F
    0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A, // G
    0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F, // H
    0x00, 0x00, 0x41, 0x7F, 0x41, 0x00, // I
    0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, // J
    0x00, 0x7F, 0x08, 0x14, 0x22, 0x41, // K
    0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, // L
    0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F, // M
    0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F, // N
    0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E, // O
    0x00, 0x7F, 0x09, 0x09, 0x09, 0x06, // P
    0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
    0x00, 0x7F, 0x09, 0x19, 0x29, 0x46, // R
    0x00, 0x46, 0x49, 0x49, 0x49, 0x31, // S
    0x00, 0x01, 0x01, 0x7F, 0x01, 0x01, // T
    0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F, // U
    0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F, // V
    0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F, // W
    0x00, 0x63, 0x14, 0x08, 0x14, 0x63, // X
    0x00, 0x07, 0x08, 0x70, 0x08, 0x07, // Y
    0x00, 0x61, 0x51, 0x49, 0x45, 0x43, // Z
    0x00, 0x00, 0x7F, 0x41, 0x41, 0x00, // [
    0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55, // backslash
    0x00, 0x00, 0x41, 0x41, 0x7F, 0x00, // ]
    0x00, 0x04, 0x02, 0x01, 0x02, 0x04, // ^
    0x00, 0x40, 0x40, 0x40, 0x40, 0x40, // _
    0x00, 0x00, 0x01, 0x02, 0x04, 0x00, // '
    0x00, 0x20, 0x54, 0x54, 0x54, 0x78, // a
    0x00, 0x7F, 0x48, 0x44, 0x44, 0x38, // b
    0x00, 0x38, 0x44, 0x44, 0x44, 0x20, // c
    0x00, 0x38, 0x44, 0x44, 0x48, 0x7F, // d
    0x00, 0x38, 0x54, 0x54, 0x54, 0x18, // e
    0x00, 0x08, 0x7E, 0x09, 0x01, 0x02, // f
    0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C, // g
    0x00, 0x7F, 0x08, 0x04, 0x04, 0x78, // h
    0x00, 0x00, 0x44, 0x7D, 0x40, 0x00, // i
    0x00, 0x40, 0x80, 0x84, 0x7D, 0x00, // j
    0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, // k
    0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, // l
    0x00, 0x7C, 0x04, 0x18, 0x04, 0x78, // m
    0x00, 0x7C, 0x08, 0x04, 0x04, 0x78, // n
    0x00, 0x38, 0x44, 0x44, 0x44, 0x38, // o
    0x00, 0xFC, 0x24, 0x24, 0x24, 0x18, // p
    0x00, 0x18, 0x24, 0x24, 0x18, 0xFC, // q
    0x00, 0x7C, 0x08, 0x04, 0x04, 0x08, // r
    0x00, 0x48, 0x54, 0x54, 0x54, 0x20, // s
    0x00, 0x04, 0x3F, 0x44, 0x40, 0x20, // t
    0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C, // u
    0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C, // v
    0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C, // w
    0x00, 0x44, 0x28, 0x10, 0x28, 0x44, // x
    0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C, // y
    0x00, 0x44, 0x64, 0x54, 0x4C, 0x44, // z
	0x00, 0x00, 0x08, 0x77, 0x41, 0x00, // {
	0x00, 0x00, 0x00, 0x63, 0x00, 0x00, // ¦
	0x00, 0x00, 0x41, 0x77, 0x08, 0x00, // }
	0x00, 0x08, 0x04, 0x08, 0x08, 0x04, // ~
	0x00, 0x3D, 0x40, 0x40, 0x20, 0x7D, // ü
	0x00, 0x3D, 0x40, 0x40, 0x40, 0x3D, // Ü
	0x00, 0x21, 0x54, 0x54, 0x54, 0x79, // ä
	0x00, 0x7D, 0x12, 0x11, 0x12, 0x7D, // Ä
	0x00, 0x39, 0x44, 0x44, 0x44, 0x39, // ö
	0x00, 0x3D, 0x42, 0x42, 0x42, 0x3D, // Ö
	0x00, 0x02, 0x05, 0x02, 0x00, 0x00, // °
	0x00, 0x7E, 0x01, 0x49, 0x55, 0x73, // ß
};

const uint8_t ssd1306_init_sequence [] PROGMEM = {	// Initialization Sequence
	LCD_DISP_OFF,			// Display OFF (sleep mode)
	0x20, 0x00,		// Set Memory Addressing Mode
	// 00=Horizontal Addressing Mode; 01=Vertical Addressing Mode;
	// 10=Page Addressing Mode (RESET); 11=Invalid
	0xB0,			// Set Page Start Address for Page Addressing Mode, 0-7
	0xC8,			// Set COM Output Scan Direction
	0x00,			// --set low column address
	0x10,			// --set high column address
	0x40,			// --set start line address
	0x81, 0x3F,		// Set contrast control register
	0xA1,			// Set Segment Re-map. A0=address mapped; A1=address 127 mapped.
	0xA6,			// Set display mode. A6=Normal; A7=Inverse
	0xA8, 0x3F,		// Set multiplex ratio(1 to 64)
	0xA4,			// Output RAM to Display
	// 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
	0xD3, 0x00,		// Set display offset. 00 = no offset
	0xD5,			// --set display clock divide ratio/oscillator frequency
	0xF0,			// --set divide ratio
	0xD9, 0x22,		// Set pre-charge period
	0xDA, 0x12,		// Set com pins hardware configuration
	0xDB,			// --set vcomh
	0x20,			// 0x20,0.77xVcc
	0x8D, 0x14,		// Set DC-DC enable

};

//const uint8_t ssd1306_init_sequence [] PROGMEM = {	// Initialization Sequence
//LCD_DISP_OFF,			// Display OFF (sleep mode)
//0x00, 0x10,		// Set Memory Addressing Mode
				//// 00=Horizontal Addressing Mode; 01=Vertical Addressing Mode;
				//// 10=Page Addressing Mode (RESET); 11=Invalid
//0x40,			// Set Page Start Address for Page Addressing Mode, 0-7
//0xB0,			// Set COM Output Scan Direction
//0x81,			// --set low column address
//0x66,			// --set high column address
//0xA1,			// --set start line address
//0xA6,			// Set contrast control register
//0xA8,			// Set Segment Re-map. A0=address mapped; A1=address 127 mapped. 
				//// Set display mode. A6=Normal; A7=Inverse
	 //0x3F,		// Set multiplex ratio(1 to 64)
//0xC8,			// Output RAM to Display
				//// 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
//0xD3, 0x00,		// Set display offset. 00 = no offset
//0xD5,			// --set display clock divide ratio/oscillator frequency
//0x80,			// --set divide ratio
//0xD9, 0x1f,		// Set pre-charge period
//0xDA, 0x12,		// Set com pins hardware configuration		
//0xDB,			// --set vcomh
//0x30,			// 0x20,0.77xVcc
//0x8D, 0x14,		// Set DC-DC enable
//
//};

void lcd_home(void){
	lcd_gotoxy(0, 0);
}



void lcd_init(uint8_t dispAttr){
	uint8_t init1[] = {
		SSD1306_DISPLAYOFF,                   // 0xAE
		SSD1306_SETDISPLAYCLOCKDIV,           // 0xD5
		0x80,                                 // the suggested ratio 0x80
		SSD1306_SETMULTIPLEX }; // 0xA8
	twi_write(&TWIE,init1,LCD_I2C_ADDR,COMMAND,4,0,0);
	uint8_t helper = SSD1306_LCDHEIGHT - 1;
	twi_write(&TWIE,&helper,LCD_I2C_ADDR,COMMAND,1,0,0);
	uint8_t init2[] = {
		SSD1306_SETDISPLAYOFFSET,             // 0xD3
		0x0,                                  // no offset
		SSD1306_SETSTARTLINE | 0x0,           // line #0
		SSD1306_CHARGEPUMP }; // 0x8D
	twi_write(&TWIE,init2,LCD_I2C_ADDR,COMMAND,4,0,0);
	helper = (dispAttr == SSD1306_EXTERNALVCC) ? 0x10 : 0x14;
	twi_write(&TWIE,&helper,LCD_I2C_ADDR,COMMAND,1,0,0);
	uint8_t init3[] = {
		SSD1306_MEMORYMODE,                   // 0x20
		0x00,                                 // 0x0 act like ks0108
		SSD1306_SEGREMAP | 0x1,
		SSD1306_COMSCANDEC };
	twi_write(&TWIE,init3,LCD_I2C_ADDR,COMMAND,4,0,0);
	uint8_t init4b[] = {
		SSD1306_SETCOMPINS,                 // 0xDA
		0x12,
	SSD1306_SETCONTRAST };              // 0x81
	twi_write(&TWIE,init4b,LCD_I2C_ADDR,COMMAND,3,0,0);
	helper = (dispAttr == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF;
	twi_write(&TWIE,&helper,LCD_I2C_ADDR,COMMAND,1,0,0);
	helper = SSD1306_SETPRECHARGE; // 0xd9
	twi_write(&TWIE,&helper,LCD_I2C_ADDR,COMMAND,1,0,0);
	helper = (dispAttr == SSD1306_EXTERNALVCC) ? 0x22 : 0xF1;
	twi_write(&TWIE,&helper,LCD_I2C_ADDR,COMMAND,1,0,0);
	uint8_t  init5[] = {
		SSD1306_SETVCOMDETECT,               // 0xDB
		0x40,
		SSD1306_DISPLAYALLON_RESUME,         // 0xA4
		SSD1306_NORMALDISPLAY,               // 0xA6
		SSD1306_DEACTIVATE_SCROLL,
		SSD1306_DISPLAYON }; // Main screen turn on
	twi_write(&TWIE,init5,LCD_I2C_ADDR,COMMAND,6,0,0);
	lcd_clrscr();
}


//void lcd_command(uint8_t cmd) {
    //lcd_send_i2c_start();
    //lcd_send_i2c_byte(0x00);	// 0x00 for command, 0x40 for data
    //lcd_send_i2c_byte(cmd);
    //lcd_send_i2c_stop();
//}
//void lcd_data(uint8_t data) {
    //lcd_send_i2c_start();
    //lcd_send_i2c_byte(0x40);	// 0x00 for command, 0x40 for data
    //lcd_send_i2c_byte(data);
    //lcd_send_i2c_stop();
//}
void lcd_gotoxy(uint8_t x, uint8_t y){
	x *= 6;
	uint8_t helper[4];
	helper[0] = 0xb0 + y;	// set page start to y
	helper[1] = 0x21;		// set column start	
	helper[2] = x; 			// to x
	helper[3] = 0x7f;		// set column end to 127
	twi_write(&TWIE,helper,LCD_I2C_ADDR,0x00,4,0,0);  // Write data to I2C
}
void lcd_clrscr(void){
    lcd_home();
	uint8_t helper = 0x00 ;
	for(uint8_t i=0;i<8;i++)
		twi_write(&TWIE,&helper,LCD_I2C_ADDR,0x40,128,1,0);
	lcd_home();
}
void lcd_putc(char c){
	static uint8_t data[6];
	if((c > 127 || 
	   c < 32) &&
	   (c != 188 && 
		c != 182 &&
		c != 176 &&
		c != 164 &&
		c != 159 &&
		c != 156 &&
		c != 150 &&
		c != 132 ) ) return;
    c -= 32;
	if( c < 127-32 ) {
		for (uint8_t i = 0; i < 6; i++)
		{
			data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
		}
		twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
		return;
	} 
	switch (c) {
		case 156:
			c = 95; // ü
			for (uint8_t i = 0; i < 6; i++)
			{
				data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
			}
			twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
			break;
		case 150:
			c = 99; // ö
			for (uint8_t i = 0; i < 6; i++)
			{
				data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
			}
			twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
			break;
		case 144:
			c = 101; // °
			for (uint8_t i = 0; i < 6; i++)
			{
				data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
			}
			twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
			break;
		case 132:
			c = 97; // ä
			for (uint8_t i = 0; i < 6; i++)
			{
				data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
			}
			twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
			break;
		case 127:
			c = 102; // ß
			for (uint8_t i = 0; i < 6; i++)
			{
				data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
			}
			twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
			break;
		case 124:
			c = 96; // Ü
			for (uint8_t i = 0; i < 6; i++)
			{
				data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
			}
			twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
			break;
		case 118:
			c = 100; // Ö
			for (uint8_t i = 0; i < 6; i++)
			{
				data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
			}
			twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
			break;
		case 100:
			c = 98; // Ä
			for (uint8_t i = 0; i < 6; i++)
			{
				data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
			}
			twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
			break;
		default:
			break;
	}
    
}

void lcd_putc_invert(char c){
	static uint8_t data[6];
	if((c > 127 ||
	c < 32) &&
	(c != 188 &&
	c != 182 &&
	c != 176 &&
	c != 164 &&
	c != 159 &&
	c != 156 &&
	c != 150 &&
	c != 132 ) ) return;
	c -= 32;
	if( c < 127-32 ) {
		for (uint8_t i = 0; i < 6; i++)
		{
			data[i] = (pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]))^0xff;	// print font to ram, print 6 columns
		}
		twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
		return;
	}
	switch (c) {
		case 156:
		c = 95; // ü
		for (uint8_t i = 0; i < 6; i++)
		{
			data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
		}
		twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
		break;
		case 150:
		c = 99; // ö
		for (uint8_t i = 0; i < 6; i++)
		{
			data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
		}
		twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
		break;
		case 144:
		c = 101; // °
		for (uint8_t i = 0; i < 6; i++)
		{
			data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
		}
		twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
		break;
		case 132:
		c = 97; // ä
		for (uint8_t i = 0; i < 6; i++)
		{
			data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
		}
		twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
		break;
		case 127:
		c = 102; // ß
		for (uint8_t i = 0; i < 6; i++)
		{
			data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
		}
		twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
		break;
		case 124:
		c = 96; // Ü
		for (uint8_t i = 0; i < 6; i++)
		{
			data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
		}
		twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
		break;
		case 118:
		c = 100; // Ö
		for (uint8_t i = 0; i < 6; i++)
		{
			data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
		}
		twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
		break;
		case 100:
		c = 98; // Ä
		for (uint8_t i = 0; i < 6; i++)
		{
			data[i] = pgm_read_byte(&ssd1306oled_font6x8[c * 6 + i]);	// print font to ram, print 6 columns
		}
		twi_write(&TWIE,data,LCD_I2C_ADDR,DATA,6,0,0);
		break;
		default:
		break;
	}
	
}
void lcd_puts(const char* s){
	while (*s) {
		lcd_putc(*s++);
	}
}
void lcd_puts_invert(const char* s){
	while (*s) {
		lcd_putc_invert(*s++);
	}
}

void lcd_puts_invert_pos(const char* s,uint8_t start,uint8_t ende){
	uint8_t position  = 0;
	while (*s) {
		if(position >= start && position <= ende)
			lcd_putc_invert(*s++);
		else
			lcd_putc(*s++);
		position++;
	}
}


//void lcd_puts_p(const char* progmem_s){
    //register uint8_t c;
    //while ((c = pgm_read_byte(progmem_s++))) {
        //lcd_putc(c);
    //}
//}
//void lcd_invert(uint8_t invert){
    //lcd_send_i2c_start();
    //lcd_send_i2c_byte(0x00);	// 0x00 for command, 0x40 for data
    //if (invert == YES) {
        //lcd_send_i2c_byte(0xA7);// set display inverted mode
    //} else {
        //lcd_send_i2c_byte(0xA6);// set display normal mode
    //}
    //lcd_send_i2c_stop();
//}
//void lcd_set_contrast(uint8_t contrast){
    //lcd_send_i2c_start();
    //lcd_send_i2c_byte(0x00);	// 0x00 for command, 0x40 for data
    //lcd_send_i2c_byte(0x81);	// set display contrast
    //lcd_send_i2c_byte(contrast);// to contrast
    //lcd_send_i2c_stop();
//}