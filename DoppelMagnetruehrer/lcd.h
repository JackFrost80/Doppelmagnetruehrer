/*
 * lcf.h
 *
 * Created: 02.02.2019 22:44:16
 *  Author: JackFrost
 */ 


#ifndef LCF_H_
#define LCF_H_

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
 *  lcd.h
 *
 *  Created by Michael Köhler on 22.12.16.
 *  Copyright 2016 Skie-Systems. All rights reserved.
 *
 *  lib for OLED-Display with SSD1306-Controller
 *  first dev-version only for I2C-Connection
 *  at ATMega328P like Arduino Uno or ATMega168PA/88PA/48PA
 *
 */

#ifndef LCD_H
#define LCD_H

#ifndef YES
#define YES				1
#define NO				0
#else
#error "Check #defines for YES and NO in other sources !"
#endif

#if (__GNUC__ * 100 + __GNUC_MINOR__) < 303
#error "This library requires AVR-GCC 3.3 or later, update to newer AVR-GCC compiler !"
#endif

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

/* TODO: setup i2c/twi */
#define LCD_I2C_ADDR	0x78	// refer lcd-manual, 0x78 for 8-bit-adressmode, 0x3c for 7-bit-adressmode
#define PCA_I2C_ADDR	0x82

#define LCD_DISP_OFF	0xAE
#define LCD_DISP_ON		0xAF

#define Output_register 0x01;
#define config_register 0x03;


void lcd_init(uint8_t dispAttr);
void lcd_home(void);
void lcd_reset();
void PCA_config();

void lcd_command(uint8_t cmd);				// transmit command to display
void lcd_data(uint8_t data);				// transmit data to display
void lcd_gotoxy(uint8_t x, uint8_t y);		// set curser at pos x, y. x means character, y means line (page, refer lcd manual)
void lcd_clrscr(void);						// clear screen
void lcd_putc(char c);						// print character on screen
void lcd_puts(const char* s);				// print string, \n-terminated, from ram on screen
void lcd_puts_p(const char* progmem_s);		// print string from flash on screen
void lcd_invert(uint8_t invert);			// invert display
void lcd_set_contrast(uint8_t contrast);	// set contrast for display
void lcd_puts_invert(const char* s);
void lcd_putc_invert(char c);
void lcd_puts_invert_pos(const char* s,uint8_t start,uint8_t ende);
#endif /*  LCD_H  */



#endif /* LCF_H_ */