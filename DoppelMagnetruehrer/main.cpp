/*
 * DoppelMagnetruehrer.cpp
 *
 * Created: 02.02.2019 11:51:42
 * Author : JackFrost
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "lcd.h"
#include "twi.h"
#include "main.h"
#define CLK_Prescaler 0x00
#define PLL_Faktor 16
#define CPU_SPEED 32000000UL
#define BAUDRATE    400000UL
#define TWI_BAUD(F_SYS, F_TWI) ((F_SYS / (2 * F_TWI)) - 5)
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)
#define left 0x01
#define right 0x0C
#define PHASE_A     (PORTA.IN & PIN2_bm)
#define PHASE_B     (PORTA.IN & PIN3_bm)
#define KEY_PIN         PORTA.IN
#define KEY0            4
#define ALL_KEYS        (1<<KEY0 )

#define REPEAT_MASK     (1<<KEY0 )       // repeat: key1, key2
#define REPEAT_START    50                        // after 500ms
#define REPEAT_NEXT     20                        // every 200ms

//Blinkfrequenzen
#define Frequenz_8Hz 0x01
#define Frequenz_4Hz 0x02
#define Frequenz_2Hz 0x04
#define Frequenz_1Hz 0x08
#define Frequenz_0_5Hz 0x10

#define profile_size sizeof(Speed_profile_t)
#define max_profile_time 2UL*86400UL




volatile uint16_t ADC_Value_a = 0x400;
volatile uint16_t ADC_Value_b = 0x400;
volatile uint16_t Sollwert_a = 0x855;
volatile uint16_t Sollwert_b = 0xF21;
volatile uint16_t Sollwert_RPM_a = 600;
volatile uint16_t Sollwert_RPM_b = 600;
volatile uint32_t Unixtimestamp = 0;
volatile uint8_t Blink = 0;
uint32_t I_Anteil = 100000;
uint32_t I_Anteil_speed = 100000;
volatile uint32_t RPM_a = 0; 
volatile uint32_t RPM_b = 0; 
volatile bool calc_regulator = false;
volatile int8_t enc_delta;          // -128 ... 127
static int8_t last;
volatile bool recalc_time = false;
bool running_a = false;
bool running_b = false;
bool hand_a = false;
bool hand_b = false;
volatile uint8_t key_state;                                // debounced and inverted key state:
// bit = 1: key pressed
volatile uint8_t key_press;                                // key press detect

volatile uint8_t key_rpt;                                  // key long press and repeat

Speed_profile_t Profile_a;
p_Speed_profile_t  p_Profile_a = &Profile_a;
Speed_profile_t Profile_b;
p_Speed_profile_t  p_Profile_b = &Profile_b;
Speed_profile_t EEPROM_profile[12] EEMEM;
uint8_t profile_ID_a EEMEM = 0;
uint8_t profile_ID_b EEMEM = 0;
bool show_menu = false;
bool screen_clear = false;
Menu_t menu = {0,0};


bool end_a = false;
bool end_b = false;

uint16_t RPM_b_raw[16] = {0};
uint8_t IndexNextValue_b=0;
uint16_t RPM_a_raw[16] = {0};
uint8_t IndexNextValue_a=0;


ISR(RTC_OVF_vect)
{
	Blink++;
	if(Blink % 8 == 0)
	{
		Unixtimestamp++;
		recalc_time =  true;
	}
	
	calc_regulator = true;
	
	
	
	
}

ISR( TCC1_CCA_vect )                            // Voltage controller a
{
	AddTotAvg(RPM_a_raw,&IndexNextValue_a,( 15000000UL / (uint32_t)TCC1.CCA ));	
	//if(TCC1.CCA > 1000)
	//RPM_a = ( 15000000UL / (uint32_t)TCC1.CCA ) ;
	//if(RPM_a > 1900)
		//RPM_a = 1900;
	
	
}

ISR( TCC1_OVF_vect )                            //Drehencoder and 
{
	
	//RPM_a = 0;
	AddTotAvg(RPM_a_raw,&IndexNextValue_a,0);
	
	
}

ISR( TCD1_CCA_vect )                            // Voltage controller a
{
	
	
	AddTotAvg(RPM_b_raw,&IndexNextValue_b,( 15000000UL / (uint32_t)TCD1.CCA ));		

}

ISR( TCD1_OVF_vect )                            //Drehencoder and
{
	
	//RPM_b = 0;
	AddTotAvg(RPM_b_raw,&IndexNextValue_b,0);
	
	
}

ISR( TCE0_OVF_vect )
{
	static uint8_t ct0 = 0xFF, ct1 = 0xFF, rpt;
	uint8_t i;
	static uint8_t counter = 0;
	counter ++;
	if(counter >= 10)
	{
		
		i = key_state ^ ~KEY_PIN;                       // key changed ?
		ct0 = ~( ct0 & i );                             // reset or count ct0
		ct1 = ct0 ^ (ct1 & i);                          // reset or count ct1
		i &= ct0 & ct1;                                 // count until roll over ?
		key_state ^= i;                                 // then toggle debounced state
		key_press |= key_state & i;                     // 0->1: key press detect
		
		if( (key_state & REPEAT_MASK) == 0 )            // check repeat function
		rpt = REPEAT_START;                          // start delay
		if( --rpt == 0 ){
			rpt = REPEAT_NEXT;                            // repeat delay
			key_rpt |= key_state & REPEAT_MASK;
		}
		
		counter %= 10;
	}
	int8_t _new, diff;
	_new = 0;
	if( PHASE_A ) _new = 3;
	if( PHASE_B ) _new ^= 1;          // convert gray to binary
	diff = last - _new;               // difference last - new
	if( diff & 1 ) {                 // bit 0 = value (1)
		last = _new;                    // store new as next last
		enc_delta += (diff & 2) - 1;   // bit 1 = direction (+/-)
	}
}

uint8_t get_key_press( uint8_t key_mask )
{
	cli();                                          // read and clear atomic !
	key_mask &= key_press;                          // read key(s)
	key_press ^= key_mask;                          // clear key(s)
	sei();
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed long enough such that the
// key repeat functionality kicks in. After a small setup delay
// the key is reported being pressed in subsequent calls
// to this function. This simulates the user repeatedly
// pressing and releasing the key.
//
uint8_t get_key_rpt( uint8_t key_mask )
{
	cli();                                          // read and clear atomic !
	key_mask &= key_rpt;                            // read key(s)
	key_rpt ^= key_mask;                            // clear key(s)
	sei();
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key is pressed right now
//
uint8_t get_key_state( uint8_t key_mask )

{
	key_mask &= key_state;
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_key_short( uint8_t key_mask )
{
	cli();                                          // read key state and key press atomic !
	return get_key_press( ~key_state & key_mask );
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_key_long( uint8_t key_mask )
{
	return get_key_press( get_key_rpt( key_mask ));
}

void twi_init(TWI_t * twiname){

	PORTE.PIN0CTRL = PORT_OPC_WIREDAND_gc;
	PORTE.PIN1CTRL = PORT_OPC_WIREDAND_gc;
	twiname->MASTER.CTRLB = TWI_MASTER_SMEN_bm;
	twiname->MASTER.BAUD = TWI_BAUDSETTING;
	twiname->MASTER.CTRLA = TWI_MASTER_ENABLE_bm;
	twiname->MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;


	return;
}

void AddTotAvg(uint16_t * io_pFloatAvgFilter,uint8_t * Index ,uint16_t  i_NewValue)
{
	io_pFloatAvgFilter[*Index] =	i_NewValue;
	*Index +=1;
	*Index %= 16;
}

uint16_t GetOutputValue(uint16_t * io_pFloatAvgFilter)
{
	cli();
	uint32_t TempSum = 0;
	for (uint8_t i = 0; i < 16; ++i)
	{
		TempSum += (uint32_t)io_pFloatAvgFilter[i];
	}
	uint16_t o_Result = 0;
	if(TempSum != 0)
		o_Result = (uint16_t) (TempSum / 16);
	sei();
	return o_Result;
}

void regulator()
{
	
	if(running_a)
	{
		
		//Speed controller a
		RPM_a = GetOutputValue(RPM_a_raw);
		volatile int32_t error = (int32_t)Sollwert_RPM_a - (int32_t)RPM_a;
		volatile int32_t temp_PWM = (error<<5) / (int32_t)(I_Anteil_speed / 125);
		volatile int32_t Stellgrad = (int32_t)TCD0.CCA - temp_PWM;
		if(Stellgrad < 0)
		Stellgrad = 0;
		if(Stellgrad > 0x320)
		Stellgrad = 0x320;
		//if((Stellgrad <= 0x3E8) && (Stellgrad > 0x7D))
		TCD0.CCA = Stellgrad;
		
		
		if(Stellgrad >= 0x300)
		Sollwert_a -= 0x02;
		if(Stellgrad < 0xFA)
		Sollwert_a += 0x02;
		
		if(Sollwert_a > 0x1000)
		Sollwert_a = 0x1000;
		if(Sollwert_a < 0x5A5)
		Sollwert_a = 0x5A5;
		
		error = (int32_t)Sollwert_a - (int32_t)ADC_Value_a;
		volatile int32_t temp = (error<<5) / (int32_t)(I_Anteil / 125);
		Stellgrad = (int32_t)TCC0.CCB + temp;
		if((Stellgrad <= 0x1FF) && (Stellgrad > 0x0))
		TCC0.CCB =  Stellgrad;
		else
		{
			if(temp < 0)
			TCC0.CCB = 0;
			else
			TCC0.CCB = 0x1FF;
		}
	}
	else
	{
		TCD0.CCA = 0;
		TCC0.CCB = 0;
	}
	
	if(running_b)
	{
		RPM_b = GetOutputValue(RPM_b_raw);
		//Speed controller b
		volatile int32_t error = (int32_t)Sollwert_RPM_b - (int32_t)RPM_b;
		volatile int32_t temp_PWM = (error<<5) / (int32_t)(I_Anteil_speed / 125);
		volatile int32_t Stellgrad = (int32_t)TCD0.CCB - temp_PWM;
		if(Stellgrad < 0)
		Stellgrad = 0;
		if(Stellgrad > 0x320)
		Stellgrad = 0x320;
		//if((Stellgrad <= 0x3E8) && (Stellgrad > 0x7D))
		TCD0.CCB = Stellgrad;
		
		
		if(Stellgrad >= 0x300)
		Sollwert_b -= 0x02;
		if(Stellgrad < 0xFA)
		Sollwert_b += 0x02;
		
		if(Sollwert_b > 0x1000)
		Sollwert_b = 0x1000;
		if(Sollwert_b < 0x5A5)
		Sollwert_b = 0x5A5;
		// Voltage controller b
		volatile int32_t error_b = (int32_t)Sollwert_b - (int32_t)ADC_Value_b;
		volatile int32_t temp_b = (error_b<<5) / (int32_t)(I_Anteil / 125);
		volatile int32_t Stellgrad_b = (int32_t)TCC0.CCA + temp_b;
		if((Stellgrad_b <= 0x1FF) && (Stellgrad_b > 0x0))
		TCC0.CCA =  Stellgrad_b;
		else
		{
			if(temp_b < 0)
			TCC0.CCA = 0;
			else
			TCC0.CCA = 0x1FF;
		}
	}
	else
	{
		TCD0.CCB = 0;
		TCC0.CCA = 0;
	}
	
	
}

void Blink_LED(PORT_t* port,uint8_t Pin_blink ,uint8_t Takt, uint8_t Frequenz)
{
	volatile uint8_t test = 0;
	if(port->OUT & (1<<Pin_blink))
	test  = 1;
	volatile uint8_t test2 = 0;
	if(Takt & Frequenz)
	test2 = 1;
	if(test ^ test2)
	port->OUTTGL = 1<<Pin_blink;
}

void display_menu(uint8_t *position_,char *int_buffer,p_Speed_profile_t p_profile_,uint8_t *old_position_,bool left_,bool *screen_clear_)
{

	static bool change = false;
	if(*screen_clear_)
	{
		*screen_clear_ = false;
		lcd_clrscr();
		lcd_gotoxy(1,0);
		if(left_)
			lcd_puts("Profil links ");
		else
			lcd_puts("Profil rechts");
		lcd_gotoxy(1,1);
		lcd_puts("ID :");
		utoa(p_profile_->ID,int_buffer,10);
		lcd_puts(int_buffer);
		lcd_gotoxy(1,2);
		lcd_puts("Typ: ");
		if(p_profile_->type == type_normal)
		lcd_puts("normal");
		if(p_profile_->type == type_switch)
		lcd_puts("wechselnd");
		if(p_profile_->type == type_switch)
		lcd_puts("sedimentieren");
		lcd_gotoxy(1,3);
		lcd_puts("Langsam: ");
		utoa(p_profile_->speed_slow,int_buffer,10);
		lcd_puts(int_buffer);
		lcd_puts(" UpM");
		lcd_gotoxy(1,4);
		lcd_puts("Schnell: ");
		utoa(p_profile_->speed_fast,int_buffer,10);
		lcd_puts(int_buffer);
		lcd_puts(" UpM");
		lcd_gotoxy(1,5);
		lcd_puts("t schnell ");
		display_time_menu(&p_profile_->time_fast_init,false,0,0);
		lcd_gotoxy(1,6);
		lcd_puts("t Gesamt  ");
		display_time_menu(&p_profile_->total_time,false,0,0);
		lcd_gotoxy(1,7);
		lcd_puts("t langsam ");
		display_time_menu(&p_profile_->time_slow,false,0,0);
	}
	if(menu.sub_menu >0)
	{
		switch(menu.sub_menu)
		{
			case 0x30:
			{
				p_profile_->speed_slow += encode_read2() * 10;
				lcd_gotoxy(10,3);
				if(p_Profile_a->speed_slow <10)
				lcd_puts(" ");
				if(p_profile_->speed_slow <100)
				lcd_puts(" ");
				if(p_profile_->speed_slow <1000)
				lcd_puts(" ");
				utoa(p_profile_->speed_slow,int_buffer,10);
				lcd_puts_invert(int_buffer);
				lcd_puts(" UpM");
				if( get_key_short( 1<<KEY0 ))
				{
					lcd_gotoxy(10,3);
					if(p_profile_->speed_slow <10)
					lcd_puts(" ");
					if(p_profile_->speed_slow <100)
					lcd_puts(" ");
					if(p_profile_->speed_slow <1000)
					lcd_puts(" ");
					utoa(p_profile_->speed_slow,int_buffer,10);
					lcd_puts(int_buffer);
					lcd_puts(" UpM");
					*position_ = 3;
					*old_position_ = 2;
					menu.sub_menu = 0;
				}
				
				
			}
			break;
			case 0x40:
			{
				p_profile_->speed_fast += encode_read2() * 10;
				lcd_gotoxy(10,4);
				if(p_profile_->speed_fast <10)
				lcd_puts(" ");
				if(p_profile_->speed_fast <100)
				lcd_puts(" ");
				if(p_profile_->speed_fast <1000)
				lcd_puts(" ");
				utoa(p_profile_->speed_fast,int_buffer,10);
				lcd_puts_invert(int_buffer);
				lcd_puts(" UpM");
				if( get_key_short( 1<<KEY0 ))
				{
					lcd_gotoxy(10,4);
					if(p_profile_->speed_fast <10)
					lcd_puts(" ");
					if(p_profile_->speed_fast <100)
					lcd_puts(" ");
					if(p_profile_->speed_fast <1000)
					lcd_puts(" ");
					utoa(p_profile_->speed_fast,int_buffer,10);
					lcd_puts(int_buffer);
					lcd_puts(" UpM");
					*position_ = 4;
					*old_position_ = 2;
					menu.sub_menu = 0;
				}
				
				
			}
			break;
			case 0x50:
			{
				
				update_proile_time(&p_profile_->time_fast_init,encode_read2() * 3600,max_profile_time);
				//p_Profile_a->time_fast_init += encode_read2() * 3600;
				lcd_gotoxy(11,5);
				display_time_menu(&p_profile_->time_fast_init,true,0,1);
				if( get_key_short( 1<<KEY0 ))
				{
					lcd_gotoxy(11,5);
					display_time_menu(&p_profile_->time_fast_init,false,0,0);
					
					menu.sub_menu = 0x51;
				}
				
				
			}
			break;
			case 0x51:
			{
				update_proile_time(&p_profile_->time_fast_init,encode_read2() * 60,max_profile_time);
				//p_Profile_a->time_fast_init += encode_read2() * 60;
				lcd_gotoxy(11,5);
				display_time_menu(&p_profile_->time_fast_init,true,3,4);
				if( get_key_short( 1<<KEY0 ))
				{
					lcd_gotoxy(11,5);
					display_time_menu(&p_profile_->time_fast_init,false,0,0);
					
					menu.sub_menu = 0x52;
				}
				
				
			}
			break;
			case 0x52:
			{
				update_proile_time(&p_profile_->time_fast_init,encode_read2(),max_profile_time);
				//p_Profile_a->time_fast_init += encode_read2() ;
				lcd_gotoxy(11,5);
				display_time_menu(&p_profile_->time_fast_init,true,6,7);
				if( get_key_short( 1<<KEY0 ))
				{
					lcd_gotoxy(11,5);
					display_time_menu(&p_profile_->time_fast_init,false,0,0);
					*position_ = 5;
					*old_position_ = 2;
					menu.sub_menu = 0x00;
				}
				
				
			}
			break;
			case 0x60:
			{
				update_proile_time(&p_profile_->total_time,encode_read2() * 3600,max_profile_time);
				//p_Profile_a->total_time += encode_read2() * 3600;
				lcd_gotoxy(11,6);
				display_time_menu(&p_profile_->total_time,true,0,1);
				if( get_key_short( 1<<KEY0 ))
				{
					lcd_gotoxy(11,6);
					display_time_menu(&p_profile_->total_time,false,0,0);
					
					menu.sub_menu = 0x61;
				}
				
				
			}
			break;
			case 0x61:
			{
				
				update_proile_time(&p_profile_->total_time,encode_read2() * 60,max_profile_time);
				//p_Profile_a->total_time += encode_read2() * 3600;
				lcd_gotoxy(11,6);
				display_time_menu(&p_profile_->total_time,true,3,4);
				if( get_key_short( 1<<KEY0 ))
				{
					lcd_gotoxy(11,6);
					display_time_menu(&p_profile_->total_time,false,0,0);
					
					menu.sub_menu = 0x62;
				}
				
				
			}
			break;
			case 0x62:
			{
				
				update_proile_time(&p_Profile_a->total_time,encode_read2(),max_profile_time);
				//p_Profile_a->total_time += encode_read2() * 3600;
				lcd_gotoxy(11,6);
				display_time_menu(&p_profile_->total_time,true,6,7);
				if( get_key_short( 1<<KEY0 ))
				{
					lcd_gotoxy(11,6);
					display_time_menu(&p_profile_->total_time,false,0,0);
					*position_ = 6;
					*old_position_ = 2;
					menu.sub_menu = 0x00;
				}
				
				
			}
			break;
			case 0x70:
			{
				
				update_proile_time(&p_profile_->time_slow,encode_read2() * 3600,max_profile_time);
				//p_Profile_a->total_time += encode_read2() * 3600;p_Profile_a->time_slow += encode_read2() * 3600;
				lcd_gotoxy(11,7);
				display_time_menu(&p_profile_->time_slow,true,0,1);
				if( get_key_short( 1<<KEY0 ))
				{
					lcd_gotoxy(11,7);
					display_time_menu(&p_profile_->time_slow,false,0,0);
					
					menu.sub_menu = 0x71;
				}
				
				
			}
			break;
			case 0x71:
			{
				
				update_proile_time(&p_Profile_a->time_slow,encode_read2() * 60,max_profile_time);
				//p_Profile_a->total_time += encode_read2() * 3600;p_Profile_a->time_slow += encode_read2() * 3600;
				lcd_gotoxy(11,7);
				display_time_menu(&p_profile_->time_slow,true,3,4);
				if( get_key_short( 1<<KEY0 ))
				{
					lcd_gotoxy(11,7);
					display_time_menu(&p_profile_->time_slow,false,0,0);
					
					menu.sub_menu = 0x72;
				}
				
				
			}
			break;
			case 0x72:
			{
				
				update_proile_time(&p_profile_->time_slow,encode_read2(),max_profile_time);
				//p_Profile_a->total_time += encode_read2() * 3600;p_Profile_a->time_slow += encode_read2() * 3600;
				lcd_gotoxy(11,7);
				display_time_menu(&p_profile_->time_slow,true,6,7);
				if( get_key_short( 1<<KEY0 ))
				{
					lcd_gotoxy(11,7);
					display_time_menu(&p_profile_->time_slow,false,0,0);
					*position_ = 7;
					*old_position_ = 2;
					menu.sub_menu = 0x00;
				}
				
				
			}
			break;
			default:
			break;
		}
	}
}

void Clock_Init(void)
{
	//Interner 2MHz Oszi
	OSC.CTRL = OSC_RC2MEN_bm;
	// Warten bis Oszillator stabil ist
	while ((OSC.STATUS & OSC_RC2MRDY_bm) == 0);
	// I/O Protection
	CCP = CCP_IOREG_gc;
	// Prescaler
	CLK.PSCTRL = CLK_Prescaler;
	// PLL Sorce und PLL Faktor
	OSC.PLLCTRL = OSC_PLLSRC_RC2M_gc | (PLL_Faktor << OSC_PLLFAC_gp);
	// PLL enable
	OSC.CTRL = OSC_PLLEN_bm ;
	while ((OSC.STATUS & OSC_PLLRDY_bm) == 0);
	// I/O Protection
	CCP = CCP_IOREG_gc;
	// System Clock selection
	CLK.CTRL = CLK_SCLKSEL_PLL_gc;
	// DFLL ein (Auto Kalibrierung)
	//DFLLRC2M.CTRL = DFLL_ENABLE_bm;
	
}

void RTC_Init(void)
{
	//Interner 32KHz Oszi
	OSC.XOSCCTRL = OSC_X32KLPM_bm | OSC_XOSCSEL_32KHz_gc;
	OSC.CTRL |= OSC_XOSCEN_bm;
	while(!(OSC.STATUS & (OSC_XOSCRDY_bm)));
	//0,5s Interrupt
	
	RTC.PER = 0x7F;
	//Prescale 1
	CLK.RTCCTRL = CLK_RTCSRC_TOSC_gc | CLK_RTCEN_bm;
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;CLK.RTCCTRL = CLK_RTCSRC_TOSC_gc | CLK_RTCEN_bm;
	//Synchronisieren
	while(!(RTC.STATUS & RTC_SYNCBUSY_bm));
	//High lvl Interupt
	RTC.INTCTRL = 0x03;
	
}

void Timer_init()
{
	// Speed capture a
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTD_PIN2_gc;
	PORTD.PIN2CTRL |= PORT_ISC_RISING_gc;
	TCC1.CTRLD =  TC_EVACT_FRQ_gc | TC_EVSEL_CH0_gc ;
	TCC1.CTRLB = TC1_CCAEN_bm;
	TCC1.CTRLA = TC_CLKSEL_DIV64_gc;
	
	//TCD2.CTRLE = TC2_BYTEM_SPLITMODE_gc;
	TCC1.INTCTRLA = TC_OVFINTLVL_HI_gc;
	TCC1.INTCTRLB = TC_CCAINTLVL_HI_gc;
	//TCD2.INTCTRLB = TC2_LCMPAINTLVL_OFF_gc;
	//TCC1.PER = 62500; // 125 ms timer
	
	//Speed capture b
	EVSYS.CH5MUX = EVSYS_CHMUX_PORTD_PIN3_gc;
	PORTD.PIN3CTRL |= PORT_ISC_RISING_gc;
	TCD1.CTRLD =  TC_EVACT_FRQ_gc | TC_EVSEL_CH5_gc ;
	TCD1.CTRLB = TC1_CCAEN_bm;
	TCD1.CTRLA = TC_CLKSEL_DIV64_gc;
	
	//TCD2.CTRLE = TC2_BYTEM_SPLITMODE_gc;
	TCD1.INTCTRLA = TC_OVFINTLVL_HI_gc;
	TCD1.INTCTRLB = TC_CCAINTLVL_HI_gc;
	//TCD2.INTCTRLB = TC2_LCMPAINTLVL_OFF_gc;
	//TCC1.PER = 62500; // 125 ms timer
	
	
	TCC0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm | TC0_CCBEN_bm;
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	//TCC0.INTCTRLA = TC_OVFINTLVL_MED_gc;
	//TCD2.INTCTRLB = TC2_LCMPAINTLVL_OFF_gc;
	TCC0.PER = 0x1FF;
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC1_CCBEN_bm | TC1_CCAEN_bm;
	//TCC0.INTCTRLA = TC_OVFINTLVL_MED_gc;
	//TCD2.INTCTRLB = TC2_LCMPAINTLVL_OFF_gc;
	TCD0.PER = 0x3E8;  // 1000 max = 0,1 % Aufl�sung 
	TCE0.PER = 4096 ; // 1ms Timer
	TCE0.INTCTRLA = TC_OVFINTLVL_LO_gc;
	TCE0.CTRLA = TC_CLKSEL_DIV8_gc;
	
	
}

void encode_init( void )
{
	int8_t _new;

	_new = 0;
	if( PHASE_A ) _new = 3;
	if( PHASE_B ) _new ^= 1;       // convert gray to binary
	last = _new;                   // power on state
	enc_delta = 0;
}

int8_t encode_read1( void )         // read single step encoders
{
	int8_t val;

	cli();
	val = enc_delta;
	enc_delta = 0;
	sei();
	return val;                   // counts since last call
}

int8_t encode_read2( void )         // read two step encoders
{
	int8_t val;

	cli();
	val = enc_delta;
	enc_delta = val & 1;
	sei();
	return val >> 1;
}

int LeseKalibrationsbyte(int Index)
{
	int result;
	
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(Index);
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return(result);
}

void ADCA_Cal(void)
{
	
	//ADCA.CALL = LeseKalibrationsbyte(offsetof(NVM_PROD_SIGNATURES_t,ADCACAL0));
	//ADCA.CALH = LeseKalibrationsbyte(offsetof(NVM_PROD_SIGNATURES_t,ADCACAL1));
}

void update_proile_time(uint32_t *timevalue, int16_t change,uint32_t max_value)
{
	
	*timevalue += change;
	int32_t helper = (int32_t)*timevalue;
	if(helper > (int32_t )max_value )
		*timevalue -= max_value;
	if(helper < 0 )
		*timevalue = max_value + helper;
}


void display_time(volatile uint32_t *timestamp, volatile uint32_t *end_timestamp,uint8_t x,uint8_t y,bool running,bool end_time)
{
	if(running)
	{
		char buffer[10];
		char int_buffer[3];
		volatile uint32_t timediff = *end_timestamp - *timestamp;
		volatile uint32_t helper = timediff/3600;
		if(end_time)
		{
			timediff = 0;	
			helper = 0;
		}
		ultoa(helper,int_buffer,10);
		if(helper < 10)
		{
			strcpy(buffer,"0");
			strcat(buffer,int_buffer);
		}
		else
		strcpy(buffer,int_buffer);
		strcat(buffer,":");
		timediff %=  3600;
		helper = timediff / 60;
		ultoa(helper,int_buffer,10);
		if(helper < 10)
		strcat(buffer,"0");
		strcat(buffer,int_buffer);
		strcat(buffer,":");
		timediff %= 60;
		ultoa(timediff,int_buffer,10);
		if(timediff < 10)
		strcat(buffer,"0");
		strcat(buffer,int_buffer);
		lcd_gotoxy(x,y);
		lcd_puts(buffer);
	}
	else
	{
		lcd_gotoxy(x,y);
		lcd_puts("         ");
	}
	
}

void display_time_menu(volatile uint32_t *timestamp,bool invert,uint8_t start, uint8_t ende)
{
		char buffer[10];
		char int_buffer[3];
		volatile uint32_t timediff = *timestamp;
		volatile uint32_t helper = timediff/3600;
		ultoa(helper,int_buffer,10);
		if(helper < 10)
		{
			strcpy(buffer,"0");
			strcat(buffer,int_buffer);
		}
		else
		strcpy(buffer,int_buffer);
		strcat(buffer,":");
		timediff %=  3600;
		helper = timediff / 60;
		ultoa(helper,int_buffer,10);
		if(helper < 10)
		strcat(buffer,"0");
		strcat(buffer,int_buffer);
		strcat(buffer,":");
		timediff %= 60;
		ultoa(timediff,int_buffer,10);
		if(timediff < 10)
		strcat(buffer,"0");
		strcat(buffer,int_buffer);
		if(!invert)
		lcd_puts(buffer);
		else
		lcd_puts_invert_pos(buffer,start,ende);

	
}


void ADCA_init()
{
	ADCA.CALL = 0x36;				//ADC Calibration Byte 0
	ADCA.CALH = 0x03;				//ADC Calibration Byte 1
	ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;
	ADCA.REFCTRL =  ADC_REFSEL_INT1V_gc | ADC_BANDGAP_bm;
	ADCA.PRESCALER = ADC_PRESCALER_DIV512_gc;
	ADCA.CTRLA = ADC_ENABLE_bm;
}

unsigned int ADCA_Conversion(ADC_CH_t *Channel, char Pin)
{
	Channel->CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
	Channel->MUXCTRL = (Pin << 3);
	//Channel->MUXCTRL = ADC_CH_MUXINT_BANDGAP_gc;
	for(uint8_t Waste = 0; Waste<2; Waste++)
	{
		Channel->CTRL |= ADC_CH_START_bm;
		while(!Channel->INTFLAGS);
		ADCA.INTFLAGS = ADCA.INTFLAGS ;
	}
		
	return Channel->RES;
}
uint16_t calc_voltage(uint16_t ADC_value)
{
	uint32_t helper = ADC_value ;
	helper *= 1000;
	helper-= 204300;
	helper = helper /1000;
	helper *= 15380;
	helper = helper>>12;
	
	return (uint16_t)helper;
}

int main(void)
{  
	p_Profile_a->speed_fast = 1100;
	p_Profile_a->speed_slow = 600;
	p_Profile_a->total_time = 24UL * 3600UL;
	p_Profile_a->speed_time = 5UL * 60UL;
	p_Profile_a->time_fast_init = 2UL * 3600UL;
	p_Profile_a->status = 0;
	p_Profile_a->type = type_normal;
	p_Profile_a->time_slow = 1UL * 3600UL;
	p_Profile_a->slowdown = true;
	p_Profile_a->switchoff = false;
	p_Profile_a->speedup = false;
	Sollwert_RPM_a = p_Profile_a->speed_fast;
	p_Profile_b->time_fast_init = 2UL * 3600UL;
	p_Profile_b->speed_fast = 1100;
	p_Profile_b->speed_slow = 600;
	p_Profile_b->total_time = 24UL * 3600UL;
	p_Profile_b->speed_time = 6UL * 10UL;
	p_Profile_b->status = 0;
	p_Profile_b->type = type_normal;
	p_Profile_b->time_slow = 1UL * 3600UL;
	p_Profile_b->slowdown = true;
	p_Profile_b->switchoff = false;
	p_Profile_b->speedup = false;
	Sollwert_RPM_a = p_Profile_a->speed_fast;
	Sollwert_RPM_b = p_Profile_b->speed_fast;
	uint16_t test = 0;
	uint8_t position = 0;
	bool update_display = true;
	bool high_speed_a = true;
	bool high_speed_b = false;
	uint32_t next_change_a = Unixtimestamp + p_Profile_a->time_fast_init;
	uint32_t end_time_a = Unixtimestamp + p_Profile_a->total_time;
	uint32_t next_change_b = Unixtimestamp + p_Profile_b->time_fast_init;
	uint32_t end_time_b = Unixtimestamp + p_Profile_b->total_time;
	uint8_t status_a = 0;
	uint8_t status_b = 0;
	bool change_a = false;
	bool change_b = false;
	bool change_speed_a = false;
	bool change_speed_b = false;
	uint8_t position_main = 0;
	uint8_t counter = 0;
	uint32_t avg_voltage_a = 0;
	uint32_t avg_voltage_b = 0;
	Clock_Init();
	Timer_init();
	//ADCA_Cal();
	ADCA_init();
	RTC_Init();
	twi_init(&TWIE);
	encode_init();
	PORTC.PIN2CTRL = PORT_OPC_WIREDANDPULL_gc;
	
	PORTD.DIRSET = PIN0_bm | PIN1_bm;
	PORTC.DIRSET = PIN1_bm | PIN0_bm |PIN2_bm;
	PORTA.DIRSET = PIN5_bm	| PIN6_bm | PIN7_bm;
	PORTB.DIRSET = PIN0_bm;
	PORTB.PIN1CTRL = PORT_OPC_PULLUP_gc;
	PORTB.PIN2CTRL = PORT_OPC_PULLUP_gc;
	PORTD.PIN2CTRL = PORT_OPC_PULLUP_gc;
	PORTD.PIN3CTRL = PORT_OPC_PULLUP_gc;
	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	PORTA.OUTSET = PIN6_bm;
	PORTB.OUTSET = PIN0_bm;
	sei();
	TCC0.CCA = 0xA5;
	TCC0.CCB = 0xA5;
	TCD0.CCA = 0x0;
	TCD0.CCB = 0x0;
	PORTC.OUTCLR = PIN2_bm;
	_delay_ms(1);
	PORTC.OUTSET = PIN2_bm;
	_delay_ms(1);
	lcd_init(0);
	lcd_gotoxy(left,0);
	char Str1[7];
	if(PORTB.IN & PIN2_bm)
		strcpy(Str1,"120 mm") ;
	else
		strcpy(Str1,"140 mm") ;
	lcd_puts(Str1);
	lcd_gotoxy(right,0);
	if(PORTB.IN & PIN1_bm)
	strcpy(Str1,"120 mm") ;
	else
	strcpy(Str1,"140 mm") ;
	lcd_puts(Str1);
	char text_buffer[16];
	char int_buffer[10];
    
	
    while (1) 
    {
		// Switch on/off and Reset side a
		if(!change_a)
		{
			if(status_a == 1)
			{
				running_a = true;
				hand_a = false;
			}
			if(status_a == 0)
			{
				running_a = false;
				hand_a = false;
			}
			if(status_a == 2)
			{
				running_a = false;
				hand_a = false;
			}
			if(status_a == 3)
			{
				running_a = true;
				hand_a = true;
			}
			if(status_a == 4)
				{
					next_change_a = Unixtimestamp + p_Profile_a->time_fast_init;
					end_time_a = Unixtimestamp + p_Profile_a->total_time;
					Sollwert_RPM_a = p_Profile_a->speed_fast;
					status_a = 0;
					end_a = false;
					p_Profile_a->slowdown = true;
					p_Profile_a->switchoff = false;
					
				}
		}
		
		if(!change_b)
		{
			if(status_b == 1)
			{
				running_b = true;
				hand_b = false;
			}
			if(status_b == 0)
			{
				running_b = false;
				hand_b = false;
			}
			if(status_b == 2)
			{
				running_b = false;
				hand_b = false;
			}
			if(status_b == 3)
			{
				running_b = true;
				hand_b = true;
			}
			
			if(status_b == 4)
			{
				next_change_b = Unixtimestamp + p_Profile_b->time_fast_init;
				end_time_b = Unixtimestamp + p_Profile_b->total_time;
				Sollwert_RPM_b = p_Profile_b->speed_fast;
				status_b = 0;
				end_b = false;
				p_Profile_b->slowdown = true;
				p_Profile_a->switchoff = false;
				
			}
		}
		
			
		
		if(!show_menu)
		{
			if(!change_a && !change_b && !change_speed_a && !change_speed_b)
				position_main += encode_read2();
			if(change_a)
			{
				status_a += encode_read2();
				if(status_a == 255)
					status_a = 4;
				status_a %= 5;
			}
			if(change_speed_a)
			{
				Sollwert_RPM_a += 10 * encode_read2();
			}
			if(change_speed_b)
			{
				Sollwert_RPM_b += 10 * encode_read2();
			}
			
			if(change_b)
			{	
				status_b += encode_read2();
				if(status_b == 255)
				status_b = 4;
				status_b %= 5;
			}
			
			
				
			if(get_key_long( 1<<KEY0))
			{
				show_menu = true;
				menu.sub_menu = 0;
				menu.menu_point = 0;
				screen_clear = true;
			}
		}
		
		//do regulator calulations	
		if(calc_regulator)
		{
			calc_regulator = false;
			regulator();
		}
		
		
		// read voltages of side a and b
		
		ADC_Value_a = ADCA_Conversion(&(ADCA.CH1), 1);
		ADC_Value_b = ADCA_Conversion(&(ADCA.CH0), 0);
		counter++;
		if(counter < 50)
		{	
			avg_voltage_a += ADC_Value_a;
			avg_voltage_b += ADC_Value_b;
		}
		if(counter >= 50)
		{
			avg_voltage_a = avg_voltage_a / counter;
			avg_voltage_b = avg_voltage_b / counter;
			counter = 0;
			update_display = true;
			
		}
		
		//automode changes of side a
		if(running_a  && !hand_a)
		{
			if(!end_a)
			PORTA.OUTSET = PIN6_bm;
			else
			Blink_LED(&PORTA,PIN6_bp,Blink,Frequenz_2Hz);
			if(Unixtimestamp >= next_change_a)
			{
				if(p_Profile_a->type == type_normal)
				{
					if(p_Profile_a->switchoff)
					{
						end_a = true;
						next_change_a = Unixtimestamp + p_Profile_a->speed_slow;
						
					}
					if(p_Profile_a->slowdown)
					{
						Sollwert_RPM_a = p_Profile_a->speed_slow;
						next_change_a = Unixtimestamp + (p_Profile_a->total_time - p_Profile_a->time_fast_init);
						p_Profile_a->slowdown = false;
						p_Profile_a->switchoff = true;
						
					}
					
					
				}
				
			}
		}
		else
		{
			PORTA.OUTCLR = PIN6_bm;
		}
		
		//automode changes of side b
		if(running_b  && !hand_b)
		{
			if(!end_b)
				PORTB.OUTSET = PIN0_bm;
			else
				Blink_LED(&PORTB,PIN0_bp,Blink,Frequenz_2Hz);
				
			if(Unixtimestamp >= next_change_b)
			{
				if(p_Profile_b->type == type_normal)
				{
					if(p_Profile_b->switchoff)
					{
						end_b = true;
						next_change_b = Unixtimestamp + p_Profile_b->speed_slow;
						
					}
					if(p_Profile_b->slowdown)
					{
						Sollwert_RPM_b = p_Profile_b->speed_slow;
						next_change_b = Unixtimestamp + (p_Profile_b->total_time - p_Profile_b->time_fast_init);
						p_Profile_b->slowdown = false;
						p_Profile_b->switchoff = true;
						
					}
					
					
				}
				
			}
		}
		else
		{
			PORTB.OUTCLR = PIN0_bm;
		}
		
		//Show display values
		
		//show live values when not in settings
		if(!show_menu)
		{
			if(recalc_time)
			{
				recalc_time = false;
				if(!running_a || hand_a)
				{
					next_change_a++;
					end_time_a++;
				}
				if(!running_b || hand_b)
				{
					next_change_b++;
					end_time_b++;
				}
				
				display_time(&Unixtimestamp,&next_change_a,left,5,(running_a  && !hand_a),end_a);
				display_time(&Unixtimestamp,&end_time_a,left,6,(running_a  && !hand_a),end_a);
				display_time(&Unixtimestamp,&next_change_b,right,5,(running_b  && !hand_b),end_b);
				display_time(&Unixtimestamp,&end_time_b,right,6,(running_b  && !hand_b),end_b);
			}
			
			
			if(update_display)
			{
				
				if(position_main == 255)
				position_main = 4;
				position_main %= 5;
				lcd_gotoxy(left,0);
				if(PORTB.IN & PIN2_bm)
				strcpy(Str1,"120 mm") ;
				else
				strcpy(Str1,"140 mm") ;
				lcd_puts(Str1);
				lcd_gotoxy(right,0);
				if(PORTB.IN & PIN1_bm)
				strcpy(Str1,"120 mm") ;
				else
				strcpy(Str1,"140 mm") ;
				lcd_puts(Str1);
				lcd_gotoxy(0,1);
				uint16_t pwm_a = 0x3E8 - TCD0.CCA;
				uint16_t pwm_b = 0x3E8 - TCD0.CCB;
				char int_buffer[4];
				update_display = false;
				if((avg_voltage_a/1000) <10 )
				{
					strcpy(text_buffer," ");
					utoa(calc_voltage(avg_voltage_a)/1000,int_buffer,10);
					strcat(text_buffer,int_buffer);
				}
				else
					utoa(calc_voltage(avg_voltage_a)/1000,text_buffer,10);
				char helper[] = " V";
				strcat(text_buffer,",");
				utoa(calc_voltage(avg_voltage_a)%1000,int_buffer,10);
				strcat(text_buffer,int_buffer);
				strcat(text_buffer,helper);
				lcd_puts(text_buffer);
				lcd_gotoxy(right,1);
				if((avg_voltage_b/1000) < 10 )
				{
					strcpy(text_buffer," ");
					utoa(calc_voltage(avg_voltage_b)/1000,int_buffer,10);
					strcat(text_buffer,int_buffer);
				}
				else
				utoa(calc_voltage(avg_voltage_b)/1000,text_buffer,10);
				strcat(text_buffer,",");
				utoa(calc_voltage(avg_voltage_b)%1000,int_buffer,10);
				strcat(text_buffer,int_buffer);
				strcat(text_buffer,helper);
				lcd_puts(text_buffer);
				avg_voltage_a = 0;
				avg_voltage_b = 0;
				lcd_gotoxy(left,2);
				if(pwm_a < 10)
					lcd_puts(" ");
				if(pwm_a < 100)
					lcd_puts(" ");	
				if(pwm_a < 1000)
					lcd_puts(" ");
				utoa(pwm_a/10,text_buffer,10);
				strcat(text_buffer,",");
				utoa(pwm_a%10,int_buffer,10);
				strcat(text_buffer,int_buffer);
				strcat(text_buffer," %");
				lcd_puts(text_buffer);
				lcd_gotoxy(right,2);
				if(pwm_b < 10)
				lcd_puts(" ");
				if(pwm_b < 100)
				lcd_puts(" ");
				if(pwm_b < 1000)
				lcd_puts(" ");
				utoa(pwm_b/10,text_buffer,10);
				strcat(text_buffer,",");
				utoa(pwm_b%10,int_buffer,10);
				strcat(text_buffer,int_buffer);
				strcat(text_buffer," %");
				lcd_puts(text_buffer);
				cli();
				utoa(RPM_a,text_buffer,10);
				sei();
				strcat(text_buffer," RPM");
				lcd_gotoxy(left,3);
				if(RPM_a < 10)
					lcd_puts(" ");
				if(RPM_a < 100)
					lcd_puts(" ");
				if(RPM_a < 1000)
					lcd_puts(" ");
				lcd_puts(text_buffer);
				utoa(Sollwert_RPM_a,text_buffer,10);
				lcd_gotoxy(left,4);
				if(Sollwert_RPM_a < 10)
					lcd_puts(" ");
				if(Sollwert_RPM_a < 100)
					lcd_puts(" ");
				if(Sollwert_RPM_a < 1000)
					lcd_puts(" ");
				strcat(text_buffer," RPM");
				if(change_speed_a)
					lcd_puts_invert(text_buffer);
				else
					lcd_puts(text_buffer);
				cli();
				utoa(RPM_b,text_buffer,10);
				sei();
				strcat(text_buffer," RPM");
				lcd_gotoxy(right,3);
				if(RPM_b < 10)
				lcd_puts(" ");
				if(RPM_b < 100)
				lcd_puts(" ");
				if(RPM_b < 1000)
				lcd_puts(" ");
				lcd_puts(text_buffer);
				utoa(Sollwert_RPM_b,text_buffer,10);
				strcat(text_buffer," RPM");
				lcd_gotoxy(right,4);
				if(Sollwert_RPM_b < 10)
				lcd_puts(" ");
				if(Sollwert_RPM_b < 100)
				lcd_puts(" ");
				if(Sollwert_RPM_b < 1000)
				lcd_puts(" ");
				if(change_speed_b)
					lcd_puts_invert(text_buffer);
				else
					lcd_puts(text_buffer);
				utoa(test,text_buffer,10);
				if(position_main == 0 )
				{
					get_key_short( 1<<KEY0 );
					lcd_gotoxy(left-1,4);
					lcd_puts(" ");
					lcd_gotoxy(right-1,4);
					lcd_puts(" ");
					lcd_gotoxy(left-1,7);
					lcd_puts(" ");
					lcd_gotoxy(right-1,7);
					lcd_puts(" ");
					lcd_gotoxy(left,7);
					if(status_a == 0)
						lcd_puts("Auto/Aus");
					if(status_a == 1)
					lcd_puts("Auto/An ");
					if(status_a == 2)
						lcd_puts("Hand/Aus ");
					if(status_a == 3)
						lcd_puts("Hand/An  ");
					if(status_a == 4)
						lcd_puts("Reset   ");
					lcd_gotoxy(right,7);
					if(status_b == 0)
					lcd_puts("Auto/Aus");
					if(status_b == 1)
					lcd_puts("Auto/An ");
					if(status_b == 2)
					lcd_puts("Hand/Aus ");
					if(status_b == 3)
					lcd_puts("Hand/An  ");
					if(status_b == 4)
					lcd_puts("Reset   ");
				}
				if(position_main == 1)
				{
					if( get_key_short( 1<<KEY0 ))
					{
						if(change_a)
							change_a = false;
						else
							change_a = true;
					}
					lcd_gotoxy(left-1,4);
					lcd_puts(" ");
					lcd_gotoxy(right-1,4);
					lcd_puts(" ");
					lcd_gotoxy(left-1,7);
					lcd_puts(">");
					lcd_gotoxy(right-1,7);
					lcd_puts(" ");
					lcd_gotoxy(left,7);
					if(change_a)
					{
						if(status_a == 0)
							lcd_puts_invert("Auto/Aus");
						if(status_a == 1)
							lcd_puts_invert("Auto/An ");
						if(status_a == 2)
							lcd_puts_invert("Hand/Aus");
						if(status_a == 3)
							lcd_puts_invert("Hand/An ");
						if(status_a == 4)
							lcd_puts_invert("Reset   ");
						lcd_gotoxy(right,7);
						if(status_b == 0)
						lcd_puts("Auto/Aus");
						if(status_b == 1)
						lcd_puts("Auto/An ");
						if(status_b == 2)
						lcd_puts("Hand/Aus");
						if(status_b == 3)
						lcd_puts("Hand/An ");
						if(status_b == 4)
						lcd_puts("Reset   ");
					}
					else
					{
						if(status_a == 0)
							lcd_puts("Auto/Aus");
						if(status_a == 1)
							lcd_puts("Auto/An ");
						if(status_a == 2)
							lcd_puts("Hand/Aus");
						if(status_a == 3)
							lcd_puts("Hand/An ");
						if(status_a == 4)
							lcd_puts("Reset   ");
						lcd_gotoxy(right,7);
						if(status_b == 0)
						lcd_puts("Auto/Aus");
						if(status_b == 1)
						lcd_puts("Auto/An ");
						if(status_b == 2)
						lcd_puts("Hand/Aus");
						if(status_b == 3)
						lcd_puts("Hand/An ");
						if(status_b == 4)
						lcd_puts("Reset   ");
					}
					
					
				}
				if(position_main == 2)
				{
					if( get_key_short( 1<<KEY0 ))
					{
						if(change_b)
						change_b = false;
						else
						change_b = true;
					}
					lcd_gotoxy(left-1,4);
					lcd_puts(" ");
					lcd_gotoxy(right-1,4);
					lcd_puts(" ");
					lcd_gotoxy(left-1,7);
					lcd_puts(" ");
					lcd_gotoxy(right-1,7);
					lcd_puts(">");
					lcd_gotoxy(left,7);
					if(change_b)
					{
						if(status_a == 0)
						lcd_puts("Auto/Aus");
						if(status_a == 1)
						lcd_puts("Auto/An ");
						if(status_a == 2)
						lcd_puts("Hand/Aus");
						if(status_a == 3)
						lcd_puts("Hand/An ");
						if(status_a == 4)
						lcd_puts("Reset   ");
						lcd_gotoxy(right,7);
						if(status_b == 0)
						lcd_puts_invert("Auto/Aus");
						if(status_b == 1)
						lcd_puts_invert("Auto/An ");
						if(status_b == 2)
						lcd_puts_invert("Hand/Aus");
						if(status_b == 3)
						lcd_puts_invert("Hand/An ");
						if(status_b == 4)
						lcd_puts_invert("Reset   ");
						
					}
					else
					{
						if(status_a == 0)
						lcd_puts("Auto/Aus");
						if(status_a == 1)
						lcd_puts("Auto/An ");
						if(status_a == 2)
						lcd_puts("Hand/Aus");
						if(status_a == 3)
						lcd_puts("Hand/An ");
						if(status_a == 4)
						lcd_puts("Reset   ");
						lcd_gotoxy(right,7);
						if(status_b == 0)
						lcd_puts("Auto/Aus");
						if(status_b == 1)
						lcd_puts("Auto/An ");
						if(status_b == 2)
						lcd_puts("Hand/Aus");
						if(status_b == 3)
						lcd_puts("Hand/An ");
						if(status_b == 4)
						lcd_puts("Reset   ");
					}
					
				}
				if(position_main == 3)
				{
					if( get_key_short( 1<<KEY0 ))
					{
						if(change_speed_a)
							change_speed_a = false;
						else
							change_speed_a = true;
					}
					lcd_gotoxy(left-1,4);
					lcd_puts(">");
					lcd_gotoxy(right-1,4);
					lcd_puts(" ");
					lcd_gotoxy(left-1,7);
					lcd_puts(" ");
					lcd_gotoxy(right-1,7);
					lcd_puts(" ");
					
				}
				if(position_main == 4)
				{
					if( get_key_short( 1<<KEY0 ))
					{
						if(change_speed_b)
						change_speed_b = false;
						else
						change_speed_b = true;
					}
					lcd_gotoxy(left-1,4);
					lcd_puts(" ");
					lcd_gotoxy(right-1,4);
					lcd_puts(">");
					lcd_gotoxy(left-1,7);
					lcd_puts(" ");
					lcd_gotoxy(right-1,7);
					lcd_puts(" ");
					
				}
				
			}
		}
		
		//show settings
		else
		{
			if(menu.sub_menu == 0)
				position += encode_read2();
			static uint8_t old_position = 255;
			char int_buffer[20];
			switch(menu.menu_point)
				{
					case 0x00:
					{
						if(screen_clear)
						{
							screen_clear = false;
							lcd_clrscr();
						}
						lcd_gotoxy(1,0);
						lcd_puts("Profil links:");
						lcd_gotoxy(1,1);
						lcd_puts("Profil rechts:");
						lcd_gotoxy(1,2);
						lcd_puts("Profile bearbeiten");
						lcd_gotoxy(1,3);
						lcd_puts("Regelparm. 1");
						lcd_gotoxy(1,4);
						lcd_puts("Regelparm. 2");
						lcd_gotoxy(1,5);
						lcd_puts("Exit");
						
						if(position != old_position)
						{
							if(position == 255)
								position = 5;
							position = position%6;
							old_position = position;
							for(uint8_t i= 0;i<6;i++)
							{
								lcd_gotoxy(0,i);
								if(i == position)
								{
									lcd_puts(">");
								}
								else
								{
									lcd_puts(" ");
								}
							}
						}
						if( get_key_short( 1<<KEY0 ))
						{
							if(position != 5)
							{
								menu.menu_point = position | 0x10;
								screen_clear = true;
								position = 0;
								old_position = 1;
								
							}
							else
							{
								show_menu = false;
								recalc_time = true;
								update_display = true;
								lcd_clrscr();
								
							}
						}			
						
						
						
					}
					break;
					case 0x10:
					{
						static bool change = false;
						if(screen_clear)
						{
							screen_clear = false;
							lcd_clrscr();
							lcd_gotoxy(1,0);
							lcd_puts("Profil links ");
							lcd_gotoxy(1,1);
							lcd_puts("ID :");
							utoa(p_Profile_a->ID,int_buffer,10);
							lcd_puts(int_buffer);
							lcd_gotoxy(1,2);
							lcd_puts("Typ: ");
							if(p_Profile_a->type == type_normal)
							lcd_puts("normal");
							if(p_Profile_a->type == type_switch)
							lcd_puts("wechselnd");
							if(p_Profile_a->type == type_switch)
							lcd_puts("sedimentieren");
							lcd_gotoxy(1,3);
							lcd_puts("Langsam: ");
							utoa(p_Profile_a->speed_slow,int_buffer,10);
							lcd_puts(int_buffer);
							lcd_puts(" UpM");
							lcd_gotoxy(1,4);
							lcd_puts("Schnell: ");
							utoa(p_Profile_a->speed_fast,int_buffer,10);
							lcd_puts(int_buffer);
							lcd_puts(" UpM");
							lcd_gotoxy(1,5);
							lcd_puts("t schnell ");
							display_time_menu(&p_Profile_a->time_fast_init,false,0,0);
							lcd_gotoxy(1,6);
							lcd_puts("t Gesamt  ");
							display_time_menu(&p_Profile_a->total_time,false,0,0);
							lcd_gotoxy(1,7);
							lcd_puts("t langsam ");
							display_time_menu(&p_Profile_a->time_slow,false,0,0);
						}
						if(menu.sub_menu >0)
						{
							switch(menu.sub_menu)
							{
								case 0x30:
								{
									p_Profile_a->speed_slow += encode_read2() * 10;
									lcd_gotoxy(10,3);
									if(p_Profile_a->speed_slow <10)
									lcd_puts(" ");
									if(p_Profile_a->speed_slow <100)
									lcd_puts(" ");
									if(p_Profile_a->speed_slow <1000)
									lcd_puts(" ");
									utoa(p_Profile_a->speed_slow,int_buffer,10);
									lcd_puts_invert(int_buffer);
									lcd_puts(" UpM");
									if( get_key_short( 1<<KEY0 ))
									{
										lcd_gotoxy(10,3);
										if(p_Profile_a->speed_slow <10)
										lcd_puts(" ");
										if(p_Profile_a->speed_slow <100)
										lcd_puts(" ");
										if(p_Profile_a->speed_slow <1000)
										lcd_puts(" ");
										utoa(p_Profile_a->speed_slow,int_buffer,10);
										lcd_puts(int_buffer);
										lcd_puts(" UpM");
										position = 3;
										old_position = 2;
										menu.sub_menu = 0;	
									}
									
									
								}
								break;
								case 0x40:
								{
									p_Profile_a->speed_fast += encode_read2() * 10;
									lcd_gotoxy(10,4);
									if(p_Profile_a->speed_fast <10)
									lcd_puts(" ");
									if(p_Profile_a->speed_fast <100)
									lcd_puts(" ");
									if(p_Profile_a->speed_fast <1000)
									lcd_puts(" ");
									utoa(p_Profile_a->speed_fast,int_buffer,10);
									lcd_puts_invert(int_buffer);
									lcd_puts(" UpM");
									if( get_key_short( 1<<KEY0 ))
									{
										lcd_gotoxy(10,4);
										if(p_Profile_a->speed_fast <10)
										lcd_puts(" ");
										if(p_Profile_a->speed_fast <100)
										lcd_puts(" ");
										if(p_Profile_a->speed_fast <1000)
										lcd_puts(" ");
										utoa(p_Profile_a->speed_fast,int_buffer,10);
										lcd_puts(int_buffer);
										lcd_puts(" UpM");
										position = 4;
										old_position = 2;
										menu.sub_menu = 0;
									}
									
									
								}
								break;
								case 0x50:
								{
									
									update_proile_time(&p_Profile_a->time_fast_init,encode_read2() * 3600,max_profile_time);
									//p_Profile_a->time_fast_init += encode_read2() * 3600;
									lcd_gotoxy(11,5);
									display_time_menu(&p_Profile_a->time_fast_init,true,0,1);
									if( get_key_short( 1<<KEY0 ))
									{
										lcd_gotoxy(11,5);
										display_time_menu(&p_Profile_a->time_fast_init,false,0,0);
										
										menu.sub_menu = 0x51;
									}
									
									
								}
								break;
								case 0x51:
								{
									update_proile_time(&p_Profile_a->time_fast_init,encode_read2() * 60,max_profile_time);
									//p_Profile_a->time_fast_init += encode_read2() * 60;
									lcd_gotoxy(11,5);
									display_time_menu(&p_Profile_a->time_fast_init,true,3,4);
									if( get_key_short( 1<<KEY0 ))
									{
										lcd_gotoxy(11,5);
										display_time_menu(&p_Profile_a->time_fast_init,false,0,0);
										
										menu.sub_menu = 0x52;
									}
									
									
								}
								break;
								case 0x52:
								{
									update_proile_time(&p_Profile_a->time_fast_init,encode_read2(),max_profile_time);
									//p_Profile_a->time_fast_init += encode_read2() ;
									lcd_gotoxy(11,5);
									display_time_menu(&p_Profile_a->time_fast_init,true,6,7);
									if( get_key_short( 1<<KEY0 ))
									{
										lcd_gotoxy(11,5);
										display_time_menu(&p_Profile_a->time_fast_init,false,0,0);
										position = 5;
										old_position = 2;
										menu.sub_menu = 0x00;
									}
									
									
								}
								break;
								case 0x60:
								{
									update_proile_time(&p_Profile_a->total_time,encode_read2() * 3600,max_profile_time);
									//p_Profile_a->total_time += encode_read2() * 3600;
									lcd_gotoxy(11,6);
									display_time_menu(&p_Profile_a->total_time,true,0,1);
									if( get_key_short( 1<<KEY0 ))
									{
										lcd_gotoxy(11,6);
										display_time_menu(&p_Profile_a->total_time,false,0,0);
										
										menu.sub_menu = 0x61;
									}
									
									
								}
								break;
								case 0x61:
								{
									
									update_proile_time(&p_Profile_a->total_time,encode_read2() * 60,max_profile_time);
									//p_Profile_a->total_time += encode_read2() * 3600;
									lcd_gotoxy(11,6);
									display_time_menu(&p_Profile_a->total_time,true,3,4);
									if( get_key_short( 1<<KEY0 ))
									{
										lcd_gotoxy(11,6);
										display_time_menu(&p_Profile_a->total_time,false,0,0);
										
										menu.sub_menu = 0x62;
									}
									
									
								}
								break;
								case 0x62:
								{
									
									update_proile_time(&p_Profile_a->total_time,encode_read2(),max_profile_time);
									//p_Profile_a->total_time += encode_read2() * 3600;
									lcd_gotoxy(11,6);
									display_time_menu(&p_Profile_a->total_time,true,6,7);
									if( get_key_short( 1<<KEY0 ))
									{
										lcd_gotoxy(11,6);
										display_time_menu(&p_Profile_a->total_time,false,0,0);
										position = 6;
										old_position = 2;
										menu.sub_menu = 0x00;
									}
									
									
								}
								break;
								case 0x70:
								{
									
									update_proile_time(&p_Profile_a->time_slow,encode_read2() * 3600,max_profile_time);
									//p_Profile_a->total_time += encode_read2() * 3600;p_Profile_a->time_slow += encode_read2() * 3600;
									lcd_gotoxy(11,7);
									display_time_menu(&p_Profile_a->time_slow,true,0,1);
									if( get_key_short( 1<<KEY0 ))
									{
										lcd_gotoxy(11,7);
										display_time_menu(&p_Profile_a->time_slow,false,0,0);
										
										menu.sub_menu = 0x71;
									}
									
									
								}
								break;
								case 0x71:
								{
									
									update_proile_time(&p_Profile_a->time_slow,encode_read2() * 60,max_profile_time);
									//p_Profile_a->total_time += encode_read2() * 3600;p_Profile_a->time_slow += encode_read2() * 3600;
									lcd_gotoxy(11,7);
									display_time_menu(&p_Profile_a->time_slow,true,3,4);
									if( get_key_short( 1<<KEY0 ))
									{
										lcd_gotoxy(11,7);
										display_time_menu(&p_Profile_a->time_slow,false,0,0);
										
										menu.sub_menu = 0x72;
									}
									
									
								}
								break;
								case 0x72:
								{
									
									update_proile_time(&p_Profile_a->time_slow,encode_read2(),max_profile_time);
									//p_Profile_a->total_time += encode_read2() * 3600;p_Profile_a->time_slow += encode_read2() * 3600;
									lcd_gotoxy(11,7);
									display_time_menu(&p_Profile_a->time_slow,true,6,7);
									if( get_key_short( 1<<KEY0 ))
									{
										lcd_gotoxy(11,7);
										display_time_menu(&p_Profile_a->time_slow,false,0,0);
										position = 7;
										old_position = 2;
										menu.sub_menu = 0x00;
									}
									
									
								}
								break;
								default:
								break;
							}
						}
						if(position != old_position)
						{
							if(position == 255)
							position = 8;
							position = position%8;
							old_position = position;
							for(uint8_t i= 0;i<8;i++)
							{
								lcd_gotoxy(0,i);
								if(i == position)
								{
									lcd_puts(">");
								}
								else
								{
									lcd_puts(" ");
								}
							}
						}
						
						if(get_key_short( 1<<KEY0 ))
						{
							if(position == 0)
							{
								menu.menu_point = 0;
								screen_clear = true;
							}
							else
							{
								menu.sub_menu = position <<4;
							}
						}
						
						
					}
					break;
					case 0x11:
					{
						display_menu(&position,int_buffer,p_Profile_b,&old_position,false,&screen_clear);
						if(position != old_position)
						{
							if(position == 255)
							position = 8;
							position = position%8;
							old_position = position;
							for(uint8_t i= 0;i<8;i++)
							{
								lcd_gotoxy(0,i);
								if(i == position)
								{
									lcd_puts(">");
								}
								else
								{
									lcd_puts(" ");
								}
							}
						}
						
						if(get_key_short( 1<<KEY0 ))
						{
							if(position == 0)
							{
								menu.menu_point = 0;
								screen_clear = true;
							}
							else
							{
								menu.sub_menu = position <<4;
							}
						}
						
						
					}
					break;
					default:
					menu.menu_point = 0;
					break;
				}
			
			
			
			
		}
		
		
		
		
		
		
			
			
			
			
		
		
		
		
		
		
    }
}

