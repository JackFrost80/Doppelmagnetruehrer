#ifndef _USB_CDC_H
#define _USB_CDC_H

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#include "usb_defaults.h"
#include "Descriptors.h"
#include "usb_ep.h"

typedef struct ep_buffer 
{ 
	unsigned char 			data[CDC_TXRX_EPSIZE];
	unsigned char 			bytes;
	unsigned char 			len;
	unsigned char 			flag;
} EP_buffer; 

extern EP_buffer cdc_rxb;
extern EP_buffer cdc_txb;

extern EP_data ep_in_data;
extern EP_data ep_out_data;
extern EP_data ep_note_data;

/* Function Prototypes: */
void usb_init(void);

#endif