#ifndef _USB_EP_H
#define _USB_EP_H

#include "usb_defaults.h"


// Callback for incomming data from usb (host to device). 
// this functions have to handle the databuffer pointet with the void-ptr
// the data-len is given in le;
// return-value not used (return 0)

// callback for outgoing data to usb (device to host). 
// this functions have to handle the databuffer pointet with the void-ptr
// the data-max-len is given in le; it have to returnd the len of data in buffer
// if nothing to send, return 0

typedef unsigned int (*USBDataCallback)(void *, unsigned int);			// pointer to Callback-Function

typedef struct ep_data 
{ 
	unsigned char 			ep;
	unsigned char 			type;
	unsigned char 			bank;
	unsigned char 			*buf;
	unsigned int 			len;

	USBDataCallback 		handler;

} EP_data; 

#define USB_EP_size_to_gc(x)  ((x <= 8   )?USB_EP_BUFSIZE_8_gc:\
                               (x <= 16  )?USB_EP_BUFSIZE_16_gc:\
                               (x <= 32  )?USB_EP_BUFSIZE_32_gc:\
                               (x <= 64  )?USB_EP_BUFSIZE_64_gc:\
                               (x <= 128 )?USB_EP_BUFSIZE_128_gc:\
                               (x <= 256 )?USB_EP_BUFSIZE_256_gc:\
                               (x <= 512 )?USB_EP_BUFSIZE_512_gc:\
                                           USB_EP_BUFSIZE_1023_gc)

#define USB_EP_IN 0x80

// Flag in the endpoint address to indicate that the endpoint should use
// PingPong (double buffer) mode. This is not actually part of the endpoint
// address as seen by the host. If PP is enabled, this flag needs to be part
// of the address passed to all USB_EP_* functions.
#define USB_EP_PP 0x40

extern USB_Request_Header_t USB_ControlRequest;

typedef union USB_EP_pair
{
	union
	{
		struct
		{
			USB_EP_t out;
			USB_EP_t in;
		};
		USB_EP_t ep[2];
	};
} ATTR_PACKED USB_EP_pair_t;

#define EP_DEF(NAME, EPNO, TYPE, PACKET_SIZE,HANDLER) \
	extern unsigned char NAME##_buf[((EPNO)&USB_EP_PP)?PACKET_SIZE*2:PACKET_SIZE ]; \
	extern EP_data NAME##_data; \
	unsigned char NAME##_buf[((EPNO)&USB_EP_PP)?PACKET_SIZE*2:PACKET_SIZE ]; \
	EP_data NAME##_data = { \
		.ep = (EPNO), \
		.type = (TYPE), \
		.bank = 0, \
		.buf = NAME##_buf, \
		.len = PACKET_SIZE, \
		.handler = HANDLER, \
	};

#define EP_DEF_init(NAME) ep_def_init(&(NAME##_data))
#define EP_DEF_init_buffer(NAME) ep_def_init_buffer(&(NAME##_data))
#define EP_DEF_in(NAME) ep_def_in(&(NAME##_data))
#define EP_DEF_out(NAME) ep_def_out(&(NAME##_data))


#define EP_NOTE_IDLE				0
#define EP_NOTE_SET_DCD				1
#define EP_NOTE_CLR_DCD				2
#define EP_NOTE_SEND				0x80

extern uint8_t ep0_buf_in[USB_DEF_EP0_SIZE];
extern uint8_t ep0_buf_out[USB_DEF_EP0_SIZE];
extern USB_EP_pair_t endpoints[USB_DEF_EP_MAX];

///////////////////////////////////////////////////////////////////////////////////////
// endpoint-prototypes
///////////////////////////////////////////////////////////////////////////////////////
void ep_def_init_buffer(EP_data *p);
void ep_def_init(EP_data *p);
void ep_def_in(EP_data *p);
void ep_def_out(EP_data *p);

#endif
