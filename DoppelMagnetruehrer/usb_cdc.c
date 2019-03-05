// CDC for xmega-prozessor

#include <util/atomic.h>
#include <avr/io.h>
#include "usb_defaults.h"
#include "usb_xm.h"
#include "usb_cdc.h"


volatile unsigned char usb_stored_control=0;
volatile unsigned char usb_notify_state=EP_NOTE_IDLE;

EP_buffer cdc_rxb;
EP_buffer cdc_txb;

// define serial defaults
CDC_LineEncoding_t LineEncoding = 
{ 
	.BaudRateBPS = 9600,
	.CharFormat  = CDC_LINEENCODING_OneStopBit,
	.ParityType  = CDC_PARITY_None,
	.DataBits    = 8
};


// forward-declaration for handler
unsigned int usb_handle_ring_rx(void *pt, unsigned int);
unsigned int usb_handle_ring_tx(void *pt, unsigned int);
unsigned int usb_handle_ring_note(void *pt, unsigned int);

// define endpoint-buffers
EP_DEF(ep_in,  CDC_TX_EPADDR | USB_EP_PP , USB_EP_TYPE_BULK_gc, CDC_TXRX_EPSIZE,usb_handle_ring_tx);
EP_DEF(ep_out,  CDC_RX_EPADDR | USB_EP_PP , USB_EP_TYPE_BULK_gc, CDC_TXRX_EPSIZE,usb_handle_ring_rx);
EP_DEF(ep_note,  CDC_NOTIFICATION_EPADDR | USB_EP_PP , USB_EP_TYPE_BULK_gc, CDC_TXRX_EPSIZE,usb_handle_ring_note);

void usb_init(void) // initialize the usb_cdc
{
	cdc_rxb.flag=0;
	cdc_rxb.len=0;
	cdc_rxb.bytes=0;

	cdc_txb.flag=0;
	cdc_txb.len=0;
	cdc_txb.bytes=0;

	EP_DEF_init_buffer(ep_in);
	EP_DEF_init_buffer(ep_out);
	EP_DEF_init_buffer(ep_note);

	// Enable USB interrupts
//	USB.INTCTRLA = /*USB_SOFIE_bm |*/ USB_BUSEVIE_bm | USB_INTLVL_MED_gc;
	USB.INTCTRLA = /*USB_SOFIE_bm |*/ USB_BUSEVIE_bm | USB_INTLVL_LO_gc;
	USB.INTCTRLB = USB_TRNIE_bm | USB_SETUPIE_bm;

	USB_xm_Init();

#ifdef USB_DEF_CONFIG_IRQ
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei(); 
#endif

}

extern unsigned long menu_rcv;					// counter for received/computed bytes

// Callbacks for endpoints 
// incomming data from usb. this functions have to handle the databuffer pointet with the void-ptr
// the data-len is given in le
unsigned int usb_handle_ring_rx(void *pt, unsigned int le)
{
//	unsigned char *a=pt;
	if(	cdc_rxb.flag)
	{
		memcpy(cdc_rxb.data,(unsigned char *)pt,le);
		cdc_rxb.len=le;
		cdc_rxb.bytes=0;
		cdc_rxb.flag=0;
		return 1;
	}
	return 0;
}

// outgoing data to usb (device to host). this functions have to handle the databuffer pointet with the void-ptr
// the data-max-len is given in le; it have to returnd the len of data in buffer
// if nothing to send, return 0
unsigned int usb_handle_ring_tx(void *pt, unsigned int le)
{
	unsigned int num=0;
	if(	cdc_txb.flag)
	{
		memcpy((unsigned char *)pt,cdc_txb.data,cdc_txb.len);
		num=cdc_txb.len;
		cdc_txb.bytes=0;
		cdc_txb.flag=0;
	}
	return num;
}

unsigned int usb_handle_ring_note(void *pt, unsigned int le)
{
	USB_Request_Header_t *r;

	if(usb_notify_state)
	{
		r=(USB_Request_Header_t *)pt;
		if(usb_notify_state<EP_NOTE_SEND)
		{
			r->bmRequestType=(REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE);
			r->bRequest=CDC_NOTIF_SerialState;
			r->wValue=0;
			r->wIndex=0;
			r->wLength=2;
			usb_notify_state|=EP_NOTE_SEND;
			return sizeof(USB_Request_Header_t);
		}
		if(usb_notify_state==(EP_NOTE_SET_DCD|EP_NOTE_SEND))
		{
			*(unsigned int *)pt=(CDC_CONTROL_LINE_IN_DCD  | CDC_CONTROL_LINE_IN_DSR);
		}
		else
		{
			*(unsigned int *)pt=0;
		}
		usb_notify_state=EP_NOTE_IDLE;
		return 2;
	}

	return 0;
}

// Events 
// if device set interface
bool EVENT_USB_Device_SetInterface(uint8_t interface, uint8_t altsetting)
{
	return false;
}

// if Configuration is changed this is set
void EVENT_USB_Device_ConfigurationChanged(uint8_t config)
{
	EP_DEF_init(ep_in);
	EP_DEF_init(ep_out);
	EP_DEF_init(ep_note);
}

// if device suspend
void EVENT_USB_Device_Suspend(void)
{
}

// if device resume
void EVENT_USB_Device_WakeUp(void)
{
}

// if device reset
void EVENT_USB_Device_Reset(void)
{
}

void EVENT_USB_Device_ControlOUT(uint8_t* data, uint8_t len)
{
	if(len)
	{
		switch(usb_stored_control)
		{
			case CDC_REQ_SetLineEncoding:
				/* Read the line coding data in from the host into the global struct */
				memcpy(&LineEncoding,data,sizeof(CDC_LineEncoding_t));
				break; 
		}
	}
}

// evnt for Control Request
// Event handler for the library USB Control Request reception event. 
bool EVENT_USB_Device_ControlRequest(USB_Request_Header_t* req)
{
	USB_EP_pair_t* pair = &endpoints[0];
	USB_EP_t* e = &pair->ep[1];
	USB_EP_t* b = &pair->ep[1];

	/* Process CDC specific control requests */
	switch (req->bRequest)
	{
		case CDC_REQ_GetLineEncoding:
			if (req->bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{

				/* Write the line coding data to the control endpoint */
				memcpy(ep0_buf_in,(char *)&LineEncoding,sizeof(CDC_LineEncoding_t));
				// send ep 0
				b->DATAPTR = (unsigned) ep0_buf_in;
				b->CNT = sizeof(CDC_LineEncoding_t);
				LACR16(&(e->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
				return true;
			}
			break;

		case CDC_REQ_SetLineEncoding:
			if (req->bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				usb_stored_control=CDC_REQ_SetLineEncoding;
				/* Read the line coding data in from the host into the global struct */
				// send ep 0
				b->DATAPTR = (unsigned) ep0_buf_in;
				b->CNT = 0;
				LACR16(&(e->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
				return true;
			}

			break;
		case CDC_REQ_SetControlLineState:
			if (req->bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				// send ep 0
				b->DATAPTR = (unsigned) ep0_buf_in;
				b->CNT = 0;
				LACR16(&(e->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);

				return true;
			}
			break;

		case CDC_REQ_SendBreak:
			if (req->bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
//				EVENT_CDC_Device_BreakSent(CDCInterfaceInfo, (uint8_t)USB_ControlRequest.wValue);
				// send ep 0
				b->DATAPTR = (unsigned) ep0_buf_in;
				b->CNT = 0;
				LACR16(&(e->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
				return true;
			}

			break;
	}

//	if ((req->bmRequestType & CONTROL_REQTYPE_TYPE) == REQTYPE_VENDOR)
//	{
//		if (req->bRequest == 0xBB)
//		{
//			USB_enter_bootloader();
//		}
//	}
	return false;
}


