#include <util/atomic.h>
#include <avr/io.h>
#include "usb_ep.h"

uint8_t ep0_buf_in[USB_DEF_EP0_SIZE];
uint8_t ep0_buf_out[USB_DEF_EP0_SIZE];
USB_EP_pair_t endpoints[USB_DEF_EP_MAX] GCC_FORCE_ALIGN_2;

// endpoint-functions
void ep_def_init_buffer(EP_data *p)
{
	if(p->ep&USB_EP_PP)
	{
		endpoints[p->ep&0x0f].in.DATAPTR=(unsigned int)p->buf;
		endpoints[p->ep&0x0f].out.DATAPTR=((unsigned int)p->buf)+p->len;
	}
	else
	{
		if(p->ep&ENDPOINT_DIR_MASK)
		{
			endpoints[p->ep&0x0f].in.DATAPTR=(unsigned int)p->buf;
		}
		else
		{
			endpoints[p->ep&0x0f].out.DATAPTR=(unsigned int)p->buf;
		}
	}
}

void ep_def_init(EP_data *p)
{
	USB_EP_t *c,*b;

	ep_def_init_buffer(p);
	p->bank=0;

	c=&endpoints[p->ep&0x0f].in;
	b=&endpoints[p->ep&0x0f].out;

	if(p->ep&USB_EP_PP)
	{
		if(p->ep&ENDPOINT_DIR_MASK)
		{
			c->CTRL=0;
			c->CNT=0;
			c->STATUS=USB_EP_BUSNACK0_bm | USB_EP_BUSNACK1_bm;
			c->CTRL=p->type| USB_EP_size_to_gc(p->len) | USB_EP_PINGPONG_bm;
			b->CNT=0;
			b->CTRL=0;
			b->STATUS=USB_EP_BUSNACK0_bm;
		}
		else
		{
			b->CTRL=0;
			b->CNT=0;
			b->STATUS=0;
			b->CTRL=p->type| USB_EP_size_to_gc(p->len) | USB_EP_PINGPONG_bm;
			c->CNT=0;
			c->CTRL=0;
			c->STATUS=0;
		}
	}
	else
	{
		if(p->ep&ENDPOINT_DIR_MASK)
		{
			c->CTRL=0;
			c->CNT=0;
			c->STATUS=USB_EP_BUSNACK0_bm | USB_EP_BUSNACK1_bm;
			c->CTRL=p->type| USB_EP_size_to_gc(p->len);
		}
		else
		{
			b->CTRL=0;
			b->CNT=0;
			b->STATUS=0;
			b->CTRL=p->type| USB_EP_size_to_gc(p->len);
		}
	}
}

void ep_def_in(EP_data *p)
{
	unsigned char ad;
	USB_EP_t *c;
	USB_EP_t *b;

	ad=p->ep&0x0f;
	c=&endpoints[ad].out;
	if(p->ep&USB_EP_PP)
	{
		if(!p->bank)
		{
			// incomming data (out-endpoint)
			// bank ist set ; the next action is made with bank 1
			// if out-endpoint the data from the last aktion (receiving data) are on bank 0
			b=&endpoints[ad].out;
			if(c->STATUS&(USB_EP_TRNCOMPL0_bm))
			{
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					if(p->handler((unsigned char *)b->DATAPTR,b->CNT))
					{
						b->CNT=p->len-1;
						LACR16(&(c->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm | USB_EP_OVF_bm);
						p->bank=1;
					}
				}
			}
		}
		else
		{
			// incomming data (out-endpoint)
			// bank ist not set ; the next action is made with bank 0
			// if out-endpoint the data from the last aktion (receiving data) are on bank 1
			b=&endpoints[ad].in;
			if(c->STATUS&(USB_EP_TRNCOMPL1_bm))
			{
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					if(p->handler((unsigned char *)b->DATAPTR,b->CNT))
					{
						b->CNT=p->len-1;
						LACR16(&(c->STATUS), USB_EP_BUSNACK1_bm | USB_EP_TRNCOMPL1_bm | USB_EP_OVF_bm);
						p->bank=0;
					}
				}
			}
		}
	}
	else
	{
		// incomming data (out-endpoint)
		b=c;
		if(c->STATUS&(USB_EP_TRNCOMPL0_bm))
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				if(p->handler((unsigned char *)b->DATAPTR,b->CNT))
				{
					b->CNT=p->len;
					LACR16(&(c->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm | USB_EP_OVF_bm);
					p->bank=0;
				}
			}
		}
	}
}

void ep_def_out(EP_data *p)
{
	unsigned int le;
	unsigned char ad;
	USB_EP_t *c;
	USB_EP_t *b;

	ad=p->ep&0x0f;
	c=&endpoints[ad].in;

	if(p->ep&USB_EP_PP)
	{
		if(p->bank)
		{
			// outgoing data (in-endpoint)
			// bank ist set ; the next action is made with bank 1
			// if in-endpoint the data for the next aktion (sending data) are on bank 1
			b=&endpoints[ad].out;
			if(c->STATUS&(USB_EP_BUSNACK1_bm))
			{
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					le=p->handler((unsigned char *)b->DATAPTR,(p->len)-1);
					if(le)
					{
						b->CNT=le;
						LACR16(&(c->STATUS), USB_EP_BUSNACK1_bm | USB_EP_TRNCOMPL1_bm);
						p->bank=0;
					}
				}
			}
		}
		else
		{
			// incomming data (out-endpoint)
			// bank ist not set ; the next action is made with bank 0
			// if out-endpoint the data from the last aktion (receiving data) are on bank 1
			b=&endpoints[ad].in;
			if(c->STATUS&(USB_EP_BUSNACK0_bm))
			{
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					le=p->handler((unsigned char *)b->DATAPTR,(p->len)-1);
					if(le)
					{
						b->CNT=le;
						LACR16(&(c->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
						p->bank=1;
					}
				}
			}
		}
	}
	else
	{
		// outgoing data (in-endpoint)
		b=c;
		if(c->STATUS&(USB_EP_BUSNACK0_bm))
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				le=p->handler((unsigned char *)b->DATAPTR,p->len);
				if(le)
				{
					b->CNT=le;
					LACR16(&(c->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
					p->bank=0;
				}
			}
		}
	}
}
