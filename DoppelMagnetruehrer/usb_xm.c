#include <util/atomic.h>
#include <avr/io.h>
#include "usb_xm.h"
#include "main.h"


ISR(USB_BUSEVENT_vect)  // global IRQ
{
	if (USB.INTFLAGSACLR & USB_SOFIF_bm)
	{
		USB.INTFLAGSACLR = USB_SOFIF_bm;
	}
	else if (USB.INTFLAGSACLR & (USB_CRCIF_bm | USB_UNFIF_bm | USB_OVFIF_bm))
	{
		USB.INTFLAGSACLR = (USB_CRCIF_bm | USB_UNFIF_bm | USB_OVFIF_bm);
	}
	else if (USB.INTFLAGSACLR & USB_STALLIF_bm)
	{
		USB.INTFLAGSACLR = USB_STALLIF_bm;
	}
	else
	{
		if (USB.INTFLAGSACLR & USB_SUSPENDIF_bm)
		{
			EVENT_USB_Device_Suspend();
		}

		if (USB.INTFLAGSACLR & USB_RESUMEIF_bm)
		{
			EVENT_USB_Device_WakeUp();
		}

		if (USB.INTFLAGSACLR & USB_RSTIF_bm)
		{
			USB.ADDR=0;
			USB_DeviceState=DEVICE_STATE_Default;
			USB_ResetInterface();
			EVENT_USB_Device_Reset();
		}

		USB.INTFLAGSACLR = USB_SUSPENDIF_bm | USB_RESUMEIF_bm | USB_RSTIF_bm;
		USB_Evt_Task();
	}
}

ISR(USB_TRNCOMPL_vect)  // irq for transaktion complete
{
	USB.FIFOWP = 0;
	USB.INTFLAGSBCLR = USB_SETUPIF_bm | USB_TRNIF_bm;
	USB_Task();
}


inline bool USB_handleSetAddress(USB_Request_Header_t* req) // USB-Funktionen
{
	uint8_t    DeviceAddress = (req -> wValue & 0x7F);
	USB_EP_pair_t* pair = &endpoints[0];
	USB_EP_t* e = &pair->ep[1];
	USB_EP_t* b = &pair->ep[1];

	LACR16(&endpoints[0].out.STATUS, USB_EP_SETUP_bm | USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm | USB_EP_OVF_bm);

	b->DATAPTR = (unsigned) ep0_buf_in;
	b->CNT = 0;
	LACR16(&(e->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);

	while (!(e->STATUS & USB_EP_TRNCOMPL0_bm)){};

	USB.ADDR = DeviceAddress;
	USB_DeviceState = (DeviceAddress) ? DEVICE_STATE_Addressed : DEVICE_STATE_Default;
	return true;
}

#if !defined(NO_INTERNAL_SERIAL) && (USE_INTERNAL_SERIAL != NO_DESCRIPTOR)
inline void USB_Device_GetSerialString(uint16_t* const UnicodeString) ATTR_NON_NULL_PTR_ARG(1);
inline void USB_Device_GetSerialString(uint16_t* const UnicodeString)
{
	//uint_reg_t CurrentGlobalInt = GetGlobalInterruptMask();
	//GlobalInterruptDisable();
	
	uint8_t SigReadAddress = INTERNAL_SERIAL_START_ADDRESS;

	for (uint8_t SerialCharNum = 0; SerialCharNum < (INTERNAL_SERIAL_LENGTH_BITS / 4); SerialCharNum++)
	{					
		uint8_t SerialByte;

		NVM.CMD    = NVM_CMD_READ_CALIB_ROW_gc;
		SerialByte = pgm_read_byte(SigReadAddress);
		NVM_CMD = NVM_CMD_NO_OPERATION_gc; 

		if (SerialCharNum & 0x01)
		{
			SerialByte >>= 4;
			SigReadAddress++;
		}

		SerialByte &= 0x0F;

		UnicodeString[SerialCharNum] = ((SerialByte >= 10) ?
									   (('A' - 10) + SerialByte) : ('0' + SerialByte));
	}
	
	//SetGlobalInterruptMask(CurrentGlobalInt);
}

inline void USB_Device_GetInternalSerialDescriptor(void)
{
	USB_EP_pair_t* pair = &endpoints[0];
	USB_EP_t* e = &pair->ep[1];
	USB_EP_t* b = &pair->ep[1];

	struct
	{
		USB_Descriptor_Header_t Header;
		uint16_t                UnicodeString[INTERNAL_SERIAL_LENGTH_BITS / 4];
	}* SignatureDescriptor = (void*) ep0_buf_in;

	SignatureDescriptor->Header.Type = DTYPE_String;
	SignatureDescriptor->Header.Size = USB_STRING_LEN(INTERNAL_SERIAL_LENGTH_BITS / 4);
	
	USB_Device_GetSerialString(SignatureDescriptor->UnicodeString);

	b->DATAPTR = (unsigned) ep0_buf_in;
	b->CNT = sizeof(*SignatureDescriptor);
	LACR16(&(e->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
}
#endif

inline bool USB_handleGetDescriptor(USB_Request_Header_t* req)
{
	const void* DescriptorPointer;
	uint16_t  DescriptorSize;
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	
	#if !defined(NO_INTERNAL_SERIAL) && (USE_INTERNAL_SERIAL != NO_DESCRIPTOR)
	if (req->wValue == ((DTYPE_String << 8) | USE_INTERNAL_SERIAL))
	{
		USB_Device_GetInternalSerialDescriptor();
		return true;
	}
	#endif
	
	if ((DescriptorSize = CALLBACK_USB_GetDescriptor(req->wValue, req->wIndex, &DescriptorPointer)))
	{
		if (DescriptorSize > req->wLength) DescriptorSize=req->wLength;
		USB_ep0_send_progmem(DescriptorPointer, DescriptorSize);
		return true;
	}
	return false;
}

inline bool USB_handleSetConfiguration(USB_Request_Header_t* req)
{
	USB_Descriptor_Device_t* DevDescriptorPtr;
	USB_EP_pair_t* pair = &endpoints[0];
	USB_EP_t* e = &pair->ep[1];
	USB_EP_t* b = &pair->ep[1];

	uint8_t r = CALLBACK_USB_GetDescriptor((DTYPE_Device << 8), 0, (void*)&DevDescriptorPtr);
	if (r == NO_DESCRIPTOR) return false;
	
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	uint8_t num_configs = pgm_read_byte(&DevDescriptorPtr->NumberOfConfigurations);
	
	if ((uint8_t)req->wValue > num_configs) return false;

	b->DATAPTR = (unsigned) ep0_buf_in;
	b->CNT = 0;
	LACR16(&(e->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);

	USB_Device_ConfigurationNumber = (uint8_t)(req -> wValue);

	if (USB_Device_ConfigurationNumber)
	  USB_DeviceState = DEVICE_STATE_Configured;
	else
	  USB_DeviceState = (USB.ADDR) ? DEVICE_STATE_Configured : DEVICE_STATE_Powered;

	EVENT_USB_Device_ConfigurationChanged(USB_Device_ConfigurationNumber);
	return true;
}

bool USB_HandleSetup(void)
{
	USB_Request_Header_t* req = (void *) ep0_buf_out;
	USB_EP_pair_t* pair = &endpoints[0];
	USB_EP_t* e __attribute__ ((unused)) = &pair->ep[1];
	USB_EP_t* b = &pair->ep[1];
	
	if ((req->bmRequestType & CONTROL_REQTYPE_TYPE) == REQTYPE_STANDARD)
	{
		switch (req->bRequest)
		{
			case REQ_GetStatus:
				ep0_buf_in[0] = 0;
				ep0_buf_in[1] = 0;
				b->DATAPTR = (unsigned) ep0_buf_in;
				b->CNT = 2;
				LACR16(&(e->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
				return true;
			case REQ_ClearFeature:
			case REQ_SetFeature:
				b->DATAPTR = (unsigned) ep0_buf_in;
				b->CNT = 0;
				LACR16(&(e->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
				return true;
			case REQ_SetAddress:
				return USB_handleSetAddress(req);
			case REQ_GetDescriptor:
				return USB_handleGetDescriptor(req);
			case REQ_GetConfiguration:
				ep0_buf_in[0] = USB_Device_ConfigurationNumber;
				b->DATAPTR = (unsigned) ep0_buf_in;
				b->CNT = 1;
				LACR16(&(e->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
				return true;
			case REQ_SetConfiguration:
				return USB_handleSetConfiguration(req);
			case REQ_SetInterface:
				if (EVENT_USB_Device_SetInterface(req->wIndex, req->wValue))
				{
					b->DATAPTR = (unsigned) ep0_buf_in;
					b->CNT = 0;
					LACR16(&(e->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
					return true;
				}
			case REQ_GetInterface:
				return false;
		}
	}
	
	return EVENT_USB_Device_ControlRequest(req);
}

volatile uint8_t USB_DeviceState;
volatile uint8_t USB_Device_ConfigurationNumber;

void USB_xm_Init(void)
{
	//uint_reg_t CurrentGlobalInt = GetGlobalInterruptMask();
	//GlobalInterruptDisable();

	NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
	USB.CAL0 = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, USBCAL0));
	NVM_CMD = NVM_CMD_NO_OPERATION_gc; 

	NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
	USB.CAL1 = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, USBCAL1));
	NVM_CMD = NVM_CMD_NO_OPERATION_gc; 

	//SetGlobalInterruptMask(CurrentGlobalInt);

	USB_ResetInterface();	
}

void USB_ResetInterface(void)
{

	CLK.USBCTRL = ((((F_USB / 48000000) - 1) << CLK_USBPSDIV_gp) | CLK_USBSRC_RC32M_gc | CLK_USBSEN_bm);
	USB.EPPTR = (unsigned) &endpoints;
	USB.ADDR = 0;
	
	endpoints[0].out.STATUS = 0;
	endpoints[0].out.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_size_to_gc(USB_DEF_EP0_SIZE);
	endpoints[0].out.DATAPTR = (unsigned) &ep0_buf_out;
	endpoints[0].in.STATUS = USB_EP_BUSNACK0_bm;
	endpoints[0].in.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_size_to_gc(USB_DEF_EP0_SIZE);
	endpoints[0].in.DATAPTR = (unsigned) &ep0_buf_in;
	
	USB.CTRLA = USB_ENABLE_bm | USB_SPEED_bm | (USB_DEF_EP_MAX-1);
	
//	USB_Attach();
	USB.CTRLB |= USB_ATTACH_bm;
}

void USB_ep0_send_progmem(const uint8_t* addr, uint16_t size)
{
	uint8_t *buf = ep0_buf_in;
	uint16_t remaining = size;
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;

	USB_EP_pair_t* pair = &endpoints[0];
	USB_EP_t* e = &pair->ep[1];
	USB_EP_t* b = &pair->ep[1];

	while (remaining--)
	{
		*buf++ = pgm_read_byte(addr++);
	}

	b->DATAPTR = (unsigned) ep0_buf_in;
	b->CNT = size;
	LACR16(&(e->STATUS), USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
}


void USB_Evt_Task(void)  // USB Tasks
{
	if (USB.STATUS & USB_BUSRST_bm)
	{
		USB.STATUS &= ~USB_BUSRST_bm;
		USB_xm_Init();
	}
}


void USB_Task(void)  // is call from irq
{
	// Read once to prevent race condition where SETUP packet is interpreted as OUT
	uint8_t status = endpoints[0].out.STATUS;

	if (status & USB_EP_SETUP_bm)
	{
		if (!USB_HandleSetup())
		{
			endpoints[0].out.CTRL |= USB_EP_STALL_bm;
			endpoints[0].in.CTRL |= USB_EP_STALL_bm; 
		}
		LACR16(&endpoints[0].out.STATUS, USB_EP_SETUP_bm | USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm | USB_EP_OVF_bm);
	}
	else if(status & USB_EP_TRNCOMPL0_bm)
	{
		EVENT_USB_Device_ControlOUT((uint8_t *) ep0_buf_out, endpoints[0].out.CNT);
		LACR16(&endpoints[0].out.STATUS, USB_EP_SETUP_bm | USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm | USB_EP_OVF_bm);
	}
}

