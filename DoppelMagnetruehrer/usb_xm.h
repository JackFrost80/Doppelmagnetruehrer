#ifndef _USB_XM_H
#define _USB_XM_H
#include "usb_defaults.h"
#include "Descriptors.h"
#include "usb_ep.h"

extern volatile uint8_t USB_DeviceState;
extern volatile uint8_t USB_Device_ConfigurationNumber;

bool USB_handleSetConfiguration(USB_Request_Header_t* req);
bool USB_handleSetAddress(USB_Request_Header_t* req);
bool USB_handleGetDescriptor(USB_Request_Header_t* req);
void USB_Device_GetInternalSerialDescriptor(void);
void USB_Device_GetSerialString(uint16_t* const UnicodeString);
void USB_ep0_send_progmem(const uint8_t* addr, uint16_t size);
void USB_ResetInterface(void);
void USB_Evt_Task(void);
void USB_Task(void);
void USB_xm_Init(void);

#endif
