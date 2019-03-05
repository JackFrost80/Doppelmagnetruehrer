// usb_defaults.h

#ifndef _USB_DEFAULTS_H
#define _USB_DEFAULTS_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
	
#define ARCH_HAS_EEPROM_ADDRESS_SPACE
#define ARCH_HAS_FLASH_ADDRESS_SPACE
#define ARCH_HAS_MULTI_ADDRESS_SPACE
#define ARCH_LITTLE_ENDIAN	

#define USB_DEF_VID							0x03EB		// VID for USB
#define USB_DEF_PID							0x2404		// PID for USB

#define USB_DEF_EP_NUM						3			// Number of Endpoints used (ep0, In, out, notification... )
#define USB_DEF_EP_MAX						16			// Number of max endpoints for this device (to allocate the endpoint-table)
#define USB_DEF_EP0_SIZE					64			// size of endpoint0 to handle controll-requests

#define ENDPOINT_DIR_IN						0x80
#define ENDPOINT_DIR_MASK					0x80
#define ENDPOINT_DIR_OUT					0x00

#define USB_DEF_CONFIG_IRQ				1			// if defined, IRQlevle are configured



#define CPU_TO_LE16(x) 						x

#define VERSION_TENS(x)						(int)((x) / 10)
#define VERSION_ONES(x)						(int)((x) - (10 * VERSION_TENS(x)))
#define VERSION_TENTHS(x)					(int)(((x) - (int)(x)) * 10)
#define VERSION_HUNDREDTHS(x)				(int)((((x) - (int)(x)) * 100) - (10 * VERSION_TENTHS(x)))

#define CONTROL_REQTYPE_DIRECTION			0x80
#define CONTROL_REQTYPE_TYPE				0x60
#define CONTROL_REQTYPE_RECIPIENT			0x1F
#define REQDIR_HOSTTODEVICE					(0 << 7)
#define REQDIR_DEVICETOHOST					(1 << 7)
#define REQTYPE_STANDARD					(0 << 5)
#define REQTYPE_CLASS						(1 << 5)
#define REQTYPE_VENDOR						(2 << 5)
#define REQREC_DEVICE						(0 << 0)
#define REQREC_INTERFACE					(1 << 0)
#define REQREC_ENDPOINT						(2 << 0)
#define REQREC_OTHER						(3 << 0)

#define NO_DESCRIPTOR						0
#define USB_CONFIG_POWER_MA(mA)				((mA) >> 1)
#define USB_STRING_LEN(UnicodeChars)		(sizeof(USB_Descriptor_Header_t) + ((UnicodeChars) << 1))
#define VERSION_BCD(x)						CPU_TO_LE16((((VERSION_TENS(x) << 4) | VERSION_ONES(x)) << 8) | ((VERSION_TENTHS(x) << 4) | VERSION_HUNDREDTHS(x)))
#define LANGUAGE_ID_ENG						0x0409
#define ENDPOINT_DESCRIPTOR_DIR_IN			ENDPOINT_DIR_IN
#define ENDPOINT_DESCRIPTOR_DIR_OUT			ENDPOINT_DIR_OUT
#define USB_CONFIG_ATTR_BUSPOWERED			0x80
#define USB_CONFIG_ATTR_SELFPOWERED			0x40
#define USB_CONFIG_ATTR_REMOTEWAKEUP		0x20
#define ENDPOINT_ATTR_NO_SYNC				(0 << 2)
#define ENDPOINT_ATTR_ASYNC					(1 << 2)
#define ENDPOINT_ATTR_ADAPTIVE				(2 << 2)
#define ENDPOINT_ATTR_SYNC					(3 << 2)
#define ENDPOINT_USAGE_DATA					(0 << 4)
#define ENDPOINT_USAGE_FEEDBACK				(1 << 4)
#define ENDPOINT_USAGE_IMPLICIT_FEEDBACK	(2 << 4)
#define EP_TYPE_CONTROL						0x00
#define EP_TYPE_ISOCHRONOUS					0x01
#define EP_TYPE_BULK						0x02
#define EP_TYPE_INTERRUPT					0x03

#define FEATURE_SELFPOWERED_ENABLED			(1 << 0)
#define FEATURE_REMOTE_WAKEUP_ENABLED		(1 << 1)

#define USE_INTERNAL_SERIAL					0xDC

#define INTERNAL_SERIAL_LENGTH_BITS			(8 * (1 + (offsetof(NVM_PROD_SIGNATURES_t, COORDY1) - offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0))))
#define INTERNAL_SERIAL_START_ADDRESS		offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0)


typedef uint8_t uint_reg_t;
struct USB_Request_Header;
typedef struct USB_Request_Header USB_Requst_Header_t;



/// From Atmel: Macros for XMEGA instructions not yet supported by the toolchain
// Load and Clear 
#ifdef __GNUC__
#define LACR16(addr,msk) \
	__asm__ __volatile__ ( \
	"ldi r16, %1" "\n\t" \
	".dc.w 0x9306" "\n\t"\
	::"z" (addr), "M" (msk):"r16")
#else
	#define LACR16(addr,msk) __lac((unsigned char)msk,(unsigned char*)addr)
#endif
	 
// Load and Set
#ifdef __GNUC__
#define LASR16(addr,msk) \
	__asm__ __volatile__ ( \
	"ldi r16, %1" "\n\t" \
	".dc.w 0x9305" "\n\t"\
	::"z" (addr), "M" (msk):"r16")
#else
#define LASR16(addr,msk) __las((unsigned char)msk,(unsigned char*)addr)
#endif

// Exchange
#ifdef __GNUC__
#define XCHR16(addr,msk) \
	__asm__ __volatile__ ( \
	"ldi r16, %1" "\n\t" \
	".dc.w 0x9304" "\n\t"\
	::"z" (addr), "M" (msk):"r16")
#else
#define XCHR16(addr,msk) __xch(msk,addr)
#endif

// Load and toggle
#ifdef __GNUC__
#define LATR16(addr,msk) \
	__asm__ __volatile__ ( \
	"ldi r16, %1" "\n\t" \
	".dc.w 0x9307" "\n\t"\
	::"z" (addr), "M" (msk):"r16")
#else
#define LATR16(addr,msk) __lat(msk,addr)
#endif


#define MACROS                  do
#define MACROE                  while (0)
#if !defined(MAX)
	#define MAX(x, y)               (((x) > (y)) ? (x) : (y))
#endif

#if !defined(MIN)
	#define MIN(x, y)               (((x) < (y)) ? (x) : (y))
#endif

#if !defined(STRINGIFY)
	#define STRINGIFY(x)            #x
	#define STRINGIFY_EXPANDED(x)   STRINGIFY(x)
#endif

#if defined(__GNUC__)
	#define GCC_FORCE_POINTER_ACCESS(StructPtr)   __asm__ __volatile__("" : "=b" (StructPtr) : "0" (StructPtr))
	#define GCC_MEMORY_BARRIER()                  __asm__ __volatile__("" ::: "memory");
	#define GCC_IS_COMPILE_CONST(x)               __builtin_constant_p(x)

	/** Compile-time assert */
	#define GCC_ASSERT(e) ((void)sizeof(char[1 - 2*!(__builtin_constant_p(e) && (e))]))
	
	/** Like __attribute__(align(2)), but actually works. 
	    From http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=121033
	 */
	#define GCC_FORCE_ALIGN_2  __attribute__((section (".data,\"aw\",@progbits\n.p2align 1;")))

	#define likely(x) __builtin_expect((x),1)
	#define unlikely(x) __builtin_expect((x),0)
#else
	#define GCC_FORCE_POINTER_ACCESS(StructPtr)
	#define GCC_MEMORY_BARRIER()
	#define GCC_IS_COMPILE_CONST(x)               0
	#define GCC_FORCE_ALIGN_2
	#define likely(x) x
	#define unlikely(x) x
#endif


#define ATTR_NO_RETURN              __attribute__ ((noreturn))
#define ATTR_WARN_UNUSED_RESULT     __attribute__ ((warn_unused_result))
#define ATTR_NON_NULL_PTR_ARG(...)  __attribute__ ((nonnull (__VA_ARGS__)))
#define ATTR_NAKED                  __attribute__ ((naked))
#define ATTR_NO_INLINE              __attribute__ ((noinline))
#define ATTR_ALWAYS_INLINE          __attribute__ ((always_inline))
#define ATTR_PURE                   __attribute__ ((pure))
#define ATTR_CONST                  __attribute__ ((const))
#define ATTR_DEPRECATED             __attribute__ ((deprecated))
#define ATTR_WEAK                   __attribute__ ((weak))
#define ATTR_NO_INIT                __attribute__ ((section (".noinit")))
#define ATTR_INIT_SECTION(SectionIndex) __attribute__ ((naked, section (".init" #SectionIndex )))
#define ATTR_ALIAS(Func)               __attribute__ ((alias( #Func )))
#define ATTR_PACKED                     __attribute__ ((packed))


/**
Event handlers. These functions are called from ISRs or are otherwise time-critical,
so handle them quickly.
*/
				
/** Callback to handle a control request that was not handled by the library. Return true
 *  if the request has been handled. Returning false will send a STALL to the host.
 */
bool EVENT_USB_Device_ControlRequest(struct USB_Request_Header* req);

/** Event when OUT data is received as part of a control transfer. */
void EVENT_USB_Device_ControlOUT(uint8_t* data, uint8_t len);

/** Event when the USB configuration is changed. The configuration is stored in
    variable USB_Device_ConfigurationNumber */
void EVENT_USB_Device_ConfigurationChanged(uint8_t config);

/** Event when an alternate setting for an interface is selected. Return true
    to accept the alternate setting, or FALSE to send a STALL reply */
bool EVENT_USB_Device_SetInterface(uint8_t interface, uint8_t altsetting);

/** Event when the USB bus suspends */
void EVENT_USB_Device_Suspend(void);

/** Event when the USB bus returns from suspend */
void EVENT_USB_Device_WakeUp(void);

/** Event when the host resets the device. Called after the library resets the control endpoint */
void EVENT_USB_Device_Reset(void);

/** Event called on start of frame, if enabled */
void EVENT_USB_Device_StartOfFrame(void);


/* Enums: */
/** Enum for the possible standard descriptor types, as given in each descriptor's header. */
enum USB_DescriptorTypes_t
{
	DTYPE_Device                    = 0x01, /**< Indicates that the descriptor is a device descriptor. */
	DTYPE_Configuration             = 0x02, /**< Indicates that the descriptor is a configuration descriptor. */
	DTYPE_String                    = 0x03, /**< Indicates that the descriptor is a string descriptor. */
	DTYPE_Interface                 = 0x04, /**< Indicates that the descriptor is an interface descriptor. */
	DTYPE_Endpoint                  = 0x05, /**< Indicates that the descriptor is an endpoint descriptor. */
	DTYPE_DeviceQualifier           = 0x06, /**< Indicates that the descriptor is a device qualifier descriptor. */
	DTYPE_Other                     = 0x07, /**< Indicates that the descriptor is of other type. */
	DTYPE_InterfacePower            = 0x08, /**< Indicates that the descriptor is an interface power descriptor. */
	DTYPE_InterfaceAssociation      = 0x0B, /**< Indicates that the descriptor is an interface association descriptor. */
	DTYPE_CSInterface               = 0x24, /**< Indicates that the descriptor is a class specific interface descriptor. */
	DTYPE_CSEndpoint                = 0x25, /**< Indicates that the descriptor is a class specific endpoint descriptor. */
};

/** Enum for possible Class, Subclass and Protocol values of device and interface descriptors. */
enum USB_Descriptor_ClassSubclassProtocol_t
{
	USB_CSCP_NoDeviceClass          = 0x00, /**< Descriptor Class value indicating that the device does not belong
	                                         *   to a particular class at the device level.
	                                         */
	USB_CSCP_NoDeviceSubclass       = 0x00, /**< Descriptor Subclass value indicating that the device does not belong
	                                         *   to a particular subclass at the device level.
	                                         */
	USB_CSCP_NoDeviceProtocol       = 0x00, /**< Descriptor Protocol value indicating that the device does not belong
	                                         *   to a particular protocol at the device level.
	                                         */
	USB_CSCP_VendorSpecificClass    = 0xFF, /**< Descriptor Class value indicating that the device/interface belongs
	                                         *   to a vendor specific class.
	                                         */
	USB_CSCP_VendorSpecificSubclass = 0xFF, /**< Descriptor Subclass value indicating that the device/interface belongs
	                                         *   to a vendor specific subclass.
	                                         */
	USB_CSCP_VendorSpecificProtocol = 0xFF, /**< Descriptor Protocol value indicating that the device/interface belongs
	                                         *   to a vendor specific protocol.
	                                         */
	USB_CSCP_IADDeviceClass         = 0xEF, /**< Descriptor Class value indicating that the device belongs to the
	                                         *   Interface Association Descriptor class.
	                                         */
	USB_CSCP_IADDeviceSubclass      = 0x02, /**< Descriptor Subclass value indicating that the device belongs to the
	                                         *   Interface Association Descriptor subclass.
	                                         */
	USB_CSCP_IADDeviceProtocol      = 0x01, /**< Descriptor Protocol value indicating that the device belongs to the
	                                         *   Interface Association Descriptor protocol.
	                                         */
};

typedef struct
{
	uint8_t Size; /**< Size of the descriptor, in bytes. */
	uint8_t Type; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a value
	               *   given by the specific class.
	               */
} ATTR_PACKED USB_Descriptor_Header_t;

typedef struct
{
	uint8_t bLength; /**< Size of the descriptor, in bytes. */
	uint8_t bDescriptorType; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a value
	                          *   given by the specific class.
	                          */
} ATTR_PACKED USB_StdDescriptor_Header_t;

typedef struct
{
	USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */

	uint16_t USBSpecification; /**< BCD of the supported USB specification. */
	uint8_t  Class; /**< USB device class. */
	uint8_t  SubClass; /**< USB device subclass. */
	uint8_t  Protocol; /**< USB device protocol. */

	uint8_t  Endpoint0Size; /**< Size of the control (address 0) endpoint's bank in bytes. */

	uint16_t VendorID; /**< Vendor ID for the USB product. */
	uint16_t ProductID; /**< Unique product ID for the USB product. */
	uint16_t ReleaseNumber; /**< Product release (version) number. */

	uint8_t  ManufacturerStrIndex; /**< String index for the manufacturer's name. The
									*   host will request this string via a separate
	                                *   control request for the string descriptor.
	                                *
	                                *   \note If no string supplied, use \ref NO_DESCRIPTOR.
	                                */
	uint8_t  ProductStrIndex; /**< String index for the product name/details.
	                           *
	                           *  \see ManufacturerStrIndex structure entry.
	                           */
	uint8_t  SerialNumStrIndex; /**< String index for the product's globally unique hexadecimal
	                             *   serial number, in uppercase Unicode ASCII.
	                             *
	                             *  \note On some microcontroller models, there is an embedded serial number
	                             *        in the chip which can be used for the device serial number.
	                             *        To use this serial number, set this to USE_INTERNAL_SERIAL.
	                             *        On unsupported devices, this will evaluate to 0 and will cause
	                             *        the host to generate a pseudo-unique value for the device upon
	                             *        insertion.
	                             *
	                             *  \see ManufacturerStrIndex structure entry.
	                             */
	uint8_t  NumberOfConfigurations; /**< Total number of configurations supported by
									*   the device.
	                                  */
} ATTR_PACKED USB_Descriptor_Device_t;

typedef struct
{
	uint8_t  bLength; /**< Size of the descriptor, in bytes. */
	uint8_t  bDescriptorType; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a value
	                              *   given by the specific class.
	                              */
	uint16_t bcdUSB; /**< BCD of the supported USB specification. */
	uint8_t  bDeviceClass; /**< USB device class. */
	uint8_t  bDeviceSubClass; /**< USB device subclass. */
	uint8_t  bDeviceProtocol; /**< USB device protocol. */
	uint8_t  bMaxPacketSize0; /**< Size of the control (address 0) endpoint's bank in bytes. */
	uint16_t idVendor; /**< Vendor ID for the USB product. */
	uint16_t idProduct; /**< Unique product ID for the USB product. */
	uint16_t bcdDevice; /**< Product release (version) number. */
	uint8_t  iManufacturer; /**< String index for the manufacturer's name. The
	                         *   host will request this string via a separate
	                         *   control request for the string descriptor.
	                         *
	                         *   \note If no string supplied, use \ref NO_DESCRIPTOR.
	                         */
	uint8_t  iProduct; /**< String index for the product name/details.
	                    *
	                    *  \see ManufacturerStrIndex structure entry.
	                    */
	uint8_t iSerialNumber; /**< String index for the product's globally unique hexadecimal
	                        *   serial number, in uppercase Unicode ASCII.
	                        *
	                        *  \note On some microcontroller models, there is an embedded serial number
	                        *        in the chip which can be used for the device serial number.
	                        *        To use this serial number, set this to USE_INTERNAL_SERIAL.
	                        *        On unsupported devices, this will evaluate to 0 and will cause
	                        *        the host to generate a pseudo-unique value for the device upon
	                        *        insertion.
	                        *
	                        *  \see ManufacturerStrIndex structure entry.
	                        */
	uint8_t  bNumConfigurations; /**< Total number of configurations supported by
	                              *   the device.
	                              */
} ATTR_PACKED USB_StdDescriptor_Device_t;

typedef struct
{
	USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */

	uint16_t USBSpecification; /**< BCD of the supported USB specification. */
	uint8_t  Class; /**< USB device class. */
	uint8_t  SubClass; /**< USB device subclass. */
	uint8_t  Protocol; /**< USB device protocol. */

	uint8_t  Endpoint0Size; /**< Size of the control (address 0) endpoint's bank in bytes. */
	uint8_t  NumberOfConfigurations; /**< Total number of configurations supported by
	                                  *   the device.
	                                  */
	uint8_t  Reserved; /**< Reserved for future use, must be 0. */
} ATTR_PACKED USB_Descriptor_DeviceQualifier_t;

typedef struct
{
	uint8_t  bLength; /**< Size of the descriptor, in bytes. */
	uint8_t  bDescriptorType; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a value
	                              *   given by the specific class.
	                              */
	uint16_t bcdUSB; /**< BCD of the supported USB specification. */
	uint8_t  bDeviceClass; /**< USB device class. */
	uint8_t  bDeviceSubClass; /**< USB device subclass. */
	uint8_t  bDeviceProtocol; /**< USB device protocol. */
	uint8_t  bMaxPacketSize0; /**< Size of the control (address 0) endpoint's bank in bytes. */
	uint8_t  bNumConfigurations; /**< Total number of configurations supported by
	                              *   the device.
	                              */
	uint8_t  bReserved; /**< Reserved for future use, must be 0. */
} ATTR_PACKED USB_StdDescriptor_DeviceQualifier_t;

typedef struct
{
	USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */

	uint16_t TotalConfigurationSize; /**< Size of the configuration descriptor header,
	                                  *   and all sub descriptors inside the configuration.
	                                  */
	uint8_t  TotalInterfaces; /**< Total number of interfaces in the configuration. */

	uint8_t  ConfigurationNumber; /**< Configuration index of the current configuration. */
	uint8_t  ConfigurationStrIndex; /**< Index of a string descriptor describing the configuration. */

	uint8_t  ConfigAttributes; /**< Configuration attributes, comprised of a mask of zero or
	                            *   more USB_CONFIG_ATTR_* masks.
	                            */

	uint8_t  MaxPowerConsumption; /**< Maximum power consumption of the device while in the
	                               *   current configuration, calculated by the \ref USB_CONFIG_POWER_MA()
	                               *   macro.
	                               */
} ATTR_PACKED USB_Descriptor_Configuration_Header_t;

typedef struct
{
	uint8_t  bLength; /**< Size of the descriptor, in bytes. */
	uint8_t  bDescriptorType; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a value
	                              *   given by the specific class.
	                              */
	uint16_t wTotalLength; /**< Size of the configuration descriptor header,
	                           *   and all sub descriptors inside the configuration.
	                           */
	uint8_t  bNumInterfaces; /**< Total number of interfaces in the configuration. */
	uint8_t  bConfigurationValue; /**< Configuration index of the current configuration. */
	uint8_t  iConfiguration; /**< Index of a string descriptor describing the configuration. */
	uint8_t  bmAttributes; /**< Configuration attributes, comprised of a mask of zero or
	                        *   more USB_CONFIG_ATTR_* masks.
	                        */
	uint8_t  bMaxPower; /**< Maximum power consumption of the device while in the
	                     *   current configuration, calculated by the \ref USB_CONFIG_POWER_MA()
	                     *   macro.
	                     */
} ATTR_PACKED USB_StdDescriptor_Configuration_Header_t;

typedef struct
{
	USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */

	uint8_t InterfaceNumber; /**< Index of the interface in the current configuration. */
	uint8_t AlternateSetting; /**< Alternate setting for the interface number. The same
	                           *   interface number can have multiple alternate settings
	                           *   with different endpoint configurations, which can be
	                           *   selected by the host.
	                           */
	uint8_t TotalEndpoints; /**< Total number of endpoints in the interface. */

	uint8_t Class; /**< Interface class ID. */
	uint8_t SubClass; /**< Interface subclass ID. */
	uint8_t Protocol; /**< Interface protocol ID. */

	uint8_t InterfaceStrIndex; /**< Index of the string descriptor describing the interface. */
} ATTR_PACKED USB_Descriptor_Interface_t;

typedef struct
{
	uint8_t bLength; /**< Size of the descriptor, in bytes. */
	uint8_t bDescriptorType; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a value
	                          *   given by the specific class.
	                          */
	uint8_t bInterfaceNumber; /**< Index of the interface in the current configuration. */
	uint8_t bAlternateSetting; /**< Alternate setting for the interface number. The same
	                            *   interface number can have multiple alternate settings
	                            *   with different endpoint configurations, which can be
	                            *   selected by the host.
	                            */
	uint8_t bNumEndpoints; /**< Total number of endpoints in the interface. */
	uint8_t bInterfaceClass; /**< Interface class ID. */
	uint8_t bInterfaceSubClass; /**< Interface subclass ID. */
	uint8_t bInterfaceProtocol; /**< Interface protocol ID. */
	uint8_t iInterface; /**< Index of the string descriptor describing the
	                     *   interface.
	                     */
} ATTR_PACKED USB_StdDescriptor_Interface_t;

typedef struct
{
	USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */

	uint8_t FirstInterfaceIndex; /**< Index of the first associated interface. */
	uint8_t TotalInterfaces; /**< Total number of associated interfaces. */

	uint8_t Class; /**< Interface class ID. */
	uint8_t SubClass; /**< Interface subclass ID. */
	uint8_t Protocol; /**< Interface protocol ID. */

	uint8_t IADStrIndex; /**< Index of the string descriptor describing the
	                      *   interface association.
	                      */
} ATTR_PACKED USB_Descriptor_Interface_Association_t;

typedef struct
{
	uint8_t bLength; /**< Size of the descriptor, in bytes. */
	uint8_t bDescriptorType; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a value
	                          *   given by the specific class.
	                          */
	uint8_t bFirstInterface; /**< Index of the first associated interface. */
	uint8_t bInterfaceCount; /**< Total number of associated interfaces. */
	uint8_t bFunctionClass; /**< Interface class ID. */
	uint8_t bFunctionSubClass; /**< Interface subclass ID. */
	uint8_t bFunctionProtocol; /**< Interface protocol ID. */
	uint8_t iFunction; /**< Index of the string descriptor describing the
	                    *   interface association.
	                    */
} ATTR_PACKED USB_StdDescriptor_Interface_Association_t;

typedef struct
{
	USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */

	uint8_t  EndpointAddress; /**< Logical address of the endpoint within the device for the current
	                           *   configuration, including direction mask.
	                           */
	uint8_t  Attributes; /**< Endpoint attributes, comprised of a mask of the endpoint type (EP_TYPE_*)
	                      *   and attributes (ENDPOINT_ATTR_*) masks.
	                      */
	uint16_t EndpointSize; /**< Size of the endpoint bank, in bytes. This indicates the maximum packet
	                        *   size that the endpoint can receive at a time.
	                        */
	uint8_t  PollingIntervalMS; /**< Polling interval in milliseconds for the endpoint if it is an INTERRUPT
	                             *   or ISOCHRONOUS type.
	                             */
} ATTR_PACKED USB_Descriptor_Endpoint_t;

typedef struct
{
	uint8_t  bLength; /**< Size of the descriptor, in bytes. */
	uint8_t  bDescriptorType; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a
	                           *   value given by the specific class.
	                           */
	uint8_t  bEndpointAddress; /**< Logical address of the endpoint within the device for the current
	                            *   configuration, including direction mask.
	                            */
	uint8_t  bmAttributes; /**< Endpoint attributes, comprised of a mask of the endpoint type (EP_TYPE_*)
	                        *   and attributes (ENDPOINT_ATTR_*) masks.
	                        */
	uint16_t wMaxPacketSize; /**< Size of the endpoint bank, in bytes. This indicates the maximum packet size
	                          *   that the endpoint can receive at a time.
	                          */
	uint8_t  bInterval; /**< Polling interval in milliseconds for the endpoint if it is an INTERRUPT or
	                     *   ISOCHRONOUS type.
	                     */
} ATTR_PACKED USB_StdDescriptor_Endpoint_t;

typedef struct
{
	USB_Descriptor_Header_t Header; /**< Descriptor header, including type and size. */

	wchar_t  UnicodeString[];
} ATTR_PACKED USB_Descriptor_String_t;

typedef struct
{
	uint8_t bLength; /**< Size of the descriptor, in bytes. */
	uint8_t bDescriptorType; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t
	                          *   or a value given by the specific class.
	                          */
	uint16_t bString[]; /**< String data, as unicode characters (alternatively, string language IDs).
	                     *   If normal ASCII characters are to be used, they must be added as an array
	                     *   of characters rather than a normal C string so that they are widened to
	                     *   Unicode size.
	                     *
	                     *   Under GCC, strings prefixed with the "L" character (before the opening string
	                     *   quotation mark) are considered to be Unicode strings, and may be used instead
	                     *   of an explicit array of ASCII characters.
	                     */
} ATTR_PACKED USB_StdDescriptor_String_t;

/* Macros: */

typedef struct USB_Request_Header
{
	uint8_t  bmRequestType; /**< Type of the request. */
	uint8_t  bRequest; /**< Request command code. */
	uint16_t wValue; /**< wValue parameter of the request. */
	uint16_t wIndex; /**< wIndex parameter of the request. */
	uint16_t wLength; /**< Length of the data to transfer in bytes. */
} ATTR_PACKED USB_Request_Header_t;

enum USB_Control_Request_t
{
	REQ_GetStatus           = 0, /**< Implemented in the library for device and endpoint recipients. Passed
	                              *   to the user application for other recipients via the
	                              *   \ref EVENT_USB_Device_ControlRequest() event when received in
	                              *   device mode. */
	REQ_ClearFeature        = 1, /**< Implemented in the library for device and endpoint recipients. Passed
	                              *   to the user application for other recipients via the
	                              *   \ref EVENT_USB_Device_ControlRequest() event when received in
	                              *   device mode. */
	REQ_SetFeature          = 3, /**< Implemented in the library for device and endpoint recipients. Passed
	                              *   to the user application for other recipients via the
	                              *   \ref EVENT_USB_Device_ControlRequest() event when received in
	                              *   device mode. */
	REQ_SetAddress          = 5, /**< Implemented in the library for the device recipient. Passed
	                              *   to the user application for other recipients via the
	                              *   \ref EVENT_USB_Device_ControlRequest() event when received in
	                              *   device mode. */
	REQ_GetDescriptor       = 6, /**< Implemented in the library for device and interface recipients. Passed to the
	                              *   user application for other recipients via the
	                              *   \ref EVENT_USB_Device_ControlRequest() event when received in
	                              *   device mode. */
	REQ_SetDescriptor       = 7, /**< Not implemented in the library, passed to the user application
	                              *   via the \ref EVENT_USB_Device_ControlRequest() event when received in
	                              *   device mode. */
	REQ_GetConfiguration    = 8, /**< Implemented in the library for the device recipient. Passed
	                              *   to the user application for other recipients via the
	                              *   \ref EVENT_USB_Device_ControlRequest() event when received in
	                              *   device mode. */
	REQ_SetConfiguration    = 9, /**< Implemented in the library for the device recipient. Passed
	                              *   to the user application for other recipients via the
	                              *   \ref EVENT_USB_Device_ControlRequest() event when received in
	                              *   device mode. */
	REQ_GetInterface        = 10, /**< Not implemented in the library, passed to the user application
	                              *   via the \ref EVENT_USB_Device_ControlRequest() event when received in
	                              *   device mode. */
	REQ_SetInterface        = 11, /**< Not implemented in the library, passed to the user application
	                              *   via the \ref EVENT_USB_Device_ControlRequest() event when received in
	                              *   device mode. */
	REQ_SynchFrame          = 12, /**< Not implemented in the library, passed to the user application
	                              *   via the \ref EVENT_USB_Device_ControlRequest() event when received in
	                              *   device mode. */
};
			

/* Private Interface - For use in library only: */
/* Macros: */

/* Enums: */
/** Enum for the various states of the USB Device state machine. Only some states are
 *  implemented in the LUFA library - other states are left to the user to implement.
 *
 *  For information on each possible USB device state, refer to the USB 2.0 specification.
 *
 *  \see \ref USB_DeviceState, which stores the current device state machine state.
 */
enum USB_Device_States_t
{
	DEVICE_STATE_Unattached                   = 0, /**< Internally implemented by the library. This state indicates
	                                                *   that the device is not currently connected to a host.
	                                                */
	DEVICE_STATE_Powered                      = 1, /**< Internally implemented by the library. This state indicates
	                                                *   that the device is connected to a host, but enumeration has not
	                                                *   yet begun.
	                                                */
	DEVICE_STATE_Default                      = 2, /**< Internally implemented by the library. This state indicates
                                                *   that the device's USB bus has been reset by the host and it is
	                                                *   now waiting for the host to begin the enumeration process.
	                                                */
	DEVICE_STATE_Addressed                    = 3, /**< Internally implemented by the library. This state indicates
	                                                *   that the device has been addressed by the USB Host, but is not
	                                                *   yet configured.
	                                                */
	DEVICE_STATE_Configured                   = 4, /**< May be implemented by the user project. This state indicates
	                                                *   that the device has been enumerated by the host and is ready
	                                                *   for USB communications to begin.
	                                                */
	DEVICE_STATE_Suspended                    = 5, /**< May be implemented by the user project. This state indicates
	                                                *   that the USB bus has been suspended by the host, and the device
	                                                *   should power down to a minimal power level until the bus is
	                                                *   resumed.
	                                                */
};



#endif
