/*
 * Copyright (C) 2017 Alexander Hold <darksunhd@gmail.com>
 *
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */


/*
 * Date: 5.10.2017
 * Version 2.0
 *
 * Converts Serial-MIDI to USB-MIDI
 *
 * USB is interrupt driven!
 * Normaly USB-MIDI should have a BULK endpoint, but due to the fact that i want it to be
 * interrupt driven I used an INTERRUPT endpoint (PC -> converter).
 * converter -> PC still BULK endpoint.
 *
 * UART RX is also interrupt driven.
 */


/*
 * All references in this file come from Universal Serial Bus Device Class
 * Definition for MIDI Devices, release 1.0.
 */






#include <stdlib.h>
#include <stdint.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencmsis/core_cm3.h>





static usbd_device *usbd_dev;


void usb_isr(usbd_device *dev, uint8_t ep);


static const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,     /*Type: uint8_t   Size: 1   Description: Size of this descriptor in bytes*/
	.bDescriptorType = USB_DT_DEVICE,  /*Type: uint8_t   Size: 1   Descriptor: Device Descriptor Type = 1*/
	.bcdUSB = 0x0200,        /*Type: uint16_t   Size: 2   Description: This field identifies the release of the USB Specification with which the device and its descriptors are compliant. */
	.bDeviceClass = 0,        /*Type: uint8_t   Size: 1   Description: Class code (assigned by the USB-IF)   0 = each interface within a configuration specifies its own class information and the various interfaces operate independently.*/
	.bDeviceSubClass = 0,     /*Type: uint8_t   Size: 1   Description: Subclass code (assigned by the USB-IF)   if bDeviceClass = 0 then bDeviceSubClass = 0*/
	.bDeviceProtocol = 0,     /*Type: uint8_t   Size: 1   Description: Protocol code (assigned by the USB-IF)   0 = the device does not use class specific protocols on a device basis. However, it may use class specific protocols on an interface basis*/
	.bMaxPacketSize0 = 64,    /*Type: uint8_t   Size: 1   Description: Maximum packet size for Endpoint zero (only 8, 16, 32, or 64 are valid)*/
	.idVendor = 0x6666,      /*Type: uint16_t   Size: 2   Description: Vendor ID (assigned by the USB-IF)*/
	.idProduct = 0x5119,     /*Type: uint16_t   Size: 2   Description: Product ID (assigned by the manufacturer)*/
	.bcdDevice = 0x0100,     /*Type: uint16_t   Size: 2   Description: Device release number in binary-coded decimal*/
	.iManufacturer = 1,       /*Type: uint8_t   Size: 1   Description: Index of string descriptor describing manufacturer*/
	.iProduct = 2,            /*Type: uint8_t   Size: 1   Description: Index of string descriptor describing product*/
	.iSerialNumber = 3,       /*Type: uint8_t   Size: 1   Description: Index of string descriptor describing the device's serial number*/
	.bNumConfigurations = 1,  /*Type: uint8_t   Size: 1   Description: Number of possible configurations*/
};

/*
 * Midi specific endpoint descriptors.
 */
static const struct usb_midi_endpoint_descriptor midi_usb_endp[] = {{
	/* Table B-12: MIDI Adapter Class-specific Bulk OUT Endpoint
	 * Descriptor, but we use an Interrupt driven Endpoint
	 */
	.head = {
		.bLength = sizeof(struct usb_midi_endpoint_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
		.bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
		.bNumEmbMIDIJack = 1,
	},
	.jack[0] = {
		.baAssocJackID = 0x01,
	},
}, {
	/* Table B-14: MIDI Adapter Class-specific Bulk IN Endpoint
	 * Descriptor
	 */
	.head = {
		.bLength = sizeof(struct usb_midi_endpoint_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
		.bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
		.bNumEmbMIDIJack = 1,
	},
	.jack[0] = {
		.baAssocJackID = 0x03,
	},
} };

/*
 * Standard endpoint descriptors
 */
static const struct usb_endpoint_descriptor usb_endp[] = {{
	/* Table B-11: MIDI Adapter Standard Bulk OUT Endpoint Descriptor, but we use an Interrupt driven Endpoint*/
	.bLength = USB_DT_ENDPOINT_SIZE,    /*Type: uint8_t   Size: 1   Descriptor: Size of this descriptor in bytes*/
	.bDescriptorType = USB_DT_ENDPOINT, /*Type: uint8_t   Size: 1   Descriptor: Endpoint Descriptor Type = 5. */
	.bEndpointAddress = 0x01,           /*Type: uint8_t   Size: 1   Descriptor: The address of the endpoint on the USB device described by this descriptor. The address is encoded as follows:  0-3: The endpoint number   4-6: Reserved, reset to zero   7: Direction, ignored for control endpoints -> 0 = OUT   1 = IN*/
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,  /*Type: uint8_t   Size: 1
Description:
The endpoint attribute when configured through bConfigurationValue.
    Bits 1..0: Transfer Type
        00 = Control
        01 = Isochronous
        10 = Bulk
        11 = Interrupt
For non-isochronous endpoints, bits 5..2 must be set to zero. For isochronous endpoints, they are defined as:
    Bits 3..2: Synchronization Type
        00 = No Synchronization
        01 = Asynchronous
        10 = Adaptive
        11 = Synchronous
    Bits 5..4: Usage Type
        00 = Data
        01 = Feedback
        10 = Implicit feedback
        11 = Reserved
All other bits are reserved and must be reset to zero. */


	.wMaxPacketSize = 0x40, /*Type: uint16_t   Size: 2
Description:
Is the maximum packet size of this endpoint. For isochronous endpoints, this value is used to reserve the time on the bus, required for the per-(micro)frame data payloads.
    Bits 10..0 = max. packet size (in bytes).
For high-speed isochronous and interrupt endpoints:
    Bits 12..11 = number of additional transaction opportunities per micro-frame:
        00 = None (1 transaction per micro-frame)
        01 = 1 additional (2 per micro-frame)
        10 = 2 additional (3 per micro-frame)
        11 = Reserved
    Bits 15..13 are reserved and must be set to zero. */


	.bInterval = 0x00, /*Type: uint8_t   Size: 1
Description:
Interval for polling endpoint for data transfers. Expressed in frames or micro-frames depending on the operating speed (1ms, or 125μs units).
    For full-/high-speed isochronous endpoints, this value must be in the range from 1 to 16. The bInterval value is used as the exponent for a 2bInterval-1 value; For example, a bInterval of 4 means a period of 8 (24-1).
    For full-/low-speed interrupt endpoints, the value of this field may be from 1 to 255.
    For high-speed interrupt endpoints, the bInterval value is used as the exponent for a 2bInterval-1 value; For Example, a bInterval of 4 means a period of 8 (24-1). This value must be from 1 to 16.
    For high-speed bulk/control OUT endpoints, the bInterval must specify the maximum NAK rate of the endpoint. A value of 0 indicates the endpoint never NAKs. Other values indicate at most 1 NAK each bInterval number of microframes. This value must be in the range from 0 to 255.*/

	.extra = &midi_usb_endp[0],		/*Needed?*/
	.extralen = sizeof(midi_usb_endp[0])	/*Needed?*/
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,         /*Look above*/
	.bDescriptorType = USB_DT_ENDPOINT,      /*Look above*/
	.bEndpointAddress = 0x81,		 /*Look above*/
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,  /*Look above*/
	.wMaxPacketSize = 0x40,			 /*Look above*/
	.bInterval = 0x00,			 /*Look above*/

	.extra = &midi_usb_endp[1],		/*Needed?*/
	.extralen = sizeof(midi_usb_endp[1])	/*Needed?*/
} };

/*
 * Table B-4: MIDI Adapter Class-specific AC Interface Descriptor
 */
static const struct {
	struct usb_audio_header_descriptor_head header_head;
	struct usb_audio_header_descriptor_body header_body;
} __attribute__((packed)) audio_control_functional_descriptors = {
	.header_head = {
		.bLength = sizeof(struct usb_audio_header_descriptor_head) +
		           1 * sizeof(struct usb_audio_header_descriptor_body),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_HEADER,
		.bcdADC = 0x0100,
		.wTotalLength =
			   sizeof(struct usb_audio_header_descriptor_head) +
			   1 * sizeof(struct usb_audio_header_descriptor_body),
		.binCollection = 1,
	},
	.header_body = {
		.baInterfaceNr = 0x01,
	},
};

/*
 * Table B-3: MIDI Adapter Standard AC Interface Descriptor
 */
static const struct usb_interface_descriptor audio_control_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_CONTROL,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.extra = &audio_control_functional_descriptors,
	.extralen = sizeof(audio_control_functional_descriptors)
} };

/*
 * Class-specific MIDI streaming interface descriptor
 */
static const struct {
	struct usb_midi_header_descriptor header;
	struct usb_midi_in_jack_descriptor in_embedded;
	struct usb_midi_in_jack_descriptor in_external;
	struct usb_midi_out_jack_descriptor out_embedded;
	struct usb_midi_out_jack_descriptor out_external;
} __attribute__((packed)) midi_streaming_functional_descriptors = {
	/* Table B-6: Midi Adapter Class-specific MS Interface Descriptor */
	.header = {
		.bLength = sizeof(struct usb_midi_header_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MS_HEADER,
		.bcdMSC = 0x0100,
		.wTotalLength = sizeof(midi_streaming_functional_descriptors),
	},
	/* Table B-7: MIDI Adapter MIDI IN Jack Descriptor (Embedded) */
	.in_embedded = {
		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
		.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
		.bJackID = 0x01,
		.iJack = 0x00,
	},
	/* Table B-8: MIDI Adapter MIDI IN Jack Descriptor (External) */
	.in_external = {
		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
		.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
		.bJackID = 0x02,
		.iJack = 0x00,
	},
	/* Table B-9: MIDI Adapter MIDI OUT Jack Descriptor (Embedded) */
	.out_embedded = {
		.head = {
			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
			.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
			.bJackID = 0x03,
			.bNrInputPins = 1,
		},
		.source[0] = {
			.baSourceID = 0x02,
			.baSourcePin = 0x01,
		},
		.tail = {
			.iJack = 0x00,
		}
	},
	/* Table B-10: MIDI Adapter MIDI OUT Jack Descriptor (External) */
	.out_external = {
		.head = {
			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
			.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
			.bJackID = 0x04,
			.bNrInputPins = 1,
		},
		.source[0] = {
			.baSourceID = 0x01,
			.baSourcePin = 0x01,
		},
		.tail = {
			.iJack = 0x00,
		},
	},
};

/*
 * Table B-5: MIDI Adapter Standard MS Interface Descriptor
 */
static const struct usb_interface_descriptor midi_streaming_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_MIDISTREAMING,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = usb_endp,

	.extra = &midi_streaming_functional_descriptors,
	.extralen = sizeof(midi_streaming_functional_descriptors)
} };

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = audio_control_iface,
}, {
	.num_altsetting = 1,
	.altsetting = midi_streaming_iface,
} };

/*
 * Table B-2: MIDI Adapter Configuration Descriptor
 */
static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0, /* can be anything, it is updated automatically
			      when the usb code prepares the descriptor */
	.bNumInterfaces = 2, /* control and data */
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80, /* bus powered */
	.bMaxPower = 0x32,

	.interface = ifaces,
};

/*USB Strings (Look at USB-Device-Descriptor)*/
static const char * usb_strings[] = {
	"Hold-Solutions.com",		/*Manufacturer*/
	"Alex Hold | Serial -> MIDI",	/*Product*/
	"AHSM00001\0"			/*SerialNumber*/
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

/* SysEx identity message, preformatted with correct USB framing information */
const uint8_t sysex_identity[] = {
	0x04,	/* USB Framing (3 byte SysEx) */
	0xf0,	/* SysEx start */
	0x7e,	/* non-realtime */
	0x00,	/* Channel 0 */
	0x04,	/* USB Framing (3 byte SysEx) */
	0x7d,	/* Educational/prototype manufacturer ID */
	0x66,	/* Family code (byte 1) */
	0x66,	/* Family code (byte 2) */
	0x04,	/* USB Framing (3 byte SysEx) */
	0x51,	/* Model number (byte 1) */
	0x19,	/* Model number (byte 2) */
	0x00,	/* Version number (byte 1) */
	0x04,	/* USB Framing (3 byte SysEx) */
	0x00,	/* Version number (byte 2) */
	0x01,	/* Version number (byte 3) */
	0x00,	/* Version number (byte 4) */
	0x05,	/* USB Framing (1 byte SysEx) */
	0xf7,	/* SysEx end */
	0x00,	/* Padding */
	0x00,	/* Padding */
};





/*FIFO*/
typedef struct {
	uint8_t *read;	/*Read pointer*/
	uint8_t *write; /*Write pointer*/
	size_t size;	/*Size of the FIFO*/
	uint8_t *start; /*Start adress pointer*/
	uint8_t *end;	/*End adress pointer*/
	uint8_t data;   /*current data (read return)*/
	uint8_t empty;  /*is the FIFO empty -> 1=yes 0=no*/
	uint8_t midi_commands; /*number of midi commands (1 command = 3 8bit)*/
} FIFO;



static FIFO uart_FIFO;		/*UART FIFO -> UART writes in this FIFO*/
static FIFO usb_FIFO;		/*USB FIFO -> USB writes in this FIFO*/


static FIFO FIFO_setup(FIFO fifo, size_t size){
	fifo.size = size;
	fifo.start = malloc(fifo.size * sizeof(uint8_t));	/*8 bit, because MIDI packets are 8 bit long*/
	fifo.end = fifo.start + size;
	fifo.write = fifo.start;
	fifo.read = fifo.start;
	return fifo;
}

static FIFO FIFO_write(FIFO fifo, uint8_t data){
	if(fifo.write == fifo.end){
		if(fifo.read != fifo.start){
			fifo.write = fifo.start;
		}else{
			//FIFO full
			return fifo;
		}
	}else{
		if((fifo.write + 1) != fifo.read){ 
			fifo.write = fifo.write + 1;
		}else{
			//FIFO full
			return fifo;
		}
	}
	*fifo.write = data;
	return fifo;
}


static FIFO FIFO_read(FIFO fifo){
	if(fifo.read == fifo.end){
		if(fifo.write != fifo.end){
			fifo.read = fifo.start;
		}else{
			//FIFO empty
			fifo.empty = 1;
			return fifo;
		}
	}else{
		if(fifo.read != fifo.write){
			fifo.read = fifo.read + 1;
		}else{
			//FIFO empty
			fifo.empty = 1;
			return fifo;
		}
	}
	fifo.data = *fifo.read;  /*write the read pointer into the data var*/
	fifo.empty = 0;		 /*set fifo = not empty*/
	return fifo;
}






static void uart_setup(void) {
	nvic_enable_irq(NVIC_USART1_IRQ);

	/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port B for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

	/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port B for rceive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);

	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART1);
}

static void usb_setup(usbd_device *dev, uint16_t wValue)
{

	(void)wValue;

	/* Setup USB Receive interrupt. */
	usbd_ep_setup(dev, 0x01, USB_ENDPOINT_ATTR_INTERRUPT, 64, usb_isr);

	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);

	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(99999);
	systick_interrupt_enable();
	systick_counter_enable();

}

void usb_lp_can_rx0_isr(void) {
    	usbd_poll(usbd_dev);
}


uint8_t uart_midi_counter = 0;
void usart1_isr(void)
{
	
	static uint8_t data = 'A';
	data = usart_recv(USART1);

	uart_midi_counter++;
	uart_FIFO = FIFO_write(uart_FIFO, data);

	if(uart_midi_counter == 3){
		uart_FIFO.midi_commands++;
		uart_midi_counter = 0;
	}
}

void usb_isr(usbd_device *dev, uint8_t ep){
	(void)ep;
	
	//TODO usb -> serial
	char buf[64];
	int len = usbd_ep_read_packet(dev, 0x01, buf, 64);
	gpio_toggle(GPIOC, GPIO13); 
}



static void usb_send(usbd_device *dev){

	/*read command (command + channel) form FIFO*/
	uart_FIFO = FIFO_read(uart_FIFO);

	if(uart_FIFO.empty == 1){
		//Error!
		return;
	}

	uint8_t midi_command = uart_FIFO.data;


	/*read note from FIFO*/
	uart_FIFO = FIFO_read(uart_FIFO);

	if(uart_FIFO.empty == 1){
		//Error!
		return;
	}
	
	uint8_t midi_note = uart_FIFO.data;


	/*read velocity form FIFO*/
	uart_FIFO = FIFO_read(uart_FIFO);

	if(uart_FIFO.empty == 1){
		//Error!
		return;
	}

	uint8_t midi_velocity = uart_FIFO.data;

	//MIDI Packet	
	char buf[4] = {
		0x08,
		midi_command,	/*command = command 3bit (ex. note on) + channel 4bit*/
		midi_note,	/*note 0-127*/
		midi_velocity	/*velocity 0 - 127*/
	};

	while (usbd_ep_write_packet(dev, 0x81, buf, sizeof(buf)) == 0);

}


static void loop(void){
	while(1){
		if(uart_FIFO.midi_commands > 0){
			usb_send(usbd_dev);
			uart_FIFO.midi_commands--; 
		}
		__asm__("nop");
	}
}



int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();


	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);
	

	/*FIFO Setup*/
	uart_FIFO = FIFO_setup(uart_FIFO, 64);
	usb_FIFO = FIFO_setup(usb_FIFO, 64);

	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config,usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, usb_setup);
	
	/*Wait for USB to register on the Pc*/
	/*incoming uart data would kill the registration process if we would not wait*/
	for (int i = 0; i < 0x800000; i++){
		__asm__("nop");
	}

	/*Wait for USB Vbus.*/
	while (gpio_get(GPIOA, GPIO8) == 0){
		__asm__("nop");
	}
	
	rcc_periph_clock_enable(RCC_USART1);
	uart_setup();

	/*Led of the STM32 board*/
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	gpio_set(GPIOC, GPIO13);

	loop();
}
