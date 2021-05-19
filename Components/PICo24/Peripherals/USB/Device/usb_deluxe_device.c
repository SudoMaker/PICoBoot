/*
    This file is part of PotatoPi PICo24 SDK.

    Copyright (C) 2021 ReimuNotMoe <reimu@sudomaker.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "usb_deluxe_device.h"
#include "../usb_deluxe.h"

USBDeviceDescriptorContext usb_device_desc_ctx;
USBDeviceDriverContext *usb_device_driver_ctx = NULL;
uint8_t usb_device_driver_ctx_size = 0;

void USBDeluxe_Device_ConfigInit() {
	USBDeluxe_DeviceDescriptor_Init();
}

void USBDeluxe_Device_ConfigApply() {
	USBDeluxe_DeviceDescriptor_RefreshSize();
}

void USBDeluxe_DeviceDescriptor_Init() {
	const uint8_t desc_hdr[] = {
		0x09,					// Size of this descriptor in bytes
		USB_DESCRIPTOR_CONFIGURATION,		// CONFIGURATION descriptor type
		0x09, 0x00,				// Total length of data for this cfg, **little endian**
		1,					// Number of interfaces in this cfg
		1,					// Index value of this configuration
		0,					// Configuration string index
		_DEFAULT|_SELF,				// Attributes, see usbdefs_std_dsc.h
		50,					// Max power consumption (2X mA)
	};

	memset(&usb_device_desc_ctx, 0, sizeof(USBDeviceDescriptorContext));
	usb_device_desc_ctx.used_endpoints = 1;

	memcpy(usb_device_desc_ctx.raw, desc_hdr, sizeof(desc_hdr));
	usb_device_desc_ctx.used_length = sizeof(desc_hdr);
}

void USBDeluxe_DeviceDescriptor_RefreshSize() {
	uint16_t len = usb_device_desc_ctx.used_length;
	usb_device_desc_ctx.raw[2] = len & 0xff;
	usb_device_desc_ctx.raw[3] = len >> 8;
	usb_device_desc_ctx.raw[4] = usb_device_desc_ctx.used_interfaces;
}

uint8_t USBDeluxe_DeviceDescriptor_InsertInterface(uint8_t alternate_setting, uint8_t nr_endpoints, uint8_t class, uint8_t subclass, uint8_t protocol) {
	uint8_t ret = usb_device_desc_ctx.used_interfaces;
	uint8_t *p = usb_device_desc_ctx.raw + usb_device_desc_ctx.used_length;

	p[0] = 0x9;					// Size of this descriptor in bytes
	p[1] = USB_DESCRIPTOR_INTERFACE;		// INTERFACE descriptor type
	p[2] = usb_device_desc_ctx.used_interfaces;	// Interface Number
	p[3] = alternate_setting;			// Alternate Setting Number
	p[4] = nr_endpoints;				// Number of endpoints in this intf
	p[5] = class;					// Class code
	p[6] = subclass;				// Subclass code
	p[7] = protocol;				// Protocol code
	p[8] = 0x0;					// Interface string index

	usb_device_desc_ctx.used_length += 9;
	usb_device_desc_ctx.used_interfaces += 1;

	return ret;
}

void USBDeluxe_DeviceDescriptor_InsertEndpointRaw(uint8_t addr, uint8_t attr, uint16_t size, uint8_t interval,
						  int16_t opt_refresh, int16_t opt_sync_addr) {
	uint8_t *p = usb_device_desc_ctx.raw + usb_device_desc_ctx.used_length;
	p[0] = 0x7;					// Size of this descriptor in bytes
	p[1] = USB_DESCRIPTOR_ENDPOINT;			// Endpoint Descriptor
	p[2] = addr;					// EndpointAddress
	p[3] = attr;					// Attributes
	p[4] = size & 0xff;				// Size, <7:0>
	p[5] = size >> 8;				// Size, <15:8>
	p[6] = interval;				// Interval

	usb_device_desc_ctx.used_length += 7;

	if (opt_refresh != -1) {
		p[0]++;
		p[7] = opt_refresh;
		usb_device_desc_ctx.used_length += 1;
	}

	if (opt_sync_addr != -1) {
		p[0]++;
		p[8] = opt_sync_addr;
		usb_device_desc_ctx.used_length += 1;
	}

}

uint8_t USBDeluxe_DeviceDescriptor_InsertEndpoint(uint8_t direction, uint8_t attr, uint16_t size, uint8_t interval, int16_t opt_refresh, int16_t opt_sync_addr) {
	uint8_t next_free_ep = usb_device_desc_ctx.used_endpoints;

	if (direction & USB_EP_DIR_IN) {
		USBDeluxe_DeviceDescriptor_InsertEndpointRaw(next_free_ep | 0x80, attr, size, interval, opt_refresh, opt_sync_addr);
	}

	if (direction & USB_EP_DIR_OUT) {
		USBDeluxe_DeviceDescriptor_InsertEndpointRaw(next_free_ep, attr, size, interval, opt_refresh, opt_sync_addr);
	}

	usb_device_desc_ctx.used_endpoints++;
	return next_free_ep;
}

void USBDeluxe_DeviceDescriptor_InsertIAD(uint8_t iface, uint8_t nr_contiguous_ifaces, uint8_t class, uint16_t subclass, uint8_t protocol) {
	uint8_t *p = usb_device_desc_ctx.raw + usb_device_desc_ctx.used_length;
	p[0] = 0x8;					// Size of this descriptor in bytes
	p[1] = 0x0b;					// Interface assocication descriptor type
	p[2] = iface;					// The first associated interface
	p[3] = nr_contiguous_ifaces;			// Number of contiguous associated interface
	p[4] = class;					// bInterfaceClass of the first interface
	p[5] = subclass;				// bInterfaceSubclass of the first interface
	p[6] = protocol;				// bInterfaceProtocol of the first interface
	p[7] = 0;					// Interface string index


	usb_device_desc_ctx.used_length += 8;
}

void USBDeluxe_DeviceDescriptor_InsertCDCSpecific(uint8_t comm_iface, uint8_t data_iface) {
	uint8_t *p;

	/* CDC Class-Specific Descriptors */

	// 5 bytes: Header Functional Descriptor
	p = usb_device_desc_ctx.raw + usb_device_desc_ctx.used_length;
	p[0] = 0x5;					// Size of this descriptor in bytes
	p[1] = CS_INTERFACE;				// bDescriptorType (class specific)
	p[2] = DSC_FN_HEADER;				// bDescriptorSubtype (header functional descriptor)
	p[3] = 0x10;					// bcdCDC (CDC spec version this fw complies with: v1.20 [stored in little endian])
	p[4] = 0x01;					// ^
	usb_device_desc_ctx.used_length += 5;

	// 4 bytes: Abstract Control Management Functional Descriptor
	p = usb_device_desc_ctx.raw + usb_device_desc_ctx.used_length;
	p[0] = 0x4;					// Size of this descriptor in bytes
	p[1] = CS_INTERFACE;				// bDescriptorType (class specific)
	p[2] = DSC_FN_ACM;				// bDescriptorSubtype (abstract control management)
	p[3] = USB_CDC_ACM_FN_DSC_VAL;			// bmCapabilities: (see PSTN120.pdf Table 4)
	usb_device_desc_ctx.used_length += 4;

	// 5 bytes: Union Functional Descriptor
	p = usb_device_desc_ctx.raw + usb_device_desc_ctx.used_length;
	p[0] = 0x5;					// Size of this descriptor in bytes
	p[1] = CS_INTERFACE;				// bDescriptorType (class specific)
	p[2] = DSC_FN_UNION;				// bDescriptorSubtype (union functional)
	p[3] = comm_iface;				// bControlInterface: Interface number of the communication class interface (1)
	p[4] = data_iface;				// bSubordinateInterface0: Data class interface #2 is subordinate to this interface
	usb_device_desc_ctx.used_length += 5;

	// 5 bytes: Call Management Functional Descriptor
	p = usb_device_desc_ctx.raw + usb_device_desc_ctx.used_length;
	p[0] = 0x5;					// Size of this descriptor in bytes
	p[1] = CS_INTERFACE;				// bDescriptorType (class specific)
	p[2] = DSC_FN_CALL_MGT;				// bDescriptorSubtype (call management functional)
	p[3] = 0x00;					// bmCapabilities: device doesn't handle call management
	p[4] = data_iface;				// bDataInterface: Data class interface ID used for the optional call management
	usb_device_desc_ctx.used_length += 5;
}


USBDeviceDriverContext *USBDeluxe_DeviceDriver_AllocateMemory() {
	uint8_t oldpos = usb_device_driver_ctx_size;
	usb_device_driver_ctx_size++;

	if (usb_device_driver_ctx) {
		usb_device_driver_ctx = realloc(usb_device_driver_ctx, sizeof(USBDeviceDriverContext) * usb_device_driver_ctx_size);
	} else {
		usb_device_driver_ctx = malloc(sizeof(USBDeviceDriverContext) * usb_device_driver_ctx_size);
	}

	USBDeviceDriverContext *ret = (USBDeviceDriverContext *)(((uint8_t *)usb_device_driver_ctx) + sizeof(USBDeviceDriverContext) * oldpos);
	memset(ret, 0, sizeof(USBDeviceDriverContext));

	return ret;
}

uint8_t USBDeluxe_DeviceFunction_Add_CDC(void *userp, USBDeluxeDevice_CDC_IOps *io_ops) {
	uint8_t last_idx = usb_device_driver_ctx_size;

	USBDeluxe_DeviceDescriptor_InsertIAD(usb_device_desc_ctx.used_interfaces, 2, COMM_INTF, ABSTRACT_CONTROL_MODEL, V25TER);

	uint8_t iface_comm = USBDeluxe_DeviceDescriptor_InsertInterface(0, 1, COMM_INTF, ABSTRACT_CONTROL_MODEL, V25TER);
	USBDeluxe_DeviceDescriptor_InsertCDCSpecific(iface_comm, usb_device_desc_ctx.used_interfaces);
	uint8_t ep_comm = USBDeluxe_DeviceDescriptor_InsertEndpoint(USB_EP_DIR_IN, _INTERRUPT, 10, 2, -1, -1);

	uint8_t iface_data = USBDeluxe_DeviceDescriptor_InsertInterface(0, 2, DATA_INTF, 0, 0);
	uint8_t ep_data = USBDeluxe_DeviceDescriptor_InsertEndpoint(USB_EP_DIR_IN|USB_EP_DIR_OUT, _BULK, 64, 0, -1, -1);

	USBDeviceDriverContext *ctx = USBDeluxe_DeviceDriver_AllocateMemory();

	ctx->func = USB_FUNC_CDC;
	USBDeluxeDevice_CDC_Create(&ctx->cdc, userp, iface_comm, iface_data, ep_comm, ep_data, io_ops);

	return last_idx;
}

inline __attribute__((always_inline)) USBDeviceDriverContext *USBDeluxe_DeviceGetDriverContext(uint8_t index) {
	return (USBDeviceDriverContext *)(((uint8_t *)usb_device_driver_ctx) + sizeof(USBDeviceDriverContext) * index);
}

void USBDeluxe_Device_EventInit() {
	for (size_t i=0; i<usb_device_driver_ctx_size; i++) {
		USBDeviceDriverContext *drv_ctx = &usb_device_driver_ctx[i];

		switch (drv_ctx->func) {
			case USB_FUNC_CDC:
				USBDeluxeDevice_CDC_Init(&drv_ctx->cdc);
				break;
			default:
				break;
		}
	}
}

void USBDeluxe_Device_EventCheckRequest() {
	for (size_t i=0; i<usb_device_driver_ctx_size; i++) {
		USBDeviceDriverContext *drv_ctx = &usb_device_driver_ctx[i];

		switch (drv_ctx->func) {
			case USB_FUNC_CDC:
				USBDeluxeDevice_CDC_CheckRequest(&drv_ctx->cdc);
				break;
			default:
				break;
		}
	}
}

//static uint8_t task_counter = 0;

void USBDeluxe_Device_Tasks() {
	if (USBGetDeviceState() < CONFIGURED_STATE) {
		return;
	}

	if (USBIsDeviceSuspended()== true) {
		return;
	}

	for (size_t i=0; i<usb_device_driver_ctx_size; i++) {
		USBDeviceDriverContext *drv_ctx = &usb_device_driver_ctx[i];

		switch (drv_ctx->func) {
			case USB_FUNC_CDC:
				USBDeluxeDevice_CDC_Tasks(&drv_ctx->cdc);
				break;
			default:
				break;
		}
	}
}

