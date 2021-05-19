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

#pragma once

#include <stdlib.h>
#include <string.h>

#include "../usb.h"
#include "usb_device.h"

#include "usb_deluxe_device_cdc.h"

typedef enum {
	USB_EP_DIR_IN = 0x1,
	USB_EP_DIR_OUT = 0x2
} USBEndpointDirection;

typedef struct {
	uint8_t raw[1024];
	uint16_t used_length;
	uint16_t *total_length;

	uint16_t used_interfaces;
	uint16_t used_endpoints;
} USBDeviceDescriptorContext;

typedef struct {
	USBFunction func;

	union {
		USBDeluxeDevice_CDCContext cdc;
	};
} USBDeviceDriverContext;


extern USBDeviceDescriptorContext usb_device_desc_ctx;
extern USBDeviceDriverContext *usb_device_driver_ctx;
extern uint8_t usb_device_driver_ctx_size;

extern void USBDeluxe_Device_ConfigInit();
extern void USBDeluxe_Device_ConfigApply();

extern void USBDeluxe_DeviceDescriptor_Init();
extern void USBDeluxe_DeviceDescriptor_RefreshSize();

extern void USBDeluxe_Device_EventInit();
extern void USBDeluxe_Device_EventCheckRequest();

extern USBDeviceDriverContext *USBDeluxe_DeviceGetDriverContext(uint8_t index);

extern void USBDeluxe_Device_Tasks();

extern uint8_t USBDeluxe_DeviceDescriptor_InsertInterface(uint8_t alternate_setting, uint8_t nr_endpoints, uint8_t class, uint8_t subclass, uint8_t protocol);
extern void USBDeluxe_DeviceDescriptor_InsertEndpointRaw(uint8_t addr, uint8_t attr, uint16_t size, uint8_t interval,
							 int16_t opt_refresh, int16_t opt_sync_addr);
extern uint8_t USBDeluxe_DeviceDescriptor_InsertEndpoint(uint8_t direction, uint8_t attr, uint16_t size, uint8_t interval,
							 int16_t opt_refresh, int16_t opt_sync_addr);
extern void USBDeluxe_DeviceDescriptor_InsertIAD(uint8_t iface, uint8_t nr_contiguous_ifaces, uint8_t class, uint16_t subclass, uint8_t protocol);
extern void USBDeluxe_DeviceDescriptor_InsertCDCSpecific(uint8_t comm_iface, uint8_t data_iface);

extern uint8_t USBDeluxe_DeviceFunction_Add_CDC(void *userp, USBDeluxeDevice_CDC_IOps *io_ops);