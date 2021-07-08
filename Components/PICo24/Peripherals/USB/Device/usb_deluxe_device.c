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

extern USBDeluxeDevice_CDCACMContext cdc_ctx;

void USBDeluxe_Device_Init() {
	USBDeviceInit();
	USBDeviceAttach();
}

void USBDeluxe_Device_EventInit() {
	USBDeluxeDevice_CDC_ACM_Init(&cdc_ctx);
}

void USBDeluxe_Device_EventCheckRequest() {
	USBDeluxeDevice_CDC_ACM_CheckRequest(&cdc_ctx);
}

void USBDeluxe_Device_Tasks() {
	if (USBGetDeviceState() < CONFIGURED_STATE) {
		return;
	}

	if (USBIsDeviceSuspended()== true) {
		return;
	}

	USBDeluxeDevice_CDC_ACM_Tasks(&cdc_ctx);
}
