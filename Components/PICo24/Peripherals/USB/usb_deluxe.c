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

#include "usb_deluxe.h"

#include "Device/usb_deluxe_device.h"

#include <stdio.h>

USBDeluxe_Role usb_deluxe_role = USB_ROLE_NONE;

void USBDeluxe_Tasks() {
	if (usb_deluxe_role == USB_ROLE_DEVICE)
		USBDeluxe_Device_Tasks();
}

void USBDeluxe_SetRole(uint8_t role) {
	if (usb_deluxe_role != role) {
		USBDisableInterrupts();
		if (role == USB_ROLE_DEVICE) {
			USBDeviceInit();
			USBDeviceAttach();
		}
		usb_deluxe_role = role;
		USBEnableInterrupts();
	}
}

void __attribute__((interrupt,auto_psv)) _USB1Interrupt() {
	if (usb_deluxe_role == USB_ROLE_DEVICE)
		USBDeviceTasks();
	else {
		USBClearUSBInterrupt();
		USBDisableInterrupts();
	}
}