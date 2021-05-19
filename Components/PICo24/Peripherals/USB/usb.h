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

/* This file contains code originally written by Microchip Technology Inc., which was licensed under the Apache License. */


/*******************************************************************************
 Module for Microchip USB Library

  Company:
    Microchip Technology Inc.

  File Name:
    usb.h

  Summary:
    This header file exposes the core library APIs and definitions for the USB
    library.

  Description:
    This header file exposes the core library APIs and definitions for the USB
    library.  The user is responsible for also including the header file for
    the specific driver they will be using.
*******************************************************************************/

#pragma once

#include <errno.h>
#include <stdio.h>

#include <PICo24/Core/IDESupport.h>

#include "usb_common.h"         // Common USB library definitions
#include "usb_ch9.h"          // USB device framework definitions
#include "usb_hal.h"            // Hardware Abstraction Layer interface

#include "Device/usb_device.h"     // USB Device abstraction layer interface
#include "Device/usb_device_config.h"
#include "Device/usb_deluxe_device.h"

/* USB Library version number.  This can be used to verify in an application 
   specific version of the library is being used.
 */
#define USB_MAJOR_VER   2        // Firmware version, major release number.
#define USB_MINOR_VER   13       // Firmware version, minor release number.
#define USB_DOT_VER     1        // Firmware version, dot release number.





