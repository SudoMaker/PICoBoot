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

/** INCLUDES *******************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "usb_device.h"

#include "usb_deluxe_device.h"


/*******************************************************************
 * Function:        bool USB_Device_EventHandler(
 *                        USB_EVENT event, void *pdata, uint16_t size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  uint16_t size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
bool USB_Device_EventHandler(USB_EVENT event, void *pdata, uint16_t size) {
	switch( (int) event )
	{
		case EVENT_TRANSFER:
			break;

		case EVENT_SOF:
			break;

		case EVENT_SUSPEND:
			//Call the hardware platform specific handler for suspend events for
			//possible further action (like optionally going reconfiguring the application
			//for lower power states and going to sleep during the suspend event).  This
			//would normally be done in USB compliant bus powered applications, although
			//no further processing is needed for purely self powered applications that
			//don't consume power from the host.
			break;

		case EVENT_RESUME:
			//Call the hardware platform specific resume from suspend handler (ex: to
			//restore I/O pins to higher power states if they were changed during the
			//preceding SYSTEM_Initialize(SYSTEM_STATE_USB_SUSPEND) call at the start
			//of the suspend condition.
			break;

		case EVENT_CONFIGURED:
			USBDeluxe_Device_EventInit();

			break;

		case EVENT_SET_DESCRIPTOR:
			break;

		case EVENT_EP0_REQUEST:
			/* We have received a non-standard USB request.  The CDC driver
			 * needs to check to see if the request was for it. */

			USBDeluxe_Device_EventCheckRequest();

			break;

		case EVENT_BUS_ERROR:
			break;

		case EVENT_TRANSFER_TERMINATED:
			break;

		default:
			break;
	}
	return true;
}

//void __attribute__((interrupt,auto_psv)) _USB1Interrupt()
//{
//	USBDeviceTasks();
//}
