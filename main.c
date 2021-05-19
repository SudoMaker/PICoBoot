/*
    This file is part of PICoBoot.

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

#include <PICoBoot/PICoBoot.h>

uint16_t CDC_RxDone(void *userp, uint8_t *buf, uint16_t len) {
	LATFbits.LATF0 = 1;

	PicoBoot_CDC_ParseIncomingData(buf, len);

	return len;
}

USBDeluxeDevice_CDC_IOps cdc_iops = {
	.RxDone = CDC_RxDone
};

int main() {
	PICoBoot_Board_PinInitialize();

	if (RCONbits.POR) {
		RCONbits.POR = 0;
	} else {
		PICoBoot_RuntimeEnvironment_Load();

		picoboot_runtime_env.reset_count++;
		picoboot_runtime_env.boot_reason = BootReason_RESET;

		if (RCONbits.WDTO) {
			RCONbits.WDTO = 0;
			picoboot_runtime_env.boot_reason = BootReason_WDT;
			picoboot_runtime_env.watchdog_failed_count++;
		}
	}

	PICoBoot_StaticEnvironment_Load();

	int boot_action = PICoBoot_Board_Reset_Action();

	PICoBoot_RuntimeEnvironment_Save();

	if (boot_action == ResetAction_RunUserApp) {
		PicoBoot_StartUserApp(PICoBoot_App_Address);
	} else {
		PICoBoot_Board_SystemInitialize();

		USBDeluxe_Device_ConfigInit();
		USBDeluxe_DeviceFunction_Add_CDC(NULL, &cdc_iops);
		USBDeluxe_Device_ConfigApply();
		USBDeluxe_SetRole(USB_ROLE_DEVICE);

		PICoBoot_LED_1 = 1;

		uint32_t cnt = 0;

		while (1) {
			if (cnt % 32768 == 0) {
				PICoBoot_LED_1 = !PICoBoot_LED_1;
			}

			PicoBoot_Tasks();
			USBDeviceTasks();
			USBDeluxe_Device_Tasks();

			cnt++;
		}
	}

	return 1;
}
