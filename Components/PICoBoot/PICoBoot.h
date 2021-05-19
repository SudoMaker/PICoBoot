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

#pragma once

#include <xc.h>

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>

#include <PICo24/PICo24.h>

#include <PICoBoot_Board.h>

enum PicoBootResetAction {
	ResetAction_RunUserApp,
	ResetAction_StayInBL,
};

enum PicoBootCommand {
	FlasherCommand_Reset = 1,

	FlasherCommand_GetBootloaderVersion,
	FlasherCommand_GetBoardName,
	FlasherCommand_GetBoardManufacturer,
	FlasherCommand_GetChipName,
	FlasherCommand_GetChipManufacturer,

	FlasherCommand_FlashSetAddr,
	FlasherCommand_FlashSetLength,
	FlasherCommand_FlashErase,
	FlasherCommand_FlashRead,
	FlasherCommand_FlashWrite4,
	FlasherCommand_FlashWrite8,
	FlasherCommand_FlashWrite16,

	FlasherCommand_EnvironmentRead,
	FlasherCommand_EnvironmentWrite,
	FlasherCommand_EnvironmentSave,
};

enum PicoBootFlashEraseMode {
	FlashEraseMode_All = 0,
	FlashEraseMode_App = 1,
};

enum PicoBootResults {
	FlasherResult_OK = 0,
	FlasherResult_CRC_Error = 1,
	FlasherResult_Verify_Error = 2,
	FlasherResult_EPERM = 3,
	FlasherResult_ERANGE = 4,
};

enum PicoBootBootReason {
	BootReason_POR = 0,
	BootReason_RESET = 1,
	BootReason_WDT = 2,
};

enum PicoBootRebootTarget {
	RebootTarget_Default = 0,
	RebootTarget_Bootloader = 1,
};

typedef struct {
	uint8_t env_initialized;
	uint8_t watchdog_config;
	uint8_t watchdog_fails_to_enter_bl;
	uint8_t _;
	uint32_t allow_read, allow_write;
	uint8_t app_name[16], app_version[8];
	uint32_t checksum;
} __attribute__((packed)) PicoBootStaticEnvironment;

typedef struct {
	uint32_t reset_count;
	uint32_t watchdog_failed_count;
	uint8_t boot_reason;
	uint8_t reboot_target;
} PicoBootRuntimeEnvironment;

extern PicoBootStaticEnvironment picoboot_static_env;
extern PicoBootStaticEnvironment picoboot_static_env_por;
extern PicoBootRuntimeEnvironment picoboot_runtime_env;

extern void PicoBoot_Tasks();
extern void PicoBoot_CDC_ParseIncomingData(const uint8_t *buf, uint16_t len);
extern void PicoBoot_StartUserApp(uint16_t addr);
extern void PICoBoot_StaticEnvironment_Load();
