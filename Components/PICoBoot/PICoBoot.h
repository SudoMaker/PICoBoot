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
	uint8_t serial[16];
	uint32_t checksum;
} __attribute__((packed)) PicoBootStaticEnvironment;

typedef struct {
	uint32_t reset_count;
	uint32_t watchdog_failed_count;
	uint8_t boot_reason;
	uint8_t reboot_target;
} PicoBootRuntimeEnvironment;

typedef union {
	uint8_t u8;
	uint16_t u16;
	uint32_t u32;
	uint64_t u64;
} VariantInt;

typedef struct {
	uint8_t current_command;
	uint8_t variable_command_length;

	uint8_t buffer[32];
	uint8_t buffer_pos;

	uint32_t current_address;
	uint16_t current_length;

	uint8_t read_command;

} PICoBootProtocolContext;

extern const char PICoBoot_Version[];
extern const int8_t flasher_cmd_arg_length_table[];

extern PicoBootStaticEnvironment picoboot_static_env;
extern PicoBootStaticEnvironment picoboot_static_env_por;
extern PicoBootRuntimeEnvironment picoboot_runtime_env;

extern PICoBootProtocolContext protocol_ctx;

// CRC
extern uint8_t crc8(const uint8_t *data, size_t len);
extern uint32_t crc32(uint8_t *data, uint16_t size);

// FlashOps
extern int PICoBoot_CheckProhibitedRange(uint32_t addr);
extern void PICoBoot_FlashEraseRange(uint32_t addr, uint32_t len);
extern uint8_t PICoBoot_FlashWriteRaw(uint32_t addr, const uint8_t *buf);
extern uint8_t PICoBoot_FlashWrite(size_t len);

// Environment
extern void PICoBoot_StaticEnvironment_Checksum_Gen(PicoBootStaticEnvironment *env);
extern int PICoBoot_StaticEnvironment_Checksum_Verify(PicoBootStaticEnvironment *env);
extern void PICoBoot_StaticEnvironment_Load();
extern int PICoBoot_StaticEnvironment_InRange(uint32_t addr);
extern uint32_t PICoBoot_StaticEnvironment_GetWord(uint16_t offset);
extern void PICoBoot_StaticEnvironment_Save();

// Protocol
extern int PICoBoot_CmdBuffer_Push(uint8_t data);
extern void PICoBoot_CmdBuffer_Clear();
extern void PICoBoot_CommandInvoke(uint8_t cmd, uint8_t *arg, uint8_t arg_len);
extern void PICoBoot_Protocol_DoReads();
extern void PicoBoot_Protocol_ParseIncomingData(const uint8_t *buf, uint16_t len);

// Main
extern void PicoBoot_Tasks();
extern void PicoBoot_StartUserApp(uint16_t addr);

