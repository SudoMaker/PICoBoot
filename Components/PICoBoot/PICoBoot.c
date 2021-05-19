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

#include "PICoBoot.h"

static const char PICoBoot_Version[] = "v0.01";

static const int8_t flasher_cmd_arg_length_table[] = {
	-1,	// 0: FlasherCommand_Unknown

	1,	// void FlasherCommand_Reset(uint8_t mode)

	0,	// char[32] FlasherCommand_GetBootloaderVersion()
	0,	// char[32] FlasherCommand_GetBoardName()
	0,	// char[32] FlasherCommand_GetBoardManufacturer()
	0,	// char[32] FlasherCommand_GetChipName()
	0,	// char[32] FlasherCommand_GetChipManufacturer()

	4,	// uint32_t FlasherCommand_FlashSetAddr(uint32_t addr)
	2,	// uint16_t FlasherCommand_FlashSetLength(uint16_t len)
	1,	// Result FlasherCommand_FlashErase(uint8_t mode)
	0,	// uint8_t[] FlasherCommand_FlashRead()
	5,	// Result FlasherCommand_FlashWrite4(uint8_t[4], uint8_t crc8)
	9,	// Result FlasherCommand_FlashWrite8(uint8_t[8], uint8_t crc8)
	17,	// Result FlasherCommand_FlashWrite8(uint8_t[16], uint8_t crc8)


	1,	// char[64] FlasherCommand_EnvironmentRead(uint8_t category)
	4,	// Result FlasherCommand_EnvironmentWrite(uint8_t category, uint8_t offset, uint8_t value, uint8_t crc)
	1,	// Result FlasherCommand_EnvironmentSave(uint8_t category)
};

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

PICoBootProtocolContext protocol_ctx = {0};

PicoBootStaticEnvironment picoboot_static_env = {0};
PicoBootStaticEnvironment picoboot_static_env_por = {0};
PicoBootRuntimeEnvironment picoboot_runtime_env = {0};


uint8_t crc8(const uint8_t *data, size_t len) {
	uint8_t crc = 0xff;
	size_t i, j;
	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (j = 0; j < 8; j++) {
			if ((crc & 0x80) != 0)
				crc = (uint8_t)((crc << 1) ^ 0x31);
			else
				crc <<= 1;
		}
	}

	return crc;
}

uint32_t crc32(uint8_t *data, uint16_t size) {
	uint32_t r = ~0;
	uint8_t *end = data + size;

	while (data < end) {
		r ^= *data++;

		for (int i = 0; i < 8; i++) {
			uint32_t t = ~((r&1) - 1);
			r = (r>>1) ^ (0xEDB88320 & t);
		}
	}

	return ~r;
}

static int PICoBoot_CmdBuffer_Push(uint8_t data) {
	protocol_ctx.buffer[protocol_ctx.buffer_pos] = data;
	protocol_ctx.buffer_pos++;
	return protocol_ctx.buffer_pos;
}

static void PICoBoot_CmdBuffer_Clear() {
	protocol_ctx.buffer_pos = 0;
	protocol_ctx.current_command = 0;
}

static void PICoBoot_CommandSendReturnData(uint8_t *arg, uint8_t arg_len) {
	USBDeluxeDevice_CDC_Write(&USBDeluxe_DeviceGetDriverContext(0)->cdc, arg, arg_len);
}

void PICoBoot_StaticEnvironment_Checksum_Gen(PicoBootStaticEnvironment *env) {
	uint16_t env_len = sizeof(PicoBootStaticEnvironment);
	uint16_t env_data_len = env_len - sizeof(env->checksum);

	env->checksum = crc32((uint8_t *) env, env_data_len);
}

int PICoBoot_StaticEnvironment_Checksum_Verify(PicoBootStaticEnvironment *env) {
	uint16_t env_len = sizeof(PicoBootStaticEnvironment);
	uint16_t env_data_len = env_len - sizeof(env->checksum);

	return env->checksum != crc32((uint8_t *) env, env_data_len);
}

void PICoBoot_StaticEnvironment_Load() {
	uint8_t *p = (uint8_t *)&picoboot_static_env;
	protocol_ctx.current_address = PICoBoot_StaticEnvironment_Address;

	for (size_t i=0; i<sizeof(PicoBootStaticEnvironment); i+=2) {
		uint32_t r = PICoBoot_Board_Flash_ReadInstruction(protocol_ctx.current_address);
		p[i] = r & 0xff;
		p[i+1] = (r >> 8) & 0xff;
		protocol_ctx.current_address += 2;
	}

	protocol_ctx.current_address = 0;

	if (picoboot_static_env.env_initialized == PICoBoot_ENVAR_UNINIT) {
		picoboot_static_env.env_initialized = 1;
		picoboot_static_env.watchdog_config = 0;
		picoboot_static_env.watchdog_fails_to_enter_bl = 0;
		picoboot_static_env.allow_read = PICoBoot_ENVAR_TRUE;
		picoboot_static_env.allow_write = PICoBoot_ENVAR_TRUE;
		picoboot_static_env.app_name[0] = 0;
		picoboot_static_env.app_version[0] = 0;

		PICoBoot_StaticEnvironment_Checksum_Gen(&picoboot_static_env);
	}

	if (PICoBoot_StaticEnvironment_Checksum_Verify(&picoboot_static_env)) { // Failed
		picoboot_static_env.allow_read = PICoBoot_ENVAR_FALSE;
		picoboot_static_env.allow_write = PICoBoot_ENVAR_FALSE;
	}

	memcpy(&picoboot_static_env_por, &picoboot_static_env, sizeof(PicoBootStaticEnvironment));
}

static int PICoBoot_StaticEnvironment_InRange(uint32_t addr) {
	if ((addr >= PICoBoot_StaticEnvironment_Address) && (addr < (PICoBoot_StaticEnvironment_Address + sizeof(PicoBootStaticEnvironment)))) {
		return 1;
	} else {
		return 0;
	}
}

static uint32_t PICoBoot_StaticEnvironment_GetWord(uint32_t offset) {
	uint32_t val = 0;

	uint8_t *p = ((uint8_t *)&picoboot_static_env) + offset;

	val |= p[0];
	val |= ((uint32_t) p[1]) << 8;

	return val;
}

void PICoBoot_StaticEnvironment_Save() {
	uint32_t page = PICoBoot_StaticEnvironment_Address / PICoBoot_Flash_PageSize;
	uint32_t pagesize_dwords = PICoBoot_Flash_PageSize / 2;
	uint32_t page_address = page * PICoBoot_Flash_PageSize;
	protocol_ctx.current_address = page_address;

	uint32_t page_contents[pagesize_dwords];

	// Read entire page into memory
	for (size_t i=0; i<pagesize_dwords; i++) {
		page_contents[i] = PICoBoot_Board_Flash_ReadInstruction(protocol_ctx.current_address);
		protocol_ctx.current_address += 2;
	}

	// Erase this page
	PICoBoot_Board_Flash_ErasePage(page_address);

	// Recalc env checksum
	PICoBoot_StaticEnvironment_Checksum_Gen(&picoboot_static_env);

	// Rewrite this page
	protocol_ctx.current_address = page_address;
	for (size_t i=0; i<pagesize_dwords; i++) {
		uint32_t val;

		if (PICoBoot_StaticEnvironment_InRange(protocol_ctx.current_address)) {
			val = PICoBoot_StaticEnvironment_GetWord(protocol_ctx.current_address - PICoBoot_StaticEnvironment_Address);
		} else {
			val = page_contents[i];
		}

		PICoBoot_Board_Flash_WriteInstruction(protocol_ctx.current_address, val);

		protocol_ctx.current_address += 2;
	}

	protocol_ctx.current_address = 0;
}

static int PICoBoot_CheckProhibitedRange(uint32_t addr) {
	for (size_t i=0; i< sizeof(PICoBoot_FlashProhibitedRanges) / 4; i+=2) {
		if (addr >= PICoBoot_FlashProhibitedRanges[i] && addr <= PICoBoot_FlashProhibitedRanges[i + 1]) {
			return 1;
		}
	}

	return 0;
}

static int addr_within_range(void *src, void *dest, size_t dest_size) {
	if (src >= dest && src < (dest+dest_size)) {
		return 1;
	} else {
		return 0;
	}
}

static void PICoBoot_FlashEraseRange(uint32_t addr, uint32_t len) {
	for (uint32_t i=0; i<len; i+=2) {
		protocol_ctx.current_address = addr + i;

		if (!PICoBoot_CheckProhibitedRange(protocol_ctx.current_address)) {
			if (protocol_ctx.current_address % PICoBoot_Flash_PageSize == 0) { // On page boundary
				// Erase it first
				PICoBoot_Board_Flash_ErasePage(protocol_ctx.current_address);

				// Rewrite reset vector on 0x0
				if (protocol_ctx.current_address == 0) {
					// GOTO at 0x0
					PICoBoot_Board_Flash_WriteInstruction(protocol_ctx.current_address, 0x00040000 | PICoBoot_Bootloader_Address);
					// Reset address at 0x2
					PICoBoot_Board_Flash_WriteInstruction(protocol_ctx.current_address + 2, 0x00000000);
				}
			}
		}
	}

	protocol_ctx.current_address = 0;
}

static uint8_t PICoBoot_FlashWriteRaw(uint32_t addr, const uint8_t *buf) {
	if (!PICoBoot_CheckProhibitedRange(addr)) {
		if (addr % PICoBoot_Flash_PageSize == 0) { // On page boundary
			// Erase it first
			PICoBoot_Board_Flash_ErasePage(addr);

			// Rewrite reset vector on 0x0
			if (addr == 0) {
				// GOTO at 0x0
				PICoBoot_Board_Flash_WriteInstruction(addr, 0x00040000 | PICoBoot_Bootloader_Address);
				// Reset address at 0x2
				PICoBoot_Board_Flash_WriteInstruction(addr + 2, 0x00000000);
			}

			// Rewrite environment if needed
			if ((addr / PICoBoot_Flash_PageSize) == (PICoBoot_StaticEnvironment_Address / PICoBoot_Flash_PageSize)) {
				for (size_t i=0; i<sizeof(PicoBootStaticEnvironment); i+=2) {
					uint32_t val = PICoBoot_StaticEnvironment_GetWord(i);
					PICoBoot_Board_Flash_WriteInstruction(PICoBoot_StaticEnvironment_Address + i, val);
				}
			}
		}

		uint32_t val = 0;

		if (addr == 0) {

		} else if (addr == 2) {

		} else if (PICoBoot_StaticEnvironment_InRange(addr)) {

		} else {
			val |= buf[0];
			val |= ((uint32_t) buf[1]) << 8;
			val |= ((uint32_t) buf[2]) << 16;
			val |= ((uint32_t) buf[3]) << 24;

			PICoBoot_Board_Flash_WriteInstruction(addr, val);
			uint32_t val_verify = PICoBoot_Board_Flash_ReadInstruction(addr);

			if (val_verify != val) {
				return FlasherResult_Verify_Error;
			}
		}
	}

	return FlasherResult_OK;
}

static uint8_t PICoBoot_FlashWrite(size_t len) {
	if (picoboot_static_env_por.allow_write != PICoBoot_ENVAR_TRUE) {
		return FlasherResult_EPERM;
	}

	if (crc8(protocol_ctx.buffer, len) != protocol_ctx.buffer[len]) {
		return FlasherResult_CRC_Error;
	}

	for (size_t i=0; i<len; i+=4) {
		uint8_t rc = PICoBoot_FlashWriteRaw(protocol_ctx.current_address, protocol_ctx.buffer+i);
		if (rc != FlasherResult_OK) {
			return rc;
		}
		protocol_ctx.current_address += 2;
	}

	return FlasherResult_OK;
}

static void PICoBoot_CommandInvoke(uint8_t cmd, uint8_t *arg, uint8_t arg_len) {
	PICoBoot_LED_2 = 1;

	VariantInt v;

	if (arg_len <= 8)
		memcpy(&v, arg, arg_len);

	switch (cmd) {
		case FlasherCommand_Reset:
			picoboot_runtime_env.reboot_target = v.u8;
			PICoBoot_RuntimeEnvironment_Save();
			asm("reset");
			break;
		case FlasherCommand_GetBootloaderVersion:
		case FlasherCommand_GetBoardName:
		case FlasherCommand_GetBoardManufacturer:
		case FlasherCommand_GetChipName:
		case FlasherCommand_GetChipManufacturer:
		case FlasherCommand_EnvironmentRead:
			protocol_ctx.read_command = cmd;
			break;
		case FlasherCommand_EnvironmentWrite: {
			uint8_t rc = FlasherResult_OK;

			if (crc8(protocol_ctx.buffer, 3) != protocol_ctx.buffer[3]) {
				rc = FlasherResult_CRC_Error;
				goto error_envwrite;
			}

			uint8_t cat = protocol_ctx.buffer[0];
			uint8_t off = protocol_ctx.buffer[1];
			uint8_t val = protocol_ctx.buffer[2];

			uint16_t max_offset = 0;
			uint8_t *abs_addr = NULL;

			if (cat == 0) { // Static
				abs_addr = ((uint8_t *)&picoboot_static_env) + off;
				max_offset = sizeof(PicoBootStaticEnvironment) - 1;

				if (addr_within_range(abs_addr, &picoboot_static_env.allow_read, 4)) {
					if (picoboot_static_env_por.allow_read != PICoBoot_ENVAR_TRUE) {
						rc = FlasherResult_EPERM;
						goto error_envwrite;
					}
				} else if (addr_within_range(abs_addr, &picoboot_static_env.allow_write, 4)) {
					if (picoboot_static_env_por.allow_write != PICoBoot_ENVAR_TRUE) {
						rc = FlasherResult_EPERM;
						goto error_envwrite;
					}
				}
			} else { // Runtime
				abs_addr = ((uint8_t *)&picoboot_runtime_env) + off;
				max_offset = sizeof(PicoBootRuntimeEnvironment) - 1;

				if (
					addr_within_range(abs_addr, &picoboot_runtime_env.boot_reason, 1)
					|| addr_within_range(abs_addr, &picoboot_runtime_env.reset_count, 1)
					|| addr_within_range(abs_addr, &picoboot_runtime_env.watchdog_failed_count, 1)
					) {
					rc = FlasherResult_EPERM;
					goto error_envwrite;
				}
			}

			if (off > max_offset) {
				rc = FlasherResult_ERANGE;
				goto error_envwrite;
			}

			*abs_addr = val;
error_envwrite:
			PICoBoot_CommandSendReturnData(&rc, 1);
		}
			break;
		case FlasherCommand_EnvironmentSave: {
			uint8_t rc = 0;

			if (v.u8 & 0x1) { // Static
				PICoBoot_StaticEnvironment_Save();
			}

			if (v.u8 & 0x2) { // Runtime
				PICoBoot_RuntimeEnvironment_Save();
			}

			PICoBoot_CommandSendReturnData(&rc, 1);
		}
		case FlasherCommand_FlashSetAddr:
			protocol_ctx.current_address = v.u32 / 2;
			PICoBoot_CommandSendReturnData(arg, arg_len);
			break;
		case FlasherCommand_FlashSetLength:
			protocol_ctx.current_length = v.u16;
			PICoBoot_CommandSendReturnData(arg, arg_len);
			break;
		case FlasherCommand_FlashRead:
			protocol_ctx.read_command = cmd;
			break;
		case FlasherCommand_FlashErase: {
			uint8_t rc = FlasherResult_OK;
			uint8_t mode = v.u8;

			if (mode == FlashEraseMode_App) {
				if (picoboot_static_env_por.allow_write != PICoBoot_ENVAR_TRUE) {
					rc = FlasherResult_EPERM;
					goto error_flasherase;
				} else {
					PICoBoot_FlashEraseRange(PICoBoot_App_Address, PICoBoot_App_Size);
				}
			} else if (mode == FlashEraseMode_All) {
				// Erase user app first, you know...
				PICoBoot_FlashEraseRange(PICoBoot_App_Address, PICoBoot_App_Size);
				memset(&picoboot_static_env, 0xff, sizeof(PicoBootStaticEnvironment));
				picoboot_static_env.env_initialized = PICoBoot_ENVAR_UNINIT;
				PICoBoot_StaticEnvironment_Save();

				picoboot_runtime_env.reboot_target = RebootTarget_Bootloader;
				PICoBoot_RuntimeEnvironment_Save();
				asm("reset");
			}
error_flasherase:
			PICoBoot_CommandSendReturnData(&rc, 1);
		}
			break;
		case FlasherCommand_FlashWrite4: {
			uint8_t rc = PICoBoot_FlashWrite(4);
			PICoBoot_CommandSendReturnData(&rc, 1);
		}
			break;
		case FlasherCommand_FlashWrite8: {
			uint8_t rc = PICoBoot_FlashWrite(8);
			PICoBoot_CommandSendReturnData(&rc, 1);
		}
			break;
		case FlasherCommand_FlashWrite16: {
			uint8_t rc = PICoBoot_FlashWrite(16);
			PICoBoot_CommandSendReturnData(&rc, 1);
		}
			break;
		default:
			break;
	}

	PICoBoot_LED_2 = 0;
}


static void PICoBoot_DoReads() {
	if (!protocol_ctx.read_command)
		return;

	PICoBoot_LED_2 = 1;

	switch (protocol_ctx.read_command) {
		case FlasherCommand_GetBootloaderVersion: {
			char name[32];
			strncpy(name, PICoBoot_Version, 31);
			USBDeluxeDevice_CDC_Write(&USBDeluxe_DeviceGetDriverContext(0)->cdc, (uint8_t *)name, sizeof(name));
			protocol_ctx.read_command = 0;
		}
			break;
		case FlasherCommand_GetBoardName: {
			char name[32];
			strncpy(name, PICoBoot_Board, 31);
			USBDeluxeDevice_CDC_Write(&USBDeluxe_DeviceGetDriverContext(0)->cdc, (uint8_t *)name, sizeof(name));
			protocol_ctx.read_command = 0;
		}
			break;
		case FlasherCommand_GetBoardManufacturer: {
			char name[32];
			strncpy(name, PICoBoot_BoardManufacturer, 31);
			USBDeluxeDevice_CDC_Write(&USBDeluxe_DeviceGetDriverContext(0)->cdc, (uint8_t *)name, sizeof(name));
			protocol_ctx.read_command = 0;
		}
			break;
		case FlasherCommand_GetChipName: {
			char name[32];
			strncpy(name, PICoBoot_Chip, 31);
			USBDeluxeDevice_CDC_Write(&USBDeluxe_DeviceGetDriverContext(0)->cdc, (uint8_t *)name, sizeof(name));
			protocol_ctx.read_command = 0;
		}
			break;
		case FlasherCommand_GetChipManufacturer: {
			char name[32];
			strncpy(name, PICoBoot_ChipManufacturer, 31);
			USBDeluxeDevice_CDC_Write(&USBDeluxe_DeviceGetDriverContext(0)->cdc, (uint8_t *)name, sizeof(name));
			protocol_ctx.read_command = 0;
		}
			break;
		case FlasherCommand_EnvironmentRead: {
			char env[64];
			if (protocol_ctx.buffer[0] == 0) {
				memcpy(env, &picoboot_static_env, sizeof(PicoBootStaticEnvironment));
			} else {
				memcpy(env, &picoboot_runtime_env, sizeof(PicoBootRuntimeEnvironment));
			}
			USBDeluxeDevice_CDC_Write(&USBDeluxe_DeviceGetDriverContext(0)->cdc, (uint8_t *)env, sizeof(env));
			protocol_ctx.read_command = 0;
		}
			break;
		case FlasherCommand_FlashRead: {
			uint32_t proc_len = protocol_ctx.current_length;
			if (proc_len > USBDeluxe_CDC_BUF_SIZE) {
				proc_len = USBDeluxe_CDC_BUF_SIZE;
			}

			USBDeluxeDevice_CDC_UserBuffer user_buf;

			if (USBDeluxeDevice_CDC_AcquireTxBuffer(&USBDeluxe_DeviceGetDriverContext(0)->cdc, &user_buf) == 0) {
				for (uint32_t j = 0; j < proc_len; j+=4) {
					if (PICoBoot_CheckProhibitedRange(protocol_ctx.current_address) || picoboot_static_env_por.allow_read != PICoBoot_ENVAR_TRUE) {
						user_buf.buf[j] = 'c';
						user_buf.buf[j + 1] = 'a';
						user_buf.buf[j + 2] = 'f';
						user_buf.buf[j + 3] = 'e';
					} else {
						uint32_t r = PICoBoot_Board_Flash_ReadInstruction(protocol_ctx.current_address);

						user_buf.buf[j] = r & 0xff;
						user_buf.buf[j + 1] = (r >> 8) & 0xff;
						user_buf.buf[j + 2] = (r >> 16) & 0xff;
						user_buf.buf[j + 3] = (r >> 24) & 0xff;
					}

					protocol_ctx.current_address += 2;
					protocol_ctx.current_length -= 4;
				}

				*user_buf.buf_len = proc_len;
			}

			if (protocol_ctx.current_length == 0) {
				protocol_ctx.read_command = 0;
			}
		}
			break;

		default:
			break;
	}

	PICoBoot_LED_2 = 0;

}

void PicoBoot_CDC_ParseIncomingData(const uint8_t *buf, uint16_t len) {
	for (uint16_t i=0; i<len; i++) {
		uint8_t cur_byte = buf[i];

		if (!protocol_ctx.current_command) {
			if (flasher_cmd_arg_length_table[cur_byte] == 0) {
				PICoBoot_CommandInvoke(cur_byte, NULL, 0);
			} else if (flasher_cmd_arg_length_table[cur_byte] == -1) {
				// Do nothing
			} else {
				protocol_ctx.current_command = cur_byte;
			}
		} else {
			if (PICoBoot_CmdBuffer_Push(cur_byte) == flasher_cmd_arg_length_table[protocol_ctx.current_command]) {
				PICoBoot_CommandInvoke(protocol_ctx.current_command, &protocol_ctx.buffer[0], protocol_ctx.buffer_pos);
				PICoBoot_CmdBuffer_Clear();
			}
		}
	}
}



void PicoBoot_StartUserApp(uint16_t addr) {
	asm("goto w0");
}

void PicoBoot_Tasks() {
	PICoBoot_DoReads();
}