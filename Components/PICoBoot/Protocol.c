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

static int addr_within_range(void *src, void *dest, size_t dest_size) {
	if (src >= dest && src < (dest+dest_size)) {
		return 1;
	} else {
		return 0;
	}
}

int PICoBoot_CmdBuffer_Push(uint8_t data) {
	protocol_ctx.buffer[protocol_ctx.buffer_pos] = data;
	protocol_ctx.buffer_pos++;
	return protocol_ctx.buffer_pos;
}

void PICoBoot_CmdBuffer_Clear() {
	protocol_ctx.buffer_pos = 0;
	protocol_ctx.current_command = 0;
}

void PICoBoot_CommandSendReturnData(uint8_t *arg, uint8_t arg_len) {
	USBDeluxeDevice_CDC_Write(&USBDeluxe_DeviceGetDriverContext(0)->cdc, arg, arg_len);
}

void PICoBoot_CommandInvoke(uint8_t cmd, uint8_t *arg, uint8_t arg_len) {
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


void PICoBoot_Protocol_DoReads() {
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

void PicoBoot_Protocol_ParseIncomingData(const uint8_t *buf, uint16_t len) {
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

