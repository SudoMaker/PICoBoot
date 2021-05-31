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

int PICoBoot_CheckProhibitedRange(uint32_t addr) {
	for (size_t i=0; i< sizeof(PICoBoot_FlashProhibitedRanges) / 4; i+=2) {
		if (addr >= PICoBoot_FlashProhibitedRanges[i] && addr <= PICoBoot_FlashProhibitedRanges[i + 1]) {
			return 1;
		}
	}

	return 0;
}

void PICoBoot_FlashEraseRange(uint32_t addr, uint32_t len) {
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

uint8_t PICoBoot_FlashWriteRaw(uint32_t addr, const uint8_t *buf) {
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

		return FlasherResult_OK;
	} else {
		return FlasherResult_ERANGE;
	}
}

uint8_t PICoBoot_FlashWrite(size_t len) {
	if (picoboot_static_env_por.allow_write != PICoBoot_ENVAR_TRUE) {
		return FlasherResult_EPERM;
	}

	if (crc8(protocol_ctx.buffer, len) != protocol_ctx.buffer[len]) {
		return FlasherResult_CRC_Error;
	}

	uint8_t rc = FlasherResult_OK;
	for (size_t i=0; i<len; i+=4) {
		rc = PICoBoot_FlashWriteRaw(protocol_ctx.current_address, protocol_ctx.buffer+i);
		if (rc != FlasherResult_OK && rc != FlasherResult_ERANGE) {
			return rc;
		}
		protocol_ctx.current_address += 2;
	}

	return rc;
}
