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

int PICoBoot_StaticEnvironment_InRange(uint32_t addr) {
	if ((addr >= PICoBoot_StaticEnvironment_Address) && (addr < (PICoBoot_StaticEnvironment_Address + sizeof(PicoBootStaticEnvironment)))) {
		return 1;
	} else {
		return 0;
	}
}

uint32_t PICoBoot_StaticEnvironment_GetWord(uint32_t offset) {
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