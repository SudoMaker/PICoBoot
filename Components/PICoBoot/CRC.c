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