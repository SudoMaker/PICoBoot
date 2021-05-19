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

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include <PICo24/PICo24.h>

#define PICoBoot_ENVAR_UNINIT		0xef
#define PICoBoot_ENVAR_TRUE		0x55555555
#define PICoBoot_ENVAR_FALSE		0xaaaaaaaa

#define PICoBoot_LED_1			LATFbits.LATF0
#define PICoBoot_LED_2			LATFbits.LATF1

extern const char PICoBoot_BoardManufacturer[];
extern const char PICoBoot_Board[];
extern const char PICoBoot_ChipManufacturer[];
extern const char PICoBoot_Chip[];

extern void PICoBoot_Board_PinInitialize();
extern void PICoBoot_Board_SystemInitialize();

extern const uint32_t PICoBoot_Bootloader_Address;
extern const uint32_t PICoBoot_App_Address;
extern const uint32_t PICoBoot_App_Size;
extern const uint32_t PICoBoot_StaticEnvironment_Address;
extern const uint32_t PICoBoot_FlashProhibitedRanges[4];
extern const uint16_t PICoBoot_Flash_PageSize;

extern int PICoBoot_Board_Reset_Action();
extern uint32_t PICoBoot_Board_Flash_ReadInstruction(uint32_t addr);
extern void PICoBoot_Board_Flash_ErasePage(uint32_t addr);
extern void PICoBoot_Board_Flash_WriteInstruction(uint32_t addr, uint32_t value);

extern void PICoBoot_RuntimeEnvironment_Load();
extern void PICoBoot_RuntimeEnvironment_Save();