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

#include "PICoBoot_Board.h"
#include <PICoBoot/PICoBoot.h>

// CONFIG3
#pragma config WPFP = WPFP255		// Write Protection Flash Page Segment Boundary->Highest Page (same as page 170)
#pragma config SOSCSEL = EC		// Secondary Oscillator Power Mode Select->External clock (SCLKI) or Digital I/O mode(
#pragma config WUTSEL = LEG		// Voltage Regulator Wake-up Time Select->Default regulator start-up time is used
#pragma config ALTPMP = ALPMPDIS	// Alternate PMP Pin Mapping->EPMP pins are in default location mode
#pragma config WPDIS = WPDIS		// Segment Write Protection Disable->Segmented code protection is disabled
#pragma config WPCFG = WPCFGDIS		// Write Protect Configuration Page Select->Last page (at the top of program memory) and Flash Configuration Words are not write-protected
#pragma config WPEND = WPENDMEM		// Segment Write Protection End Page Select->Protected code segment upper boundary is at the last page of program memory; the lower boundary is the code page specified by WPFP

// CONFIG2
#pragma config POSCMOD = XT		// Primary Oscillator Select->XT Oscillator mode is selected
#pragma config IOL1WAY = ON		// IOLOCK One-Way Set Enable->The IOLOCK bit (OSCCON<6>) can be set once, provided the unlock sequence has been completed. Once set, the Peripheral Pin Select registers cannot be written to a second time.
#pragma config OSCIOFNC = ON		// OSCO Pin Configuration->OSCO/CLKO/RC15 functions as port I/O (RC15)
#pragma config FCKSM = CSECMD		// Clock Switching and Fail-Safe Clock Monitor->Clock switching is enabled, Fail-Safe Clock Monitor is disabled
#pragma config FNOSC = FRC		// Initial Oscillator Select->FRC
#pragma config PLL96MHZ = ON		// 96MHz PLL Startup Select->96 MHz PLL is enabled automatically on start-up
#pragma config PLLDIV = DIV2		// 96 MHz PLL Prescaler Select->Oscillator input is divided by 2 (8 MHz input)
#pragma config IESO = OFF		// Internal External Switchover->IESO mode (Two-Speed Start-up) is disabled

// CONFIG1
#pragma config WDTPS = PS32768		// Watchdog Timer Postscaler->1:32768
#pragma config FWPSA = PR128		// WDT Prescaler->Prescaler ratio of 1:128
#pragma config WINDIS = OFF		// Windowed WDT->Standard Watchdog Timer enabled,(Windowed-mode is disabled)
#pragma config FWDTEN = OFF		// Watchdog Timer->Watchdog Timer is disabled
#pragma config ICS = PGx3		// Emulator Pin Placement Select bits->Emulator functions are shared with PGEC3/PGED3
#pragma config GWRP = OFF		// General Segment Write Protect->Writes to program memory are allowed
#pragma config GCP = ON			// General Segment Code Protect->Code protection is **enabled**
#pragma config JTAGEN = OFF		// JTAG Port Enable->JTAG port is disabled

const uint32_t XTAL_FREQ = 32000000UL;
const uint32_t FCY = 32000000UL / 2;

const char PICoBoot_BoardManufacturer[] = "SudoMaker";
const char PICoBoot_Board[] = "PotatoPi PICo24";
const char PICoBoot_ChipManufacturer[] = "Microchip";
const char PICoBoot_Chip[] = "PIC24FJ256GB206";

#define StaticEnvironment_Address	0x0200

const uint32_t PICoBoot_Bootloader_Address = 0x0400;
const uint32_t PICoBoot_App_Address = 0x4000;
const uint32_t PICoBoot_App_Size = 0x26bf8;
const uint32_t PICoBoot_StaticEnvironment_Address = StaticEnvironment_Address;

const uint16_t PICoBoot_Flash_PageSize = 1024;

const uint32_t PICoBoot_FlashProhibitedRanges[] = {
	0x0400, 0x3fff,		// BL itself
	0x2a800, 0x2afff	// Config bits
};

volatile const uint8_t __attribute__((space(prog), address(StaticEnvironment_Address))) PICoBoot_StaticEnvironment_DefaultValues[32] = {
	PICoBoot_ENVAR_UNINIT, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, PICoBoot_ENVAR_UNINIT
};

void PICoBoot_Board_PinInitialize() {
	/****************************************************************************
	 * Setting the Output Latch SFR(s)
	 ***************************************************************************/
	LATB = 0x0000;
	LATC = 0x0000;
	LATD = 0x0000;
	LATE = 0x0000;
	LATF = 0x0000;
	LATG = 0x0000;

	/****************************************************************************
	 * Setting the GPIO Direction SFR(s)
	 ***************************************************************************/
	TRISB = 0xFF3C;
	TRISC = 0x7000;
	TRISD = 0x0FD7;
	TRISE = 0x00FF;
	TRISF = 0x0038;
	TRISG = 0x034C;

	/****************************************************************************
	 * Setting the Weak Pull Up and Weak Pull Down SFR(s)
	 ***************************************************************************/
	CNPD1 = 0x0000;
	CNPD2 = 0x0000;
	CNPD3 = 0x0000;
	CNPD4 = 0x0000;
	CNPD5 = 0x0000;
	CNPD6 = 0x0000;
	CNPU1 = 0x0000;
	CNPU2 = 0x0001;
	CNPU3 = 0x0000;
	CNPU4 = 0x0000;
	CNPU5 = 0x0000;
	CNPU6 = 0x0000;

	/****************************************************************************
	 * Setting the Open Drain SFR(s)
	 ***************************************************************************/
	ODCB = 0x0000;
	ODCC = 0x0000;
	ODCD = 0x0000;
	ODCE = 0x0000;
	ODCF = 0x0000;
	ODCG = 0x0000;

	/****************************************************************************
	 * Setting the Analog/Digital Configuration SFR(s)
	 ***************************************************************************/
	ANSB = 0xFF08;
	ANSC = 0x6000;
	ANSD = 0x0040;
	ANSF = 0x0000;
	ANSG = 0x0300;
}

void PICoBoot_Board_SystemInitialize() {
	__builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS

	RPINR18bits.U1RXR = 0x0015;    //RG6->UART1:U1RX
	RPOR13bits.RP26R = 0x0003;    //RG7->UART1:U1TX

	__builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS

	// CPDIV 1:1; RCDIV FRC/2; DOZE 1:8; DOZEN disabled; ROI disabled;
	CLKDIV = 0x3100;
	// TUN Center frequency;
	OSCTUN = 0x00;
	// ADC1MD enabled; T3MD enabled; T4MD enabled; T1MD enabled; U2MD enabled; T2MD enabled; U1MD enabled; SPI2MD enabled; SPI1MD enabled; T5MD enabled; I2C1MD enabled;
	PMD1 = 0x00;
	// OC5MD enabled; OC6MD enabled; OC7MD enabled; OC8MD enabled; OC1MD enabled; IC2MD enabled; OC2MD enabled; IC1MD enabled; OC3MD enabled; OC4MD enabled; IC6MD enabled; IC7MD enabled; IC5MD enabled; IC8MD enabled; IC4MD enabled; IC3MD enabled;
	PMD2 = 0x00;
	// I2C3MD enabled; PMPMD enabled; U3MD enabled; RTCCMD enabled; CMPMD enabled; CRCMD enabled; I2C2MD enabled;
	PMD3 = 0x00;
	// U4MD enabled; UPWMMD enabled; USB1MD enabled; CTMUMD enabled; REFOMD enabled; LVDMD enabled;
	PMD4 = 0x00;
	// IC9MD enabled; OC9MD enabled;
	PMD5 = 0x00;
	// SPI3MD enabled;
	PMD6 = 0x00;
	// CF no clock failure; NOSC PRIPLL; SOSCEN disabled; POSCEN disabled; CLKLOCK unlocked; OSWEN Switch is Complete;
	__builtin_write_OSCCONH((uint8_t) (0x03));
	__builtin_write_OSCCONL((uint8_t) (0x01));

	// Important: Make use of the DefaultValues so it will not be LTO'd
	for (size_t i=0; i<sizeof(PICoBoot_StaticEnvironment_DefaultValues); i++) {
		PICoBoot_LED_1 = PICoBoot_StaticEnvironment_DefaultValues[i];
		PICoBoot_LED_1 = 0;
	}

	// Wait for Clock switch to occur
	while (OSCCONbits.OSWEN != 0);
	while (OSCCONbits.LOCK != 1);
}

int PICoBoot_Board_Reset_Action() {
	Delay_Milliseconds(10);

	if (picoboot_runtime_env.reboot_target == RebootTarget_Bootloader) {
		picoboot_runtime_env.reboot_target = RebootTarget_Default;
		return ResetAction_StayInBL;
	}

	if (picoboot_static_env.watchdog_fails_to_enter_bl) {
		if (picoboot_runtime_env.watchdog_failed_count >= picoboot_static_env.watchdog_fails_to_enter_bl) {
			return ResetAction_StayInBL;
		}
	}

	if (PORTDbits.RD7 == 1) {
		return ResetAction_RunUserApp;
	} else {
		return ResetAction_StayInBL;
	}
}

uint32_t PICoBoot_Board_Flash_ReadInstruction(uint32_t addr) {
	uint32_t ret;

	uint16_t tblpag_save = TBLPAG;

	TBLPAG = (addr >> 16) & 0xff;		// Load TBLPAG register with read address <23:16>
	uint16_t addrOffset = addr & 0xffff;	// Load offset with read address <15:0>

	// Read data from program memory
	ret = __builtin_tblrdl(addrOffset);   // readDataL contains lower word data
	ret |= (uint32_t)__builtin_tblrdh(addrOffset) << 16;

	TBLPAG = tblpag_save;

	return ret;
}

void PICoBoot_Board_Flash_ErasePage(uint32_t addr) {
	NVMCON = 0x4042;			// Set WREN, ERASE, NVMOP<3:0> = 0010 (Memory page erase operation)

	uint16_t tblpag_save = TBLPAG;

	TBLPAG = (addr >> 16) & 0xff;		// Load TBLPAG register with read address <23:16>
	uint16_t addrOffset = addr & 0xffff;	// Load offset with read address <15:0>

	__builtin_tblwtl(addrOffset, 0);

	__builtin_disable_interrupts();
	__builtin_write_NVM();

	while (NVMCONbits.WR == 1);

	TBLPAG = tblpag_save;

	NVMCONbits.WREN = 0;
}

void PICoBoot_Board_Flash_WriteInstruction(uint32_t addr, uint32_t value) {
	NVMCON = 0x4003;			// Set WREN, NVMOP<3:0> = 0011 (Memory word program operation)

	uint16_t tblpag_save = TBLPAG;

	TBLPAG = (addr >> 16) & 0xff;		// Load TBLPAG register with read address <23:16>
	uint16_t addrOffset = addr & 0xffff;	// Load offset with read address <15:0>

	__builtin_tblwtl(addrOffset, value & 0xffff);
	__builtin_tblwth(addrOffset, (value >> 16) & 0xff);

	__builtin_disable_interrupts();
	__builtin_write_NVM();

	while (NVMCONbits.WR == 1);

	TBLPAG = tblpag_save;

	NVMCONbits.WREN = 0;
}

__eds__ __attribute__((persistent, address(0x18000))) uint8_t __runtime_env[sizeof(PicoBootRuntimeEnvironment)];

void PICoBoot_RuntimeEnvironment_Load() {
	uint8_t *penv = (uint8_t *) &picoboot_runtime_env;

	for (size_t i=0; i<sizeof(PicoBootRuntimeEnvironment); i++) {
		penv[i] = __runtime_env[i];
	}
}

void PICoBoot_RuntimeEnvironment_Save() {
	uint8_t *penv = (uint8_t *) &picoboot_runtime_env;

	for (size_t i=0; i<sizeof(PicoBootRuntimeEnvironment); i++) {
		__runtime_env[i] = penv[i];
	}
}