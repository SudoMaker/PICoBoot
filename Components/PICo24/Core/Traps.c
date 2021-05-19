/*
    This file is part of PotatoPi PICo24 SDK.

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

/* This file contains code originally written by Microchip Technology Inc., which was licensed under the Apache License. */

/**
  System Traps Generated Driver File 

  @Company:
    Microchip Technology Inc.

  @File Name:
    traps.h

  @Summary:
    This is the generated driver implementation file for handling traps
    using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description:
    This source file provides implementations for PIC24 / dsPIC33 / PIC32MM MCUs traps.
    Generation Information : 
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.170.0
        Device            :  PIC24FJ256GB206
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.61
        MPLAB             :  MPLAB X v5.45
*/

/**
    Section: Includes
*/
#include <xc.h>

#include <stdio.h>
#include <PICoBoot_Board.h>
#include "Traps.h"
#include "Delay.h"


#define ERROR_HANDLER __attribute__((interrupt,no_auto_psv))
#define FAILSAFE_STACK_GUARDSIZE 8

/**
 * a private place to store the error code if we run into a severe error
 */
static uint16_t TRAPS_error_code = -1;

/**
 * Halts 
 * 
 * @param code error code
 */
void __attribute__((weak)) TRAPS_halt_on_error(uint16_t code)
{
	TRAPS_error_code = code;

#ifdef __DEBUG
	__builtin_software_breakpoint();
    /* If we are in debug mode, cause a software breakpoint in the debugger */
#endif
	while (1) {
		Delay_Milliseconds(10);
	}
}

/**
 * Sets the stack pointer to a backup area of memory, in case we run into
 * a stack error (in which case we can't really trust the stack pointer)
 */
inline static void use_failsafe_stack(void)
{
	static uint8_t failsafe_stack[32];
	asm volatile (
	"   mov    %[pstack], W15\n"
	:
	: [pstack]"r"(failsafe_stack)
	);
/* Controls where the stack pointer limit is, relative to the end of the
 * failsafe stack
 */
	SPLIM = (uint16_t)(((uint8_t *)failsafe_stack) + sizeof(failsafe_stack)
			   - FAILSAFE_STACK_GUARDSIZE);
}

/** Oscillator Fail Trap vector**/
void ERROR_HANDLER _OscillatorFail(void)
{
	INTCON1bits.OSCFAIL = 0;  //Clear the trap flag
	TRAPS_halt_on_error(TRAPS_OSC_FAIL);
}
/** Stack Error Trap Vector**/
void ERROR_HANDLER _StackError(void)
{
	/* We use a failsafe stack: the presence of a stack-pointer error
	 * means that we cannot trust the stack to operate correctly unless
	 * we set the stack pointer to a safe place.
	 */
	use_failsafe_stack();
	INTCON1bits.STKERR = 0;  //Clear the trap flag
	TRAPS_halt_on_error(TRAPS_STACK_ERR);
}
/** Address Error Trap Vector**/
void ERROR_HANDLER _AddressError(void)
{
	INTCON1bits.ADDRERR = 0;  //Clear the trap flag
	TRAPS_halt_on_error(TRAPS_ADDRESS_ERR);
}
/** Math Error Trap Vector**/
void ERROR_HANDLER _MathError(void)
{
	INTCON1bits.MATHERR = 0;  //Clear the trap flag
	TRAPS_halt_on_error(TRAPS_MATH_ERR);
}
