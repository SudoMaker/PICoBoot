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

#ifndef _TRAPS_H
#define _TRAPS_H

#include <stdint.h>

/**
 * Error codes
 */
typedef enum 
{
    /* ----- Traps ----- */
    TRAPS_OSC_FAIL = 0, /** Oscillator Fail Trap vector */
    TRAPS_STACK_ERR = 1, /** Stack Error Trap Vector */
    TRAPS_ADDRESS_ERR = 2, /** Address Error Trap Vector */
    TRAPS_MATH_ERR = 3, /** Math Error Trap Vector */
} TRAPS_ERROR_CODE;

/**
  @Summary
    Default handler for the traps

  @Description
    This routine will be called whenever a trap happens. It stores the trap
    error code and waits forever.
    This routine has a weak attribute and can be over written.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    None.

*/
void TRAPS_halt_on_error(uint16_t code);

#endif