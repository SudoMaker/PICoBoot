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

#pragma once

#include <stdint.h>

__extension__ typedef struct {
	union {
		struct {
			uint16_t STSEL:1;
			uint16_t PDSEL:2;
			uint16_t BRGH:1;
			uint16_t RXINV:1;
			uint16_t ABAUD:1;
			uint16_t LPBACK:1;
			uint16_t WAKE:1;
			uint16_t UEN:2;
			uint16_t :1;
			uint16_t RTSMD:1;
			uint16_t IREN:1;
			uint16_t USIDL:1;
			uint16_t :1;
			uint16_t UARTEN:1;
		};
		struct {
			uint16_t :1;
			uint16_t PDSEL0:1;
			uint16_t PDSEL1:1;
			uint16_t :5;
			uint16_t UEN0:1;
			uint16_t UEN1:1;
		};
	};
} UARTMODEBITS;

__extension__ typedef struct {
	union {
		struct {
			uint16_t URXDA:1;
			uint16_t OERR:1;
			uint16_t FERR:1;
			uint16_t PERR:1;
			uint16_t RIDLE:1;
			uint16_t ADDEN:1;
			uint16_t URXISEL:2;
			uint16_t TRMT:1;
			uint16_t UTXBF:1;
			uint16_t UTXEN:1;
			uint16_t UTXBRK:1;
			uint16_t :1;
			uint16_t UTXISEL0:1;
			uint16_t UTXINV:1;
			uint16_t UTXISEL1:1;
		};
		struct {
			uint16_t :6;
			uint16_t URXISEL0:1;
			uint16_t URXISEL1:1;
		};
	};
} UARTSTABITS;


typedef struct {
	volatile UARTSTABITS *STA;
	volatile UARTMODEBITS *MODE;
	volatile uint16_t *BRG;
	volatile uint16_t *TXREG;
	volatile uint16_t *RXREG;
} UART_HandleTypeDef;

enum {
	UART_CS8 = 0x0,
	UART_CS9 = 0x1,

	UART_CRTSCTS = 0x10,
	UART_PARENB = 0x20,
	UART_PARODD = 0x40,
	UART_CSTOPB = 0x80,
};

extern const UART_HandleTypeDef huart1;
extern const UART_HandleTypeDef huart2;
extern const UART_HandleTypeDef huart3;

extern void UART_Initialize(const UART_HandleTypeDef *huart, uint16_t uart_mode, uint16_t speed);
extern void UART_SetSpeed(const UART_HandleTypeDef *huart, uint16_t speed);
extern void UART_Transmit(const UART_HandleTypeDef *huart, const uint8_t *buf, uint16_t len);
extern uint16_t UART_Receive(const UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len);