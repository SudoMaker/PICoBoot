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

#include "UART.h"

#include <PICo24/Core/Delay.h>

void UART_Initialize(const UART_HandleTypeDef *huart, uint16_t uart_mode, uint16_t speed) {
	volatile UARTMODEBITS *m = huart->MODE;

	m->UARTEN = 0;

	if (uart_mode & UART_CSTOPB)
		m->STSEL = 1;
	else
		m->STSEL = 0;


	if (uart_mode & UART_CS9) {
		m->PDSEL = 0x3;
	} else {
		if (uart_mode & UART_PARENB) {
			if (uart_mode & UART_PARODD) {
				m->PDSEL = 2;
			} else {
				m->PDSEL = 1;
			}
		} else {
			m->PDSEL = 0;
		}
	}

	m->BRGH = 1;
	m->RXINV = 0;
	m->ABAUD = 0;
	m->LPBACK = 0;
	m->WAKE = 0;

	if (uart_mode & UART_CRTSCTS) {
		m->UEN = 2;
		m->RTSMD = 1;
	} else {
		m->UEN = 0;
		m->RTSMD = 0;
	}

	m->IREN = 0;
	m->USIDL = 0;

	// UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled;
	*(uint16_t *)huart->STA = 0x0000;

	*huart->BRG = speed;

	m->UARTEN = 1;
	huart->STA->UTXEN = 1;
}

void UART_SetSpeed(const UART_HandleTypeDef *huart, uint16_t speed) {
	*huart->BRG = speed;
}

void UART_Transmit(const UART_HandleTypeDef *huart, const uint8_t *buf, uint16_t len) {
	for (uint16_t i=0; i<len; i++) {
		while (huart->STA->UTXBF == 1);
		*huart->TXREG = buf[i];
	}
}

uint16_t UART_Receive(const UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len) {
	uint32_t byte_usecs = *huart->BRG * 3;
	uint32_t byte_usecs_used = 0;

	for (uint16_t i=0; i<len; i++) {
		while (huart->STA->URXDA == 0) {
			__delay32(FCY / 1000000);
			byte_usecs_used++;

			if (byte_usecs_used >= byte_usecs) {
				return i;
			}
		}

		if (huart->STA->OERR == 1) {
			huart->STA->OERR = 0;
		}

		buf[i] = *huart->RXREG;
		byte_usecs_used = 0;
	}

	return len;
}