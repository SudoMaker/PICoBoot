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

#include "usb_deluxe_device.h"
#include "usb_deluxe_device_cdc.h"

extern volatile CTRL_TRF_SETUP SetupPkt;

void USBDeluxeDevice_CDC_Create(USBDeluxeDevice_CDCContext *cdc_ctx, void *userp, uint8_t usb_iface_comm, uint8_t usb_iface_data,
				uint8_t usb_ep_comm, uint8_t usb_ep_data, USBDeluxeDevice_CDC_IOps *io_ops) {
	memset(cdc_ctx, 0, sizeof(USBDeluxeDevice_CDCContext));

	memcpy(&cdc_ctx->io_ops, io_ops, sizeof(USBDeluxeDevice_CDC_IOps));

	cdc_ctx->userp = userp;

	cdc_ctx->USB_IFACE_COMM = usb_iface_comm;
	cdc_ctx->USB_IFACE_DATA = usb_iface_data;

	cdc_ctx->USB_EP_COMM = usb_ep_comm;
	cdc_ctx->USB_EP_DATA = usb_ep_data;
}

void USBDeluxeDevice_CDC_Init(USBDeluxeDevice_CDCContext *cdc_ctx) {
	//Abstract line coding information
	cdc_ctx->line_coding.dwDTERate   = 19200;      // baud rate
	cdc_ctx->line_coding.bCharFormat = 0x00;             // 1 stop bit
	cdc_ctx->line_coding.bParityType = 0x00;             // None
	cdc_ctx->line_coding.bDataBits = 0x08;               // 5,6,7,8, or 16

	/*
	 * Do not have to init Cnt of IN pipes here.
	 * Reason:  Number of BYTEs to send to the host
	 *          varies from one transaction to
	 *          another. Cnt should equal the exact
	 *          number of BYTEs to transmit for
	 *          a given IN transaction.
	 *          This number of BYTEs will only
	 *          be known right before the data is
	 *          sent.
	 */

//	USBEnableEndpoint(cdc_ctx->USB_EP_COMM,USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
	USBEnableEndpoint(cdc_ctx->USB_EP_DATA,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

	cdc_ctx->CDCDataInHandle = USBRxOnePacket(cdc_ctx->USB_EP_DATA,(uint8_t *)&cdc_ctx->rx_buf, USBDeluxe_CDC_PKT_SIZE);
	cdc_ctx->CDCDataOutHandle = NULL;

	cdc_ctx->cdc_trf_state = CDC_TX_READY;
}//end CDCInitEP

#define dummy_length    0x08
static const uint8_t dummy_encapsulated_cmd_response[dummy_length];

void USBDeluxeDevice_CDC_CheckRequest(USBDeluxeDevice_CDCContext *cdc_ctx) {
	/*
	 * If request recipient is not an interface then return
	 */
	if (SetupPkt.Recipient != USB_SETUP_RECIPIENT_INTERFACE_BITFIELD) return;

	/*
	 * If request type is not class-specific then return
	 */
	if (SetupPkt.RequestType != USB_SETUP_TYPE_CLASS_BITFIELD) return;

	/*
	 * Interface ID must match interface numbers associated with
	 * CDC class, else return
	 */
	if ((SetupPkt.bIntfID != cdc_ctx->USB_IFACE_COMM)&&
	    (SetupPkt.bIntfID != cdc_ctx->USB_IFACE_DATA)) return;

	switch(SetupPkt.bRequest) {
		//****** These commands are required ******//
		case SEND_ENCAPSULATED_COMMAND:
			//send the packet
			inPipes[0].pSrc.bRam = (uint8_t*)&dummy_encapsulated_cmd_response;
			inPipes[0].wCount.Val = dummy_length;
			inPipes[0].info.bits.ctrl_trf_mem = USB_EP0_RAM;
			inPipes[0].info.bits.busy = 1;
			break;
		case GET_ENCAPSULATED_RESPONSE:
			// Populate dummy_encapsulated_cmd_response first.
			inPipes[0].pSrc.bRam = (uint8_t*)&dummy_encapsulated_cmd_response;
			inPipes[0].info.bits.busy = 1;
			break;
			//****** End of required commands ******//

#if defined(USB_CDC_SUPPORT_ABSTRACT_CONTROL_MANAGEMENT_CAPABILITIES_D1)
		case SET_LINE_CODING:
			outPipes[0].wCount.Val = SetupPkt.wLength;
			outPipes[0].pDst.bRam = (uint8_t*)LINE_CODING_TARGET;
			outPipes[0].pFunc = LINE_CODING_PFUNC;
			outPipes[0].info.bits.busy = 1;
			break;

		case GET_LINE_CODING:
		USBEP0SendRAMPtr(
			(uint8_t*)&cdc_ctx->line_coding,
			LINE_CODING_LENGTH,
			USB_EP0_INCLUDE_ZERO);
			break;

		case SET_CONTROL_LINE_STATE:
			cdc_ctx->control_signal_bitmap._byte = (uint8_t)SetupPkt.wValue;
			//------------------------------------------------------------------
			//One way to control the RTS pin is to allow the USB host to decide the value
			//that should be output on the RTS pin.  Although RTS and CTS pin functions
			//are technically intended for UART hardware based flow control, some legacy
			//UART devices use the RTS pin like a "general purpose" output pin
			//from the PC host.  In this usage model, the RTS pin is not related
			//to flow control for RX/TX.
			//In this scenario, the USB host would want to be able to control the RTS
			//pin, and the below line of code should be uncommented.
			//However, if the intention is to implement true RTS/CTS flow control
			//for the RX/TX pair, then this application firmware should override
			//the USB host's setting for RTS, and instead generate a real RTS signal,
			//based on the amount of remaining buffer space available for the
			//actual hardware UART of this microcontroller.  In this case, the
			//below code should be left commented out, but instead RTS should be
			//controlled in the application firmware responsible for operating the
			//hardware UART of this microcontroller.
			//---------
			//CONFIGURE_RTS(control_signal_bitmap.CARRIER_CONTROL);
			//------------------------------------------------------------------

			inPipes[0].info.bits.busy = 1;
			break;
#endif

		default:
			break;
	}//end switch(SetupPkt.bRequest)

}//end USBCheckCDCRequest

void USBDeluxeDevice_CDC_TryRx(USBDeluxeDevice_CDCContext *cdc_ctx) {
	if (!USBHandleBusy(cdc_ctx->CDCDataInHandle)) {
		uint8_t last_rx_len = USBHandleGetLength(cdc_ctx->CDCDataInHandle);

		if (last_rx_len) {
			cdc_ctx->rx_buf_pos += last_rx_len;

			if (cdc_ctx->io_ops.RxDone) {
				uint16_t user_processed_size = cdc_ctx->io_ops.RxDone(cdc_ctx->userp, cdc_ctx->rx_buf, cdc_ctx->rx_buf_pos);
				uint16_t left_size = cdc_ctx->rx_buf_pos - user_processed_size;

				if (left_size) {
					memmove(cdc_ctx->rx_buf, cdc_ctx->rx_buf + user_processed_size, left_size);
				}

				cdc_ctx->rx_buf_pos = left_size;

			} else {
				cdc_ctx->rx_buf_pos = 0;
			}
		}

		if (cdc_ctx->rx_buf_pos + USBDeluxe_CDC_PKT_SIZE < USBDeluxe_CDC_BUF_SIZE) {
			cdc_ctx->CDCDataInHandle = USBRxOnePacket(cdc_ctx->USB_EP_DATA, cdc_ctx->rx_buf + cdc_ctx->rx_buf_pos, USBDeluxe_CDC_PKT_SIZE);
		}
	}
}

void USBDeluxeDevice_CDC_SetTx(USBDeluxeDevice_CDCContext *cdc_ctx, uint8_t *data, uint16_t length) {
	USBMaskInterrupts();

	if (cdc_ctx->cdc_trf_state == CDC_TX_READY) {
		cdc_ctx->tx_target = data;
		cdc_ctx->tx_target_len = length;
		cdc_ctx->tx_target_pos = 0;
		cdc_ctx->cdc_trf_state = CDC_TX_BUSY;
	}

	USBUnmaskInterrupts();
}

void USBDeluxeDevice_CDC_DoTx(USBDeluxeDevice_CDCContext *cdc_ctx) {
	USBMaskInterrupts();


	if (USBHandleBusy(cdc_ctx->CDCDataOutHandle)) {
		USBUnmaskInterrupts();
		return;
	}

	/*
	 * Completing stage is necessary while [ mCDCUSartTxIsBusy()==1 ].
	 * By having this stage, user can always check cdc_trf_state,
	 * and not having to call mCDCUsartTxIsBusy() directly.
	 */
	if (cdc_ctx->cdc_trf_state == CDC_TX_COMPLETING) {
		cdc_ctx->cdc_trf_state = CDC_TX_READY;

		if (cdc_ctx->io_ops.TxDone) {
			cdc_ctx->io_ops.TxDone(cdc_ctx->userp);
		}
	}

	/*
	 * If CDC_TX_READY state, nothing to do, just return.
	 */
	if (cdc_ctx->cdc_trf_state == CDC_TX_READY) {
		USBUnmaskInterrupts();
		return;
	}

	/*
	 * If CDC_TX_BUSY_ZLP state, send zero length packet
	 */
	if (cdc_ctx->cdc_trf_state == CDC_TX_BUSY_ZLP) {
		cdc_ctx->CDCDataOutHandle = USBTxOnePacket(cdc_ctx->USB_EP_DATA, NULL, 0);
		//CDC_DATA_BD_IN.CNT = 0;
		cdc_ctx->cdc_trf_state = CDC_TX_COMPLETING;
	} else if (cdc_ctx->cdc_trf_state == CDC_TX_BUSY) {
		// First, have to figure out how many byte of data to send.
		uint16_t left_len = cdc_ctx->tx_target_len - cdc_ctx->tx_target_pos;
		uint8_t byte_to_send = left_len > USBDeluxe_CDC_PKT_SIZE ? USBDeluxe_CDC_PKT_SIZE : left_len;


		uint8_t *cur_mem_pos = cdc_ctx->tx_target + cdc_ctx->tx_target_pos;
		cdc_ctx->tx_target_pos += byte_to_send;


		cdc_ctx->CDCDataOutHandle = USBTxOnePacket(cdc_ctx->USB_EP_DATA, cur_mem_pos, byte_to_send);

		/*
		 * Lastly, determine if a zero length packet state is necessary.
		 * See explanation in USB Specification 2.0: Section 5.8.3
		 */
		if (cdc_ctx->tx_target_pos == cdc_ctx->tx_target_len) {
			if (byte_to_send == CDC_DATA_IN_EP_SIZE)
				cdc_ctx->cdc_trf_state = CDC_TX_BUSY_ZLP;
			else
				cdc_ctx->cdc_trf_state = CDC_TX_COMPLETING;
		}//end if(cdc_tx_len...)

	}//end if(cdc_tx_sate == CDC_TX_BUSY)

	USBUnmaskInterrupts();
}//end CDCTxService

void USBDeluxeDevice_CDC_Tasks(USBDeluxeDevice_CDCContext *cdc_ctx) {
	if (cdc_ctx->cdc_trf_state == CDC_TX_READY) { // All TX IO operations done
		USBDeluxeDevice_CDC_TryRx(cdc_ctx);

		if (cdc_ctx->tx_buf_len[cdc_ctx->tx_buf_idx]) {
			USBDeluxeDevice_CDC_SetTx(cdc_ctx, cdc_ctx->tx_buf[cdc_ctx->tx_buf_idx], cdc_ctx->tx_buf_len[cdc_ctx->tx_buf_idx]);
			cdc_ctx->tx_buf_len[cdc_ctx->tx_buf_idx] = 0;
		}

		cdc_ctx->tx_buf_idx = !cdc_ctx->tx_buf_idx;
	}

	USBDeluxeDevice_CDC_DoTx(cdc_ctx);
}

int USBDeluxeDevice_CDC_AcquireTxBuffer(USBDeluxeDevice_CDCContext *cdc_ctx, USBDeluxeDevice_CDC_UserBuffer *user_buf) {
	if (cdc_ctx->tx_buf_len[cdc_ctx->tx_buf_idx] == 0) {
		user_buf->buf = cdc_ctx->tx_buf[cdc_ctx->tx_buf_idx];
		user_buf->buf_len = &cdc_ctx->tx_buf_len[cdc_ctx->tx_buf_idx];
		return 0;
	} else {
		return -1;
	}
}

int USBDeluxeDevice_CDC_Write(USBDeluxeDevice_CDCContext *cdc_ctx, uint8_t *buf, uint16_t len) {
	USBDeluxeDevice_CDC_UserBuffer user_buf;

	if (USBDeluxeDevice_CDC_AcquireTxBuffer(cdc_ctx, &user_buf)) {
		return -1;
	} else {
		memcpy(user_buf.buf, buf, len);
		*user_buf.buf_len = len;
		return 0;
	}
}