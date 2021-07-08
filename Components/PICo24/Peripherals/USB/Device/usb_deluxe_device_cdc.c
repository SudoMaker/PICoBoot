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


void USBDeluxeDevice_CDC_ACM_Create(USBDeluxeDevice_CDCACMContext *cdc_ctx, void *userp, uint8_t usb_iface_comm, uint8_t usb_iface_data,
				    uint8_t usb_ep_comm, uint8_t usb_ep_data, USBDeluxeDevice_CDC_ACM_IOps *io_ops) {
	memset(cdc_ctx, 0, sizeof(USBDeluxeDevice_CDCACMContext));

	if (io_ops) {
		memcpy(&cdc_ctx->io_ops, io_ops, sizeof(USBDeluxeDevice_CDC_ACM_IOps));
	}

	cdc_ctx->userp = userp;

	cdc_ctx->USB_IFACE_COMM = usb_iface_comm;
	cdc_ctx->USB_IFACE_DATA = usb_iface_data;

	cdc_ctx->USB_EP_COMM = usb_ep_comm;
	cdc_ctx->USB_EP_DATA = usb_ep_data;
}

void USBDeluxeDevice_CDC_ACM_Init(USBDeluxeDevice_CDCACMContext *cdc_ctx) {
	USBEnableEndpoint(cdc_ctx->USB_EP_DATA,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

	cdc_ctx->CDCDataInHandle = USBRxOnePacket(cdc_ctx->USB_EP_DATA,(uint8_t *)cdc_ctx->rx_buf, USBDeluxe_CDC_PKT_SIZE);
	cdc_ctx->CDCDataOutHandle = NULL;

}

void USBDeluxeDevice_CDC_ACM_CheckRequest(USBDeluxeDevice_CDCACMContext *cdc_ctx) {
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

	static uint8_t dummy[8];

	switch (SetupPkt.bRequest) {
		case SEND_ENCAPSULATED_COMMAND:
		case GET_ENCAPSULATED_RESPONSE:
		USBEP0SendROMPtr(dummy, sizeof(dummy), USB_EP0_INCLUDE_ZERO);
			break;
		default:
			break;
	}

}

void USBDeluxeDevice_CDC_ACM_TryRx(USBDeluxeDevice_CDCACMContext *cdc_ctx) {
	if (!USBHandleBusy(cdc_ctx->CDCDataInHandle)) {
		uint8_t cur_buf_idx = cdc_ctx->rx_buf_idx;
		uint8_t last_rx_len = USBHandleGetLength(cdc_ctx->CDCDataInHandle);

		cdc_ctx->rx_buf_idx++;
		cdc_ctx->rx_buf_idx %= sizeof(cdc_ctx->rx_buf_len);

		cdc_ctx->CDCDataInHandle = USBRxOnePacket(cdc_ctx->USB_EP_DATA, cdc_ctx->rx_buf[cdc_ctx->rx_buf_idx], USBDeluxe_CDC_PKT_SIZE);

		cdc_ctx->rx_buf_len[cur_buf_idx] = last_rx_len;
		cdc_ctx->rx_buf_pos[cur_buf_idx] = 0;

		if (cdc_ctx->io_ops.RxDone) {
			cdc_ctx->io_ops.RxDone(cdc_ctx->userp, cdc_ctx->rx_buf[cur_buf_idx], last_rx_len);
		}
	}
}

uint8_t USBDeluxeDevice_CDC_ACM_DoTx(USBDeluxeDevice_CDCACMContext *cdc_ctx) {
	uint8_t tx_done = 0;

	USBMaskInterrupts();

	if (!USBHandleBusy(cdc_ctx->CDCDataOutHandle)) { // Last TX completed
		uint8_t cur_buf_idx = cdc_ctx->tx_buf_idx; // Last configured buffer index

		if (cdc_ctx->tx_buf_len[cur_buf_idx]) {
			cdc_ctx->CDCDataOutHandle = USBTxOnePacket(cdc_ctx->USB_EP_DATA, cdc_ctx->tx_buf[cur_buf_idx], cdc_ctx->tx_buf_len[cur_buf_idx]);

			cdc_ctx->tx_buf_idx++; // Switch to another buffer
			cdc_ctx->tx_buf_idx %= sizeof(cdc_ctx->tx_buf_len);
			cdc_ctx->tx_buf_len[cdc_ctx->tx_buf_idx] = 0; // Mark the 'another' buffer will be used as empty
			tx_done = 1;
		}
	} else {
		tx_done = 2;
	}

	USBUnmaskInterrupts();

	return tx_done;
}

void USBDeluxeDevice_CDC_ACM_Tasks(USBDeluxeDevice_CDCACMContext *cdc_ctx) {
	USBDeluxeDevice_CDC_ACM_DoTx(cdc_ctx);
	USBDeluxeDevice_CDC_ACM_TryRx(cdc_ctx);
}

int USBDeluxeDevice_CDC_ACM_AcquireTxBuffer(USBDeluxeDevice_CDCACMContext *cdc_ctx, USBDeluxeDevice_CDC_UserBuffer *user_buf) {
	user_buf->buf = cdc_ctx->tx_buf[cdc_ctx->tx_buf_idx];
	user_buf->buf_len = &cdc_ctx->tx_buf_len[cdc_ctx->tx_buf_idx];
	return 0;
}

ssize_t USBDeluxeDevice_CDC_ACM_Write(USBDeluxeDevice_CDCACMContext *cdc_ctx, uint8_t *buf, size_t len) {
	USBDeluxeDevice_CDC_UserBuffer user_buf;

	if (USBDeluxeDevice_CDC_ACM_AcquireTxBuffer(cdc_ctx, &user_buf) == -1) {
		return -1;
	}

	if (len > USBDeluxe_CDC_BUF_SIZE) {
		len = USBDeluxe_CDC_BUF_SIZE;
	}

	memcpy(user_buf.buf, buf, len);
	*user_buf.buf_len = len;

	return len;
}