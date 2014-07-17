/* Copyright (c) 2014 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * USB FTDI definitions.
 */

#ifndef USB_FTDI_H
#define USB_FTDI_H

#define USB_FTDI_SLEEP_USEC			500

#define USB_FTDI_PACKET_SIZE			(USB_MAX_PACKET_SIZE)
#define USB_FTDI_BUFFER_SIZE			(2 * USB_MAX_PACKET_SIZE)

#define USB_FTDI_REQ_RESET			0x0
#define USB_FTDI_REQ_MODEM_CTRL			0x1
#define USB_FTDI_REQ_SET_FLOW_CTRL		0x2
#define USB_FTDI_REQ_SET_BAUD_RATE		0x3
#define USB_FTDI_REQ_SET_DATA			0x4
#define USB_FTDI_REQ_GET_MODEM_STAT		0x5
#define USB_FTDI_REQ_SET_EVENT_CHAR		0x6
#define USB_FTDI_REQ_SET_ERROR_CHAR		0x7
#define USB_FTDI_REQ_SET_LAT_TIMER		0x9
#define USB_FTDI_REQ_GET_LAT_TIMER		0xa
#define USB_FTDI_REQ_SET_BIT_MODE		0xb
#define USB_FTDI_REQ_READ_PINS			0xc

#define USB_FTDI_BITMODE_RESET			0x00
#define USB_FTDI_BITMODE_BITBANG		0x01
#define USB_FTDI_BITMODE_MPSSE			0x02
#define USB_FTDI_BITMODE_SYNCBB			0x04
#define USB_FTDI_BITMODE_MCU			0x08
#define USB_FTDI_BITMODE_OPTO			0x10
#define USB_FTDI_BITMODE_CBUS			0x20
#define USB_FTDI_BITMODE_SYNCFF			0x40
#define USB_FTDI_BITMODE_ALL			0x4f

#define USB_FTDI_STATUS_BYTE1			0x32
#define USB_FTDI_STATUS_BYTE2			0x60

void ftdi_iface_request(usb_uint *ep0_buf_rx, usb_uint *ep0_buf_tx);

#endif /* USB_FTDI_H */
