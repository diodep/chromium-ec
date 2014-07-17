/* Copyright (c) 2014 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "clock.h"
#include "common.h"
#include "config.h"
#include "console.h"
#include "gpio.h"
#include "hooks.h"
#include "hwtimer.h"
#include "link_defs.h"
#include "registers.h"
#include "task.h"
#include "timer.h"
#include "util.h"
#include "usb.h"
#include "usb_ftdi.h"

/* Console output macro */
#define CPRINTF(format, args...) cprintf(CC_USBFTDI, format, ## args)

/*
 * Implements the FTDI MPSSE protocol on an emulated FT232H.
 * Supports only interface A, so USB_IFACE_FTDI must be 0, TX (USB_EP_FTDI_TX)
 * must be 0x81, and RX (USB_EP_FTDI_RX) must be 0x02.
 */

#define USB_FTDI_CMD_BIT_WRITE_MCE		(1 << 0)
#define USB_FTDI_CMD_BIT_BIT_MODE		(1 << 1)
#define USB_FTDI_CMD_BIT_READ_MCE		(1 << 2)
#define USB_FTDI_CMD_BIT_LSB_FIRST		(1 << 3)
#define USB_FTDI_CMD_BIT_WRITE_TDI		(1 << 4)
#define USB_FTDI_CMD_BIT_READ_TDO		(1 << 5)
#define USB_FTDI_CMD_BIT_WRITE_TMS		(1 << 6)
#define USB_FTDI_CMD_BIT_SPECIAL		(1 << 7)

#define USB_FTDI_CMD_WRITE_LOW_BYTE		0x80
#define USB_FTDI_CMD_WRITE_HIGH_BYTE		0x82
#define USB_FTDI_CMD_LOOPBACK_OFF		0x85
#define USB_FTDI_CMD_SET_CLK_DIV		0x86
#define USB_FTDI_CMD_SEND_IMMEDIATE		0x87
#define USB_FTDI_CMD_CLK_DIV_5_OFF		0x8a
#define USB_FTDI_CMD_CLK_DIV_5_ON		0x8b
#define USB_FTDI_CMD_CLOCK_3_PHASE_OFF		0x8d
#define USB_FTDI_CMD_ADAPT_CLOCK_OFF		0x97
#define USB_FTDI_CMD_BAD			0xfa

/*
 * FT232H JTAG Pin Configuration (JTAGkey-compatible):
 * ADBUS0 (13): TCK/SK      : EC_JTAG_TCK_GATED
 * ADBUS1 (14): TDI/DO      : EC_JTAG_TDI_GATED
 * ADBUS2 (15): TDO/DI      : EC_JTAG_TDO
 * ADBUS3 (16): TMS/CS      : EC_JTAG_TMS
 * ADBUS4 (17): JTAG_OE_EN  : PD_DEBUG_EN_MCU
 * ADBUS5 (18): VREF_IN_N   : HIGH
 * ADBUS6 (19): SRST_IN     : EC_RST_L
 * ACBUS0 (21): TRST_OUT    :
 * ACBUS1 (25): SRST_OUT    : EC_RST_L
 * ACBUS2 (26): TRST_OE_N   : HIGH
 * ACBUS3 (27): SRST_OE_N   : HIGH
 */

#define TIM_JTAG_TCK				15

#define ADBUS_TCK				(1 << 0)
#define ADBUS_TDI				(1 << 1)
#define ADBUS_TDO				(1 << 2)
#define ADBUS_TMS				(1 << 3)
#define ADBUS_JTAG_OE				(1 << 4)
#define ADBUS_VREF				(1 << 5)
#define ADBUS_SRST_IN				(1 << 6)
#define ACBUS_TRST_OUT				(1 << 0)
#define ACBUS_SRST_OUT				(1 << 1)
#define ACBUS_TRST_OE				(1 << 2)
#define ACBUS_SRST_OE				(1 << 3)

/* FTDI descriptors */
const struct usb_interface_descriptor USB_IFACE_DESC(USB_IFACE_FTDI) = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = USB_IFACE_FTDI,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,
};
const struct usb_endpoint_descriptor USB_EP_DESC(USB_IFACE_FTDI, USB_EP_FTDI_TX) = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x80 | USB_EP_FTDI_TX,
	.bmAttributes = 0x02 /* Bulk IN */,
	.wMaxPacketSize = USB_FTDI_PACKET_SIZE,
	.bInterval = 0,
};
const struct usb_endpoint_descriptor USB_EP_DESC(USB_IFACE_FTDI, USB_EP_FTDI_RX) = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_EP_FTDI_RX,
	.bmAttributes = 0x02 /* Bulk OUT */,
	.wMaxPacketSize = USB_FTDI_PACKET_SIZE,
	.bInterval = 0,
};

/* Hardware buffers for USB endpoints */
usb_uint ftdi_ep_tx[USB_FTDI_PACKET_SIZE] __usb_ram;
usb_uint ftdi_ep_rx[USB_FTDI_PACKET_SIZE] __usb_ram;

/* Local buffer for command execution and reassembly */
static uint8_t cmd_buf[USB_FTDI_BUFFER_SIZE];
static int cmd_pos;
static int cmd_len;

/* Local buffer for message send queue */
static uint8_t send_buf[USB_FTDI_BUFFER_SIZE];
static int send_pos;
static int send_len;

/* FTDI chip state variables */
static int clk_div5 = 1;

static void ftdi_tx_reset(void)
{
	btable_ep[USB_EP_FTDI_TX].tx_addr = usb_sram_addr(ftdi_ep_tx);
	btable_ep[USB_EP_FTDI_TX].tx_count = 0;
	btable_ep[USB_EP_FTDI_TX].rx_count = 0;

	STM32_USB_EP(USB_EP_FTDI_TX) =
		(USB_EP_FTDI_TX << 0) /* Endpoint Address */ |
		(2 << 4) /* TX NAK */ |
		(0 << 9) /* Bulk EP */ |
		(0 << 12) /* RX Disabled */;
}

static void ftdi_rx_reset(void)
{
	btable_ep[USB_EP_FTDI_RX].rx_addr = usb_sram_addr(ftdi_ep_rx);
	btable_ep[USB_EP_FTDI_RX].rx_count = 0x8000 |
				((USB_FTDI_PACKET_SIZE/32-1) << 10);
	btable_ep[USB_EP_FTDI_RX].tx_count = 0;

	STM32_USB_EP(USB_EP_FTDI_RX) =
		(USB_EP_FTDI_RX << 0) /* Endpoint Address */ |
		(0 << 4) /* TX Disabled */ |
		(0 << 9) /* Bulk EP */ |
		(3 << 12) /* RX VALID */;
}

/*
 * Transfers data to be sent from queue to USB buffer, with header prepended.
 */
static void ftdi_send(void)
{
	int write_len;

	if (send_pos >= send_len) {
		send_pos = 0;
		send_len = 0;
		return;
	}

	ftdi_ep_tx[0] = ((USB_FTDI_STATUS_BYTE2 << 8)
		| USB_FTDI_STATUS_BYTE1);
	write_len = MIN(send_len - send_pos, USB_FTDI_PACKET_SIZE - 2);

	memcpy_usbram(ftdi_ep_tx + 1, send_buf + send_pos, write_len);
	btable_ep[USB_EP_FTDI_TX].tx_count = write_len + 2;

	send_pos += write_len;

	/* enable data read */
	STM32_TOGGLE_EP(USB_EP_FTDI_TX, EP_TX_MASK, EP_TX_VALID, 0);
}

/*
 * Triggers send and waits for transaction to complete.
 */
static void ftdi_send_wait(void)
{
	ftdi_send();

	/* wait for send to finish */
	while (send_len)
		usleep(USB_FTDI_SLEEP_USEC);
}

/*
 * Queues data to be sent.
 * Length argument is in bytes.
 */
static void ftdi_send_queue(uint8_t *src, int len)
{
	/* buffer will overflow, trigger send now */
	if (send_len + len > USB_FTDI_BUFFER_SIZE) {
		ftdi_send_wait();
	}

	memcpy(send_buf + send_len, src, len * sizeof(*src));
	send_len += len;
}

static void ftdi_tx(void)
{
	/* continue sending anything remianing in buffer */
	ftdi_send();

	STM32_USB_EP(USB_EP_FTDI_TX) &= EP_MASK;
}

static void ftdi_rx(void)
{
	task_set_event(TASK_ID_USB_FTDI, TASK_EVENT_WAKE, 0);

	STM32_USB_EP(USB_EP_FTDI_RX) &= EP_MASK;
}
USB_DECLARE_EP(USB_EP_FTDI_TX, ftdi_tx, ftdi_tx, ftdi_tx_reset);
USB_DECLARE_EP(USB_EP_FTDI_RX, ftdi_rx, ftdi_rx, ftdi_rx_reset);

/* TODO put static */
void ftdi_iface_request(usb_uint *ep0_buf_rx, usb_uint *ep0_buf_tx)
{
	uint16_t *req = (uint16_t *) ep0_buf_rx;

	if ((req[0] & (USB_DIR_IN | USB_RECIP_MASK | USB_TYPE_MASK)) ==
	    (USB_DIR_IN | USB_RECIP_DEVICE | USB_TYPE_VENDOR)) {
		switch (req[0] >> 8) {
		case USB_FTDI_REQ_GET_MODEM_STAT:
			/* return hardcoded status */
			ep0_buf_tx[0] = ((USB_FTDI_STATUS_BYTE2 << 8)
				| USB_FTDI_STATUS_BYTE1);
			btable_ep[0].tx_count = 2 * sizeof(uint8_t);
			STM32_TOGGLE_EP(USB_EP_CONTROL, EP_TX_RX_MASK,
				EP_TX_RX_VALID, EP_STATUS_OUT
				/*null OUT transaction */);
		break;
		case USB_FTDI_REQ_GET_LAT_TIMER:
			/* return default value */
			ep0_buf_tx[0] = 0x02;
			btable_ep[0].tx_count = sizeof(uint8_t);
			STM32_TOGGLE_EP(USB_EP_CONTROL, EP_TX_RX_MASK,
				EP_TX_RX_VALID, EP_STATUS_OUT
				/*null OUT transaction */);
		break;
		}
	} else if ((req[0] & (USB_DIR_OUT | USB_RECIP_MASK | USB_TYPE_MASK)) ==
	    (USB_DIR_OUT | USB_RECIP_DEVICE | USB_TYPE_VENDOR)) {
		switch (req[0] >> 8) {
		case USB_FTDI_REQ_SET_BIT_MODE:
			/* only MPSSE mode supported */
			if (!(req[1] & USB_FTDI_BITMODE_MPSSE))
				STM32_TOGGLE_EP(USB_EP_CONTROL, EP_TX_RX_MASK,
				EP_RX_STALL | EP_TX_VALID, 0);
			/* fall through to default handler */
		default:
			/* silently acknowledge */
			btable_ep[0].tx_count = 0;
			STM32_TOGGLE_EP(USB_EP_CONTROL, EP_TX_RX_MASK,
				EP_TX_RX_VALID, 0);
		break;
		}
	} else {
		CPRINTF("ftdi stalling: %x %x %x %x\n",
		  req[0], req[1], req[2], req[3]);
		STM32_TOGGLE_EP(USB_EP_CONTROL, EP_TX_RX_MASK,
			EP_RX_VALID | EP_TX_STALL, 0);
	}
}

static void ftdi_set_byte(int high, uint8_t value, uint8_t direction)
{
	/* check value before calling to prevent glitch on
	 * transition to open drain in gpio_set_flags() */
	if (!high) {
		if (direction & ADBUS_TCK) {
			if ((STM32_GPIO_MODER(GPIO_C) & (3 << (6 * 2)))
				!= (1 << (6 * 2)))
				gpio_set_flags(GPIO_EC_JTAG_TCK,
					(value & ADBUS_TCK) ?
					GPIO_OUT_HIGH : GPIO_OUT_LOW);
			else
				/* we do not support starting from CLK = 1 */
				ASSERT(!(value & ADBUS_TCK));
				gpio_set_level(GPIO_EC_JTAG_TCK,
					value & ADBUS_TCK);
		} else
			gpio_set_flags(GPIO_EC_JTAG_TCK, GPIO_INPUT);

		if (direction & ADBUS_TDI) {
			if ((STM32_GPIO_MODER(GPIO_C) & (3 << (9 * 2)))
				!= (1 << (9 * 2)))
				gpio_set_flags(GPIO_EC_JTAG_TDI,
					(value & ADBUS_TDI) ?
					GPIO_OUT_HIGH : GPIO_OUT_LOW);
			else
				gpio_set_level(GPIO_EC_JTAG_TDI,
					value & ADBUS_TDI);
		} else
			gpio_set_flags(GPIO_EC_JTAG_TDI, GPIO_INPUT);

		if (direction & ADBUS_TDO) {
			if ((STM32_GPIO_MODER(GPIO_C) & (3 << (8 * 2)))
				!= (1 << (8 * 2)))
				gpio_set_flags(GPIO_EC_JTAG_TDO,
					(value & ADBUS_TDO) ?
					GPIO_OUT_HIGH : GPIO_OUT_LOW);
			else
				gpio_set_level(GPIO_EC_JTAG_TDO,
					value & ADBUS_TDO);
		} else
			gpio_set_flags(GPIO_EC_JTAG_TDO, GPIO_INPUT);

		if (direction & ADBUS_TMS) {
			if ((STM32_GPIO_MODER(GPIO_C) & (3 << (7 * 2)))
				!= (1 << (7 * 2)))
				gpio_set_flags(GPIO_EC_JTAG_TMS,
					(value & ADBUS_TMS) ?
					GPIO_OUT_HIGH : GPIO_OUT_LOW);
			else
				gpio_set_level(GPIO_EC_JTAG_TMS,
					value & ADBUS_TMS);
		} else
			gpio_set_flags(GPIO_EC_JTAG_TMS, GPIO_INPUT);

		/* input only pin */
		if (!(direction & ADBUS_JTAG_OE))
			gpio_set_flags(GPIO_PD_DEBUG_EN, GPIO_INPUT);
	} else {
		/* inverted value */
		if (direction & ACBUS_SRST_OUT) {
			if ((STM32_GPIO_MODER(GPIO_C) & (3 << (13 * 2)))
				!= (1 << (13 * 2)))
				gpio_set_flags(GPIO_EC_RST_L,
					(value & ACBUS_SRST_OUT) ?
					GPIO_OUT_LOW : GPIO_OUT_HIGH);
			else
				gpio_set_level(GPIO_EC_RST_L,
					~(value & ACBUS_SRST_OUT));
		} else
			gpio_set_flags(GPIO_EC_RST_L, GPIO_INPUT);
	}
}

static void ftdi_set_clock(uint8_t low, uint8_t high)
{
	int clk = (clk_div5 ? 12000000 : 60000000) /
		(2 * (1 + ((high << 8) | low)));
	CPRINTF("clock: %d\n", clk);
}

/*
 * Bitbangs an operation.
 * Length argument is in bits.
 */
static void ftdi_bitbang(uint8_t *args, int bits)
{
	int i;

	for (i = 0; i < 2 * bits; i++) {
		/* order of operations is write, +CE, read, -CE  */
		if (!(i % 2)) {
			if (cmd_buf[0] & USB_FTDI_CMD_BIT_WRITE_TMS) {
				gpio_set_level(GPIO_EC_JTAG_TMS,
					args[i / 16] &
					(1 << ((i / 2) % 8)));
			} else if (cmd_buf[0] & USB_FTDI_CMD_BIT_WRITE_TDI) {
				gpio_set_level(GPIO_EC_JTAG_TDI,
					args[i / 16] &
					(1 << ((i / 2) % 8)));
			}
		} else {
			if (cmd_buf[0] & USB_FTDI_CMD_BIT_READ_TDO) {
				/* clear bit */
				args[i / 16] &=	~(1 << ((i / 2) % 8));
				args[i / 16] |=
					(gpio_get_level(GPIO_EC_JTAG_TDO) <<
					((i / 2) % 8));
			}
		}

		/* toggle GPIO_EC_JTAG_TCK for +/- CE */
		STM32_GPIO_BSRR(GPIO_C) |= (i % 2) ?
			(1 << (6 + 16)) : (1 << 6);
	}

	/* Wipe remaining bits */
	if (bits % 8) {
		for (i = bits; i < bits + (8 - (bits % 8)); i++)
			args[i / 8] &= ~(1 << (i % 8));
	}
}

/*
 * Executes commands from local buffer.
 */
void ftdi_execute(void)
{
/* NOTE: FT2232 TMS bug is not emulated
 * http://developer.intra2net.com/mailarchive/html/libftdi/
 * 2009/msg00292.html
 */
	/* not special command */
	if (!(cmd_buf[0] & USB_FTDI_CMD_BIT_SPECIAL)) {
		/* only support write on -CE */
		ASSERT(cmd_buf[0] & USB_FTDI_CMD_BIT_WRITE_MCE);

		/* only support read on +CE */
		ASSERT(!(cmd_buf[0] & USB_FTDI_CMD_BIT_READ_MCE));

		/* only support LSB first */
		ASSERT(cmd_buf[0] & USB_FTDI_CMD_BIT_LSB_FIRST);

		/* bit mode command */
		if (cmd_buf[0] & USB_FTDI_CMD_BIT_BIT_MODE) {
			if (cmd_buf[0] & USB_FTDI_CMD_BIT_WRITE_TMS)
				gpio_set_level(GPIO_EC_JTAG_TDI,
					cmd_buf[2] & (1 << 7));

			ftdi_bitbang(cmd_buf + 2, cmd_buf[1] + 1);

			if (cmd_buf[0] & USB_FTDI_CMD_BIT_READ_TDO)
				ftdi_send_queue(cmd_buf + 2, 1);
		/* byte mode command */
		} else {
			ftdi_bitbang(cmd_buf + 3,
				8 * (1 + ((cmd_buf[2] << 8) | cmd_buf[1])));

			if (cmd_buf[0] & USB_FTDI_CMD_BIT_READ_TDO) {
				ftdi_send_queue(cmd_buf + 3,
					1 + ((cmd_buf[2] << 8) | cmd_buf[1]));
				/* force immediate send */
				ftdi_send_wait();
			}
		}
	} else {
		switch (cmd_buf[0]) {
		case USB_FTDI_CMD_WRITE_LOW_BYTE:
		case USB_FTDI_CMD_WRITE_HIGH_BYTE:
			ftdi_set_byte((cmd_buf[0] & 0x2) ? 1 : 0, cmd_buf[1],
				cmd_buf[2]);
		break;

		case USB_FTDI_CMD_SET_CLK_DIV:
			ftdi_set_clock(cmd_buf[1], cmd_buf[2]);
		break;
		}
	}
}

/*
 * Segments commands into local buffer from USB buffer.
 */
void ftdi_task(void)
{
	uint8_t *msg = (uint8_t *) ftdi_ep_rx;
	unsigned int pos, len, read_len;

	while (1) {
		/* wait for event or usb reset */
		task_wait_event(-1);

		pos = 0;
		len = btable_ep[USB_EP_FTDI_RX].rx_count & 0x3ff;

		/* finish reassembling previous command */
		if (cmd_pos < cmd_len) {
			/* length field was missing earlier */
			if (cmd_len == -1 &&
			   (!(cmd_buf[0] & USB_FTDI_CMD_BIT_BIT_MODE))) {
				cmd_len = 3 + 1 +
				((msg[2 - cmd_pos] << 8) | msg[1 - cmd_pos]);
			}

			ASSERT(cmd_len < USB_FTDI_BUFFER_SIZE);

			read_len = MIN(cmd_len - cmd_pos, len);
			memcpy(cmd_buf + cmd_pos,
				ftdi_ep_rx, read_len * sizeof(*msg));

			cmd_pos += read_len;
			pos = read_len;

			/* execute if done */
			if (cmd_pos >= cmd_len)
				ftdi_execute();
		}

		/* segment commands in buffer */
		while (pos < len) {
			if (!(msg[pos] & USB_FTDI_CMD_BIT_SPECIAL) ||
			      msg[pos] == USB_FTDI_CMD_SET_CLK_DIV ||
			      msg[pos] == USB_FTDI_CMD_WRITE_LOW_BYTE ||
			      msg[pos] == USB_FTDI_CMD_WRITE_HIGH_BYTE) {
				/* segment the above three special commands as regular
				 * regular bit commands */
				if (!(msg[pos] & USB_FTDI_CMD_BIT_SPECIAL) &&
				    !(msg[pos] & USB_FTDI_CMD_BIT_BIT_MODE)) {
					/* check for length field */
					cmd_len = (len - pos >= 3) ?
						(3 + 1 + ((msg[pos + 2] << 8)
						| msg[pos + 1])) : -1;
				} else
					cmd_len = 3;

				read_len = MIN(cmd_len, len - pos);

				memcpy(cmd_buf, msg + pos,
					read_len * sizeof(*msg));

				cmd_pos = read_len;
				pos += read_len;

				if (cmd_pos >= cmd_len)
					ftdi_execute();

			} else {
				switch (msg[pos]) {
				case USB_FTDI_CMD_CLK_DIV_5_OFF:
				case USB_FTDI_CMD_CLK_DIV_5_ON:
					clk_div5 = msg[pos] & (1 << 0);
				case USB_FTDI_CMD_ADAPT_CLOCK_OFF:
				case USB_FTDI_CMD_LOOPBACK_OFF:
				case USB_FTDI_CMD_SEND_IMMEDIATE:
					pos += 1;
				break;

				default:
					CPRINTF("bad cmd: %x\n", msg[pos]);

					cmd_buf[0] = USB_FTDI_CMD_BAD;
					cmd_buf[1] = msg[pos];

					ftdi_send_queue(cmd_buf, 2);
					ftdi_send_wait();

					pos += 1;
				break;
				}
			}
		}

		/* send if anything in buffer */
		ftdi_send_wait();

		/* receive next packet */
		STM32_TOGGLE_EP(USB_EP_FTDI_RX, EP_RX_MASK, EP_RX_VALID, 0);
	}
}
