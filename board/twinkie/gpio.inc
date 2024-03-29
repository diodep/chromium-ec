/* -*- mode:c -*-
 *
 * Copyright (c) 2014 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */
GPIO(CC2_ALERT_L,  A, 7,  GPIO_INT_FALLING, cc2_event)
GPIO(VBUS_ALERT_L, B, 2,  GPIO_INT_FALLING, vbus_event)

GPIO(CC1_EN,       A, 0,  GPIO_OUT_HIGH, NULL)
GPIO(CC1_PD,       A, 1,  GPIO_ANALOG,   NULL)
GPIO(CC2_EN,       A, 2,  GPIO_OUT_HIGH, NULL)
GPIO(CC2_PD,       A, 3,  GPIO_ANALOG,   NULL)
GPIO(DAC,          A, 4,  GPIO_ANALOG,   NULL)
GPIO(CC2_TX_DATA,  A, 6,  GPIO_OUT_LOW,  NULL)

GPIO(CC1_RA,       A, 8,  GPIO_ODR_HIGH, NULL)
GPIO(USB_DM,       A, 11, GPIO_ANALOG,   NULL)
GPIO(USB_DP,       A, 12, GPIO_ANALOG,   NULL)
GPIO(CC1_RPUSB,    A, 13, GPIO_ODR_HIGH, NULL)
GPIO(CC1_RP1A5,    A, 14, GPIO_ODR_HIGH, NULL)
GPIO(CC1_RP3A0,    A, 15, GPIO_ODR_HIGH, NULL)
GPIO(CC2_RPUSB,    B, 0,  GPIO_ODR_HIGH, NULL)

GPIO(CC1_TX_EN,    B, 1,  GPIO_OUT_LOW,  NULL)
GPIO(CC2_TX_EN,    B, 3,  GPIO_OUT_LOW,  NULL)
GPIO(CC1_TX_DATA,  B, 4,  GPIO_OUT_LOW,  NULL)
GPIO(CC1_RD,       B, 5,  GPIO_ODR_HIGH, NULL)
GPIO(I2C_SCL,      B, 6,  GPIO_INPUT,    NULL)
GPIO(I2C_SDA,      B, 7,  GPIO_INPUT,    NULL)
GPIO(CC2_RD,       B, 8,  GPIO_ODR_HIGH, NULL)
GPIO(LED_G_L,      B, 11, GPIO_ODR_HIGH, NULL)
GPIO(LED_R_L,      B, 13, GPIO_ODR_HIGH, NULL)
GPIO(LED_B_L,      B, 14, GPIO_ODR_HIGH, NULL)
GPIO(CC2_RA,       B, 15, GPIO_ODR_HIGH, NULL)
GPIO(CC2_RP1A5,    C, 14, GPIO_ODR_HIGH, NULL)
GPIO(CC2_RP3A0,    C, 15, GPIO_ODR_HIGH, NULL)

/* Unimplemented signals which we need to emulate for now */
UNIMPLEMENTED(ENTERING_RW)
UNIMPLEMENTED(WP_L)

ALTERNATE(A, 0x0020, 0, MODULE_USB_PD, 0)            /* SPI1: SCK(PA5) */
ALTERNATE(B, 0x0200, 2, MODULE_USB_PD, 0)            /* TIM17_CH1: PB9 */
ALTERNATE(A, 0x0600, 1, MODULE_UART,   GPIO_PULL_UP) /* USART1: PA9/PA10 */
ALTERNATE(B, 0x00C0, 1, MODULE_I2C,    0)            /* I2C1 MASTER:PB6/7 */
