/* Copyright (c) 2014 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* USB Power delivery board configuration */

#ifndef __USB_PD_CONFIG_H
#define __USB_PD_CONFIG_H

/* USB-PD configuration */
#define PD_PORT_COUNT 1
#define PORT_TO_TASK_ID(port) TASK_ID_PD
#define TASK_ID_TO_PORT(id)   0

/* Timer selection for baseband PD communication */
#define TIM_CLOCK_PD_TX_C0 17
#define TIM_CLOCK_PD_RX_C0 1

#define TIM_CLOCK_PD_TX(p) TIM_CLOCK_PD_TX_C0
#define TIM_CLOCK_PD_RX(p) TIM_CLOCK_PD_RX_C0

/* Timer channel */
#define TIM_RX_CCR_C0 1

/* RX timer capture/compare register */
#define TIM_CCR_C0 (&STM32_TIM_CCRx(TIM_CLOCK_PD_RX_C0, TIM_RX_CCR_C0))
#define TIM_RX_CCR_REG(p) TIM_CCR_C0

/* TX and RX timer register */
#define TIM_REG_TX_C0 (STM32_TIM_BASE(TIM_CLOCK_PD_TX_C0))
#define TIM_REG_RX_C0 (STM32_TIM_BASE(TIM_CLOCK_PD_RX_C0))
#define TIM_REG_TX(p) TIM_REG_TX_C0
#define TIM_REG_RX(p) TIM_REG_RX_C0

/* use the hardware accelerator for CRC */
#define CONFIG_HW_CRC

/* TX is using SPI1 on PA4-7 */
#define SPI_REGS(p) STM32_SPI1_REGS

static inline void spi_enable_clock(int port)
{
	STM32_RCC_APB2ENR |= STM32_RCC_PB2_SPI1;
}

#define DMAC_SPI_TX(p) STM32_DMAC_CH3

/* RX is using COMP1 triggering TIM1 CH1 */
#define CMP1OUTSEL STM32_COMP_CMP1OUTSEL_TIM1_IC1
#define CMP2OUTSEL 0

#define TIM_CCR_IDX(p) TIM_RX_CCR_C0
#define TIM_CCR_CS  1
#define EXTI_COMP_MASK(p) (1 << 21)
#define IRQ_COMP STM32_IRQ_COMP
/* triggers packet detection on comparator falling edge */
#define EXTI_XTSR STM32_EXTI_FTSR

#define DMAC_TIM_RX(p) STM32_DMAC_CH2

/* the pins used for communication need to be hi-speed */
static inline void pd_set_pins_speed(int port)
{
	/* 40 MHz pin speed on SPI1 PA5/6 */
	STM32_GPIO_OSPEEDR(GPIO_A) |= 0x00003C00;
	/* 40 MHz pin speed on TIM17_CH1 (PB9) */
	STM32_GPIO_OSPEEDR(GPIO_B) |= 0x000C0000;
}

/* Reset SPI peripheral used for TX */
static inline void pd_tx_spi_reset(int port)
{
	/* Reset SPI1 */
	STM32_RCC_APB2RSTR |= (1 << 12);
	STM32_RCC_APB2RSTR &= ~(1 << 12);
}

/* Drive the CC line from the TX block */
static inline void pd_tx_enable(int port, int polarity)
{
	/* put SPI function on TX pin */
	/* PA6 is SPI1 MISO */
	gpio_set_alternate_function(GPIO_A, 0x0040, 0);

	/* set the low level reference */
	gpio_set_level(GPIO_USBC_CC_TX_EN, 1);
}

/* Put the TX driver in Hi-Z state */
static inline void pd_tx_disable(int port, int polarity)
{
	/* output low on SPI TX to disable the FET */
	/* PA6 is SPI1_MISO */
	STM32_GPIO_MODER(GPIO_A) = (STM32_GPIO_MODER(GPIO_A)
				   & ~(3 << (2*6)))
				   |  (1 << (2*6));
	/* put the low level reference in Hi-Z */
	gpio_set_level(GPIO_USBC_CC_TX_EN, 0);
}

/* we know the plug polarity, do the right configuration */
static inline void pd_select_polarity(int port, int polarity)
{
	/* use the right comparator non inverted input for COMP1 */
	STM32_COMP_CSR = (STM32_COMP_CSR & ~STM32_COMP_CMP1INSEL_MASK)
			| STM32_COMP_CMP1EN
			| STM32_COMP_CMP1INSEL_INM6;
}

/* Initialize pins used for TX and put them in Hi-Z */
static inline void pd_tx_init(void)
{
	/* Configure SCK pin */
	gpio_config_module(MODULE_USB_PD, 1);
}

static inline void pd_set_host_mode(int port, int enable)
{
	ccprintf("Host mode: %d\n", enable);
	if (enable) {
		/* Source mode, disable charging */
		gpio_set_level(GPIO_USBC_CHARGE_EN, 0);
		/* High Z for no pull-down resistor on CC */
		gpio_set_flags_by_mask(GPIO_A, (1 << 9), GPIO_INPUT);
		/* Set pull-up resistor on CC */
		gpio_set_flags_by_mask(GPIO_A, (1 << 2), GPIO_OUT_HIGH);
	} else {
		/* Device mode, disable VBUS */
		gpio_set_level(GPIO_USBC_5V_EN, 0);
		gpio_set_level(GPIO_USBC_12V_EN, 0);
		/* High Z for no pull-up resistor on CC */
		gpio_set_flags_by_mask(GPIO_A, (1 << 2), GPIO_INPUT);
		/* Set pull-down resistor on CC. */
		gpio_set_flags_by_mask(GPIO_A, (1 << 9), GPIO_OUT_LOW);
		/* Set charge enable */
		gpio_set_level(GPIO_USBC_CHARGE_EN, 1);
	}
}

static inline int pd_adc_read(int port, int cc)
{
	/* Always return CC1 */
	return adc_read_channel(ADC_CH_CC1_PD);
}

static inline int pd_snk_is_vbus_provided(int port)
{
	return gpio_get_level(GPIO_VBUS_WAKE);
}

/* 3.0A DFP : no-connect voltage is 2.45V */
#define PD_SRC_VNC 2450 /* mV */

/* UFP-side : threshold for DFP connection detection */
#define PD_SNK_VA   250 /* mV */

/* we are acting only as a sink */
#define PD_DEFAULT_STATE PD_STATE_SNK_DISCONNECTED

/* delay necessary for the voltage transition on the power supply */
#define PD_POWER_SUPPLY_TRANSITION_DELAY 50000 /* us */

#endif /* __USB_PD_CONFIG_H */
