/* Copyright (c) 2013 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* Pit board configuration */

#ifndef __BOARD_H
#define __BOARD_H

/* 16 MHz SYSCLK clock frequency */
#define CPU_CLOCK 16000000

/* Use USART1 as console serial port */
#define CONFIG_CONSOLE_UART 1

/* Debug features */
#define CONFIG_ASSERT_HELP
#define CONFIG_CONSOLE_CMDHELP
#define CONFIG_PANIC_HELP
#undef  CONFIG_TASK_PROFILING

/* Optional features */
#define CONFIG_BATTERY_BQ20Z453
#define CONFIG_BOARD_POST_GPIO_INIT
#define CONFIG_CHIPSET_GAIA
#define CONFIG_CMD_PMU
#define CONFIG_EXTPOWER_SNOW
#define CONFIG_HOST_COMMAND_STATUS
#define CONFIG_I2C
#define CONFIG_KEYBOARD_PROTOCOL_MKBP
#define CONFIG_PMU_BOARD_INIT
#define CONFIG_PMU_HARD_RESET
#define CONFIG_PMU_TPS65090
#define CONFIG_SMART_BATTERY


#ifdef PORT_TO_PIT
/* TODO(rspangler): enable these features when they compile */
#define CONFIG_LOW_POWER_IDLE
#define CONFIG_SPI
#define CONFIG_WATCHDOG_HELP
#endif


#ifndef __ASSEMBLER__

/* By default, enable all console messages except keyboard */
#define CC_DEFAULT	(CC_ALL & ~CC_MASK(CC_KEYSCAN))

#define USB_CHARGE_PORT_COUNT 0

/* Keyboard output port list */
#define KB_OUT_PORT_LIST GPIO_A, GPIO_B, GPIO_C

/* Charging */
#define I2C_PORT_HOST 0
#define I2C_PORT_BATTERY I2C_PORT_HOST
#define I2C_PORT_CHARGER I2C_PORT_HOST
#define I2C_PORT_SLAVE 1

/* Timer selection */
#define TIM_CLOCK_MSB 3
#define TIM_CLOCK_LSB 4

/* GPIO signal list */
enum gpio_signal {
	/* Inputs with interrupt handlers are first for efficiency */
	GPIO_KB_PWR_ON_L = 0,
	GPIO_PP1800_LDO2,
	GPIO_SOC1V8_XPSHOLD,
	GPIO_CHARGER_INT,
	GPIO_LID_OPEN,
	GPIO_SUSPEND_L,
	/* Keyboard inputs */
	GPIO_KB_IN00,
	GPIO_KB_IN01,
	GPIO_KB_IN02,
	GPIO_KB_IN03,
	GPIO_KB_IN04,
	GPIO_KB_IN05,
	GPIO_KB_IN06,
	GPIO_KB_IN07,
	/* Other inputs */
	GPIO_AC_PWRBTN_L,
	GPIO_WP_L,
	/* Outputs */
	GPIO_AC_STATUS,
	GPIO_AP_RESET_L,
	GPIO_CHARGER_EN,
	GPIO_EC_INT,
	GPIO_EN_PP1350,
	GPIO_EN_PP3300,
	GPIO_EN_PP5000,
	GPIO_ENTERING_RW,
	GPIO_I2C1_SCL,
	GPIO_I2C1_SDA,
	GPIO_I2C2_SCL,
	GPIO_I2C2_SDA,
	GPIO_LED_POWER_L,
	GPIO_PMIC_PWRON_L,
	GPIO_PMIC_RESET,
#ifndef CONFIG_SPI
	GPIO_SPI1_MISO,
	GPIO_SPI1_NSS,
#endif
	GPIO_KB_OUT00,
	GPIO_KB_OUT01,
	GPIO_KB_OUT02,
	GPIO_KB_OUT03,
	GPIO_KB_OUT04,
	GPIO_KB_OUT05,
	GPIO_KB_OUT06,
	GPIO_KB_OUT07,
	GPIO_KB_OUT08,
	GPIO_KB_OUT09,
	GPIO_KB_OUT10,
	GPIO_KB_OUT11,
	GPIO_KB_OUT12,
	/* Number of GPIOs; not an actual GPIO */
	GPIO_COUNT
};

#endif /* !__ASSEMBLER__ */

#endif /* __BOARD_H */
