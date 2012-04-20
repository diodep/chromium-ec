/* Copyright (c) 2012 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* Clocks and power management settings */

#include "board.h"
#include "clock.h"
#include "config.h"
#include "console.h"
#include "gpio.h"
#include "hooks.h"
#include "registers.h"
#include "system.h"
#include "task.h"
#include "timer.h"
#include "uart.h"
#include "util.h"

#define PLL_CLOCK 66666667  /* System clock = 200MHz PLL/3 = 66.667MHz */

/* Disable the PLL; run off internal oscillator. */
static void clock_disable_pll(void)
{
	/* Switch to 16MHz internal oscillator and power down the PLL */
	LM4_SYSTEM_RCC = LM4_SYSTEM_RCC_SYSDIV(0) |
		LM4_SYSTEM_RCC_BYPASS |
		LM4_SYSTEM_RCC_PWRDN |
		LM4_SYSTEM_RCC_OSCSRC(1) |
		LM4_SYSTEM_RCC_MOSCDIS;
	LM4_SYSTEM_RCC2 &= ~LM4_SYSTEM_RCC2_USERCC2;
}


/* Enable the PLL to run at full clock speed */
static void clock_enable_pll(void)
{
	/* Disable the PLL so we can reconfigure it */
	clock_disable_pll();

	/* Enable the PLL (PWRDN is no longer set) and set divider.  PLL is
	 * still bypassed, since it hasn't locked yet. */
	LM4_SYSTEM_RCC = LM4_SYSTEM_RCC_SYSDIV(2) |
		LM4_SYSTEM_RCC_USESYSDIV |
		LM4_SYSTEM_RCC_BYPASS |
		LM4_SYSTEM_RCC_OSCSRC(1) |
		LM4_SYSTEM_RCC_MOSCDIS;

	/* Wait for the PLL to lock */
	clock_wait_cycles(1024);
	while (!(LM4_SYSTEM_PLLSTAT & 1))
		;

	/* Remove bypass on PLL */
	LM4_SYSTEM_RCC &= ~LM4_SYSTEM_RCC_BYPASS;
}


void clock_wait_cycles(uint32_t cycles)
{
	asm("1: subs %0, #1\n"
	    "   bne 1b\n" :: "r"(cycles));
}


int clock_get_freq(void)
{
	return (LM4_SYSTEM_PLLSTAT & 1) ? PLL_CLOCK : INTERNAL_CLOCK;
}


/*****************************************************************************/
/* Console commands */

/* Function to measure baseline for power consumption.
 *
 * Levels :
 *   0 : CPU running in tight loop
 *   1 : CPU running in tight loop but peripherals gated
 *   2 : CPU in sleep mode
 *   3 : CPU in sleep mode and peripherals gated
 *   4 : CPU in deep sleep mode
 *   5 : CPU in deep sleep mode and peripherals gated */
static int command_sleep(int argc, char **argv)
{
	int level = 0;
	int clock = 0;
	uint32_t uartibrd = 0;
	uint32_t uartfbrd = 0;

	if (argc >= 2) {
		level = strtoi(argv[1], NULL, 10);
	}
	if (argc >= 3) {
		clock = strtoi(argv[2], NULL, 10);
	}

#ifdef BOARD_bds
	/* remove LED current sink  */
	gpio_set_level(GPIO_DEBUG_LED, 0);
#endif

	uart_printf("Going to sleep : level %d clock %d...\n", level, clock);
	uart_flush_output();

	/* clock setting */
	if (clock) {
		/* Use ROM code function to set the clock */
		void **func_table = (void **)*(uint32_t *)0x01000044;
		void (*rom_clock_set)(uint32_t rcc) = func_table[23];

		/* disable interrupts */
		asm volatile("cpsid i");

		switch (clock) {
		case 1: /* 16MHz IOSC */
			uartibrd = 17;
			uartfbrd = 23;
			rom_clock_set(0x00000d51);
			break;
		case 2: /* 1MHz IOSC */
			uartibrd = 1;
			uartfbrd = 5;
			rom_clock_set(0x07C00d51);
			break;
		case 3: /* 30 kHz */
			uartibrd = 0;
			uartfbrd = 0;
			rom_clock_set(0x00000d71);
			break;
		}

		/* TODO: move this to the UART module; ugly to have
		   UARTisms here.  Also note this only fixes UART0,
		   not UART1. */
		if (uartfbrd) {
			/* Disable the port via UARTCTL and add HSE */
			LM4_UART_CTL(0) = 0x0320;
			/* Set the baud rate divisor */
			LM4_UART_IBRD(0) = uartibrd;
			LM4_UART_FBRD(0) = uartfbrd;
			/* Poke UARTLCRH to make the new divisor take effect. */
			LM4_UART_LCRH(0) = LM4_UART_LCRH(0);
			/* Enable the port */
			LM4_UART_CTL(0) |= 0x0001;
		}
		asm volatile("cpsie i");
	}

	if (uartfbrd) {
		uart_printf("We are still alive. RCC=%08x\n", LM4_SYSTEM_RCC);
		uart_flush_output();
	}

	asm volatile("cpsid i");

	/* gate peripheral clocks */
	if (level & 1) {
		LM4_SYSTEM_RCGCTIMER = 0;
		LM4_SYSTEM_RCGCGPIO = 0;
		LM4_SYSTEM_RCGCDMA = 0;
		LM4_SYSTEM_RCGCUART = 0;
		LM4_SYSTEM_RCGCLPC = 0;
		LM4_SYSTEM_RCGCWTIMER = 0;
	}
	/* set deep sleep bit */
	if (level >= 4)
		LM4_SCB_SYSCTRL |= 0x4;
	/* go to low power mode (forever ...) */
	if (level > 1)
		while (1)
			asm("wfi");
	else
		while(1);

	return EC_SUCCESS;
}
DECLARE_CONSOLE_COMMAND(sleep, command_sleep);


static int command_disable_pll(int argc, char **argv)
{
	/* Notify modules of frequency change */
	return hook_notify(HOOK_FREQ_CHANGE, 0);
}
DECLARE_CONSOLE_COMMAND(nopll, command_disable_pll);

/*****************************************************************************/
/* Initialization */

int clock_init(void)
{

#ifdef BOARD_bds
	/* Perform an auto calibration of the internal oscillator using the
	 * 32.768KHz hibernate clock, unless we've already done so.  This is
	 * only necessary on A2 silicon as on BDS; A3 silicon is all
	 * factory-trimmed. */
	if ((LM4_SYSTEM_PIOSCSTAT & 0x300) != 0x100) {
		/* Start calibration */
		LM4_SYSTEM_PIOSCCAL = 0x80000000;
		LM4_SYSTEM_PIOSCCAL = 0x80000200;
		/* Wait for result */
		clock_wait_cycles(16);
		while (!(LM4_SYSTEM_PIOSCSTAT & 0x300))
			;
	}
#else
	/* Only BDS has an external crystal; other boards don't have one, and
	 * can disable main oscillator control to reduce power consumption. */
	LM4_SYSTEM_MOSCCTL = 0x04;
#endif

	/* TODO: UART seems to glitch unless we wait 500k cycles before
	 * enabling the PLL, but only if this is a cold boot.  Why?  UART
	 * doesn't even use the PLL'd system clock.  I've heard rumors the
	 * Stellaris ROM library does this too, but why? */
	if (!system_jumped_to_this_image())
		clock_wait_cycles(500000);

	clock_enable_pll();

	return EC_SUCCESS;
}
