/* Copyright (c) 2014 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "adc.h"
#include "board.h"
#include "common.h"
#include "console.h"
#include "gpio.h"
#include "hooks.h"
#include "registers.h"
#include "task.h"
#include "timer.h"
#include "util.h"
#include "usb_pd.h"

#define CPRINTS(format, args...) cprints(CC_USBPD, format, ## args)

/* Acceptable margin between requested VBUS and measured value */
#define MARGIN_MV 400 /* mV */

/* Source PDOs */
const uint32_t pd_src_pdo[] = {
		PDO_FIXED(5000,   500, PDO_FIXED_EXTERNAL),
		PDO_FIXED(12000, 3000, PDO_FIXED_EXTERNAL),
};
static const int pd_src_pdo_cnts[2] = {
		[SRC_CAP_5V] = 1,
		[SRC_CAP_12V] = 2,
};

static int pd_src_pdo_idx;

/* Fake PDOs : we just want our pre-defined voltages */
const uint32_t pd_snk_pdo[] = {
		PDO_FIXED(5000,   500, 0),
		PDO_FIXED(12000,  500, 0),
		PDO_FIXED(20000,  500, 0),
};
const int pd_snk_pdo_cnt = ARRAY_SIZE(pd_snk_pdo);

/* Desired voltage requested as a sink (in millivolts) */
static unsigned select_mv = 5000;

void board_set_source_cap(enum board_src_cap cap)
{
	pd_src_pdo_idx = cap;
}

int pd_get_source_pdo(const uint32_t **src_pdo)
{
	*src_pdo = pd_src_pdo;
	return pd_src_pdo_cnts[pd_src_pdo_idx];
}

int pd_choose_voltage(int cnt, uint32_t *src_caps, uint32_t *rdo)
{
	int i;
	int ma;
	int set_mv = select_mv;

	/* Default to 5V */
	if (set_mv <= 0)
		set_mv = 5000;

	/* Get the selected voltage */
	for (i = cnt; i >= 0; i--) {
		int mv = ((src_caps[i] >> 10) & 0x3FF) * 50;
		int type = src_caps[i] & PDO_TYPE_MASK;
		if ((mv == set_mv) && (type == PDO_TYPE_FIXED))
			break;
	}
	if (i < 0)
		return -EC_ERROR_UNKNOWN;

	/* request all the power ... */
	ma = 10 * (src_caps[i] & 0x3FF);
	*rdo = RDO_FIXED(i + 1, ma, ma, 0);
	ccprintf("Request [%d] %dV %dmA\n", i, set_mv/1000, ma);
	return ma;
}

void pd_set_input_current_limit(uint32_t max_ma)
{
	/* No battery, nothing to do */
	return;
}

void pd_set_max_voltage(unsigned mv)
{
	select_mv = mv;
}

int requested_voltage_idx;
int pd_request_voltage(uint32_t rdo)
{
	int op_ma = rdo & 0x3FF;
	int max_ma = (rdo >> 10) & 0x3FF;
	int idx = rdo >> 28;
	uint32_t pdo;
	uint32_t pdo_ma;

	if (!idx || idx > pd_src_pdo_cnts[pd_src_pdo_idx])
		return EC_ERROR_INVAL; /* Invalid index */

	/* check current ... */
	pdo = pd_src_pdo[idx - 1];
	pdo_ma = (pdo & 0x3ff);
	if (op_ma > pdo_ma)
		return EC_ERROR_INVAL; /* too much op current */
	if (max_ma > pdo_ma)
		return EC_ERROR_INVAL; /* too much max current */

	ccprintf("Switch to %d V %d mA (for %d/%d mA)\n",
		 ((pdo >> 10) & 0x3ff) * 50, (pdo & 0x3ff) * 10,
		 ((rdo >> 10) & 0x3ff) * 10, (rdo & 0x3ff) * 10);

	requested_voltage_idx = idx;

	return EC_SUCCESS;
}

int pd_set_power_supply_ready(int port)
{
	/* Output the correct voltage */
	gpio_set_level(requested_voltage_idx ? GPIO_USBC_12V_EN :
					       GPIO_USBC_5V_EN, 1);

	return EC_SUCCESS;
}

void pd_power_supply_reset(int port)
{
	/* Kill VBUS */
	requested_voltage_idx = 0;
	gpio_set_level(GPIO_USBC_5V_EN, 0);
	gpio_set_level(GPIO_USBC_12V_EN, 0);
}

int pd_board_checks(void)
{
	return EC_SUCCESS;
}

