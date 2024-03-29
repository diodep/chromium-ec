/* Copyright (c) 2014 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* Host command module for PD MCU */

#include "charge_state.h"
#include "common.h"
#include "console.h"
#include "host_command.h"
#include "task.h"
#include "timer.h"
#include "util.h"

#define CPRINTS(format, args...) cprints(CC_PD_HOST_CMD, format, ## args)

#define TASK_EVENT_EXCHANGE_PD_STATUS  TASK_EVENT_CUSTOM(1)

void host_command_pd_send_status(void)
{
	task_set_event(TASK_ID_PDCMD, TASK_EVENT_EXCHANGE_PD_STATUS, 0);
}

static void pd_exchange_status(void)
{
	struct ec_params_pd_status ec_status;
	struct ec_response_pd_status pd_status;
	int rv = 0, tries = 0;

	/* Send battery state of charge */
	if (charge_get_flags() & CHARGE_FLAG_BATT_RESPONSIVE)
		ec_status.batt_soc = charge_get_percent();
	else
		ec_status.batt_soc = -1;

	/* Try 3 times to get the PD MCU status. */
	while (tries++ < 3) {
		rv = pd_host_command(EC_CMD_PD_EXCHANGE_STATUS, 0, &ec_status,
			     sizeof(struct ec_params_pd_status), &pd_status,
			     sizeof(struct ec_response_pd_status));
		if (rv >= 0)
			break;
		task_wait_event(500*MSEC);
	}

	if (rv < 0) {
		CPRINTS("Host command to PD MCU failed");
		return;
	}

	/* Set input current limit */
#ifdef BOARD_SAMUS
	/*
	 * TODO(crosbug.com/p/28532): Remove this workaround for Samus p2b
	 * boards which cannot correctly limit input current.
	 */
	pd_status.curr_lim_ma = pd_status.curr_lim_ma * 2 / 3;
#endif
	rv = charge_set_input_current_limit(MAX(pd_status.curr_lim_ma,
					CONFIG_CHARGER_INPUT_CURRENT));
	if (rv < 0)
		CPRINTS("Failed to set input current limit from PD MCU");
}

void pd_command_task(void)
{

	while (1) {
		/* Wait for the next command event */
		int evt = task_wait_event(-1);

		/* Process event to send status to PD */
		if (evt & TASK_EVENT_EXCHANGE_PD_STATUS)
			pd_exchange_status();
	}
}

