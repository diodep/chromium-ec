/* Copyright (c) 2013 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/io.h>
#include <unistd.h>

#include "battery.h"
#include "comm-host.h"
#include "compile_time_macros.h"
#include "ec_flash.h"
#include "ectool.h"
#include "lightbar.h"
#include "lock/gec_lock.h"
#include "misc_util.h"
#include "panic.h"

/* Command line options */
enum {
	OPT_DEV = 1000,
	OPT_INTERFACE,
};

static struct option long_opts[] = {
	{"dev", 1, 0, OPT_DEV},
	{"interface", 1, 0, OPT_INTERFACE},
	{NULL, 0, 0, 0}
};

#define GEC_LOCK_TIMEOUT_SECS	30  /* 30 secs */

const char help_str[] =
	"Commands:\n"
	"  extpwrcurrentlimit\n"
	"      Set the maximum external power current\n"
	"  autofanctrl <on>\n"
	"      Turn on automatic fan speed control.\n"
	"  backlight <enabled>\n"
	"      Enable/disable LCD backlight\n"
	"  battery\n"
	"      Prints battery info\n"
	"  batterycutoff [at-shutdown]\n"
	"      Cut off battery output power\n"
	"  batteryparam\n"
	"      Read or write board-specific battery parameter\n"
	"  boardversion\n"
	"      Prints the board version\n"
	"  chargecurrentlimit\n"
	"      Set the maximum battery charging current\n"
	"  chargecontrol\n"
	"      Force the battery to stop charging or discharge\n"
	"  chargestate\n"
	"      Handle commands related to charge state v2 (and later)\n"
	"  chipinfo\n"
	"      Prints chip info\n"
	"  cmdversions <cmd>\n"
	"      Prints supported version mask for a command number\n"
	"  console\n"
	"      Prints the last output to the EC debug console\n"
	"  echash [CMDS]\n"
	"      Various EC hash commands\n"
	"  eventclear <mask>\n"
	"      Clears EC host events flags where mask has bits set\n"
	"  eventclearb <mask>\n"
	"      Clears EC host events flags copy B where mask has bits set\n"
	"  eventget\n"
	"      Prints raw EC host event flags\n"
	"  eventgetb\n"
	"      Prints raw EC host event flags copy B\n"
	"  eventgetscimask\n"
	"      Prints SCI mask for EC host events\n"
	"  eventgetsmimask\n"
	"      Prints SMI mask for EC host events\n"
	"  eventgetwakemask\n"
	"      Prints wake mask for EC host events\n"
	"  eventsetscimask <mask>\n"
	"      Sets the SCI mask for EC host events\n"
	"  eventsetsmimask <mask>\n"
	"      Sets the SMI mask for EC host events\n"
	"  eventsetwakemask <mask>\n"
	"      Sets the wake mask for EC host events\n"
	"  fanduty <percent>\n"
	"      Forces the fan PWM to a constant duty cycle\n"
	"  flasherase <offset> <size>\n"
	"      Erases EC flash\n"
	"  flashinfo\n"
	"      Prints information on the EC flash\n"
	"  flashprotect [now] [enable | disable]\n"
	"      Prints or sets EC flash protection state\n"
	"  flashread <offset> <size> <outfile>\n"
	"      Reads from EC flash to a file\n"
	"  flashwrite <offset> <infile>\n"
	"      Writes to EC flash from a file\n"
	"  gpioget <GPIO name>\n"
	"      Get the value of GPIO signal\n"
	"  gpioset <GPIO name>\n"
	"      Set the value of GPIO signal\n"
	"  hangdetect <flags> <event_msec> <reboot_msec> | stop | start\n"
	"      Configure or start/stop the hang detect timer\n"
	"  hello\n"
	"      Checks for basic communication with EC\n"
	"  kbpress\n"
	"      Simulate key press\n"
	"  i2cread\n"
	"      Read I2C bus\n"
	"  i2cwrite\n"
	"      Write I2C bus\n"
	"  i2cxfer <port> <slave_addr> <read_count> [write bytes...]\n"
	"      Perform I2C transfer on EC's I2C bus\n"
	"  keyscan <beat_us> <filename>\n"
	"      Test low-level key scanning\n"
	"  led <name> <query | auto | off | <color> | <color>=<value>...>\n"
	"      Set the color of an LED or query brightness range\n"
	"  lightbar [CMDS]\n"
	"      Various lightbar control commands\n"
	"  motionsense [CMDS]\n"
	"      Various motion sense control commands\n"
	"  panicinfo\n"
	"      Prints saved panic info\n"
	"  pause_in_s5 [on|off]\n"
	"      Whether or not the AP should pause in S5 on shutdown\n"
	"  port80flood\n"
	"      Rapidly write bytes to port 80\n"
	"  port80read\n"
	"      Print history of port 80 write\n"
	"  powerinfo\n"
	"	Prints power-related information\n"
	"  protoinfo\n"
	"       Prints EC host protocol information\n"
	"  pstoreinfo\n"
	"      Prints information on the EC host persistent storage\n"
	"  pstoreread <offset> <size> <outfile>\n"
	"      Reads from EC host persistent storage to a file\n"
	"  pstorewrite <offset> <infile>\n"
	"      Writes to EC host persistent storage from a file\n"
	"  pwmgetfanrpm [<index> | all]\n"
	"      Prints current fan RPM\n"
	"  pwmgetkblight\n"
	"      Prints current keyboard backlight percent\n"
	"  pwmsetfanrpm <targetrpm>\n"
	"      Set target fan RPM\n"
	"  pwmsetkblight <percent>\n"
	"      Set keyboard backlight in percent\n"
	"  readtest <patternoffset> <size>\n"
	"      Reads a pattern from the EC via LPC\n"
	"  reboot_ec <RO|RW|cold|hibernate|disable-jump> [at-shutdown]\n"
	"      Reboot EC to RO or RW\n"
	"  rtcget\n"
	"      Print real-time clock\n"
	"  rtcset <time>\n"
	"      Set real-time clock\n"
	"  sertest\n"
	"      Serial output test for COM2\n"
	"  switches\n"
	"      Prints current EC switch positions\n"
	"  temps <sensorid>\n"
	"      Print temperature.\n"
	"  tempsinfo <sensorid>\n"
	"      Print temperature sensor info.\n"
	"  thermalget <platform-specific args>\n"
	"      Get the threshold temperature values from the thermal engine.\n"
	"  thermalset <platform-specific args>\n"
	"      Set the threshold temperature values for the thermal engine.\n"
	"  tmp006cal <tmp006_index> [<S0> <b0> <b1> <b2>]\n"
	"      Get/set TMP006 calibration\n"
	"  tmp006raw <tmp006_index>\n"
	"      Get raw TMP006 data\n"
	"  usbchargemode <port> <mode>\n"
	"      Set USB charging mode\n"
	"  usbmux <mux>\n"
	"      Set USB mux switch state\n"
	"  usbpd <port> <auto | "
			"[toggle|toggle-off|sink|source] [none|usb|dp|dock]>\n"
	"      Control USB PD/type-C\n"
	"  version\n"
	"      Prints EC version\n"
	"  wireless <flags> [<mask> [<suspend_flags> <suspend_mask>]]\n"
	"      Enable/disable WLAN/Bluetooth radio\n"
	"";

/* Note: depends on enum system_image_copy_t */
static const char * const image_names[] = {"unknown", "RO", "RW"};

/* Note: depends on enum ec_led_colors */
static const char * const led_color_names[EC_LED_COLOR_COUNT] = {
	"red", "green", "blue", "yellow", "white"};

/* Note: depends on enum ec_led_id */
static const char * const led_names[EC_LED_ID_COUNT] = {
	"battery", "power", "adapter"};

/* Check SBS numerical value range */
int is_battery_range(int val)
{
	return (val >= 0 && val <= 65535) ? 1 : 0;
}

int parse_bool(const char *s, int *dest)
{
	if (!strcasecmp(s, "off") || !strncasecmp(s, "dis", 3) ||
	    tolower(*s) == 'f' || tolower(*s) == 'n') {
		*dest = 0;
		return 1;
	} else if (!strcasecmp(s, "on") || !strncasecmp(s, "ena", 3) ||
	    tolower(*s) == 't' || tolower(*s) == 'y') {
		*dest = 1;
		return 1;
	} else {
		return 0;
	}
}

void print_help(const char *prog, int print_cmds)
{
	printf("Usage: %s [--dev=n] [--interface=dev|lpc|i2c] <command> "
	       "[params]\n\n",
	       prog);
	if (print_cmds)
		puts(help_str);
	else
		printf("Use '%s help' to print a list of commands.\n", prog);
}

static uint8_t read_mapped_mem8(uint8_t offset)
{
	int ret;
	uint8_t val;

	ret = ec_readmem(offset, sizeof(val), &val);
	if (ret <= 0) {
		fprintf(stderr, "failure in %s(): %d\n", __func__, ret);
		exit(1);
	}
	return val;
}

static uint16_t read_mapped_mem16(uint8_t offset)
{
	int ret;
	uint16_t val;

	ret = ec_readmem(offset, sizeof(val), &val);
	if (ret <= 0) {
		fprintf(stderr, "failure in %s(): %d\n", __func__, ret);
		exit(1);
	}
	return val;
}

static uint32_t read_mapped_mem32(uint8_t offset)
{
	int ret;
	uint32_t val;

	ret = ec_readmem(offset, sizeof(val), &val);
	if (ret <= 0) {
		fprintf(stderr, "failure in %s(): %d\n", __func__, ret);
		exit(1);
	}
	return val;
}

static int read_mapped_string(uint8_t offset, char *buffer, int max_size)
{
	int ret;

	ret = ec_readmem(offset, max_size, buffer);
	if (ret <= 0) {
		fprintf(stderr, "failure in %s(): %d\n", __func__, ret);
		exit(1);
	}
	return ret;
}

int cmd_hello(int argc, char *argv[])
{
	struct ec_params_hello p;
	struct ec_response_hello r;
	int rv;

	p.in_data = 0xa0b0c0d0;

	rv = ec_command(EC_CMD_HELLO, 0, &p, sizeof(p), &r, sizeof(r));
	if (rv < 0)
		return rv;

	if (r.out_data != 0xa1b2c3d4) {
		fprintf(stderr, "Expected response 0x%08x, got 0x%08x\n",
			0xa1b2c3d4, r.out_data);
		return -1;
	}

	printf("EC says hello!\n");
	return 0;
}

int cmd_test(int argc, char *argv[])
{
	struct ec_params_test_protocol p = {
		.buf = "0123456789abcdef0123456789ABCDEF"
	};
	struct ec_response_test_protocol r;
	int rv, version = 0;
	char *e;

	if (argc < 3) {
		fprintf(stderr, "Usage: %s result length [version]\n",
			argv[0]);
		return -1;
	}

	p.ec_result = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "invalid param (result)\n");
		return -1;
	}
	p.ret_len = strtol(argv[2], &e, 0);
	if (e && *e) {
		fprintf(stderr, "invalid param (length)\n");
		return -1;
	}

	if (argc > 3) {
		version = strtol(argv[3], &e, 0);
		if (e && *e) {
			fprintf(stderr, "invalid param (version)\n");
			return -1;
		}
	}

	rv = ec_command(EC_CMD_TEST_PROTOCOL, version,
			&p, sizeof(p), &r, sizeof(r));
	printf("rv = %d\n", rv);

	return rv;
}

int cmd_s5(int argc, char *argv[])
{
	struct ec_params_get_set_value p;
	struct ec_params_get_set_value r;
	int rv;

	p.flags = 0;

	if (argc > 1) {
		p.flags |= EC_GSV_SET;
		if (!parse_bool(argv[1], &p.value)) {
			fprintf(stderr, "invalid arg \"%s\"\n", argv[1]);
			return -1;
		}
	}

	rv = ec_command(EC_CMD_GSV_PAUSE_IN_S5, 0,
			&p, sizeof(p), &r, sizeof(r));
	if (rv > 0)
		printf("%s\n", r.value ? "on" : "off");

	return rv < 0;
}


int cmd_cmdversions(int argc, char *argv[])
{
	struct ec_params_get_cmd_versions p;
	struct ec_response_get_cmd_versions r;
	char *e;
	int cmd;
	int rv;

	if (argc < 2) {
		fprintf(stderr, "Usage: %s <cmd>\n", argv[0]);
		return -1;
	}
	cmd = strtol(argv[1], &e, 0);
	if ((e && *e) || cmd < 0 || cmd > 0xff) {
		fprintf(stderr, "Bad command number.\n");
		return -1;
	}

	p.cmd = cmd;
	rv = ec_command(EC_CMD_GET_CMD_VERSIONS, 0, &p, sizeof(p),
			&r, sizeof(r));
	if (rv < 0) {
		if (rv == -EC_RES_INVALID_PARAM)
			printf("Command 0x%02x not supported by EC.\n", cmd);

		return rv;
	}

	printf("Command 0x%02x supports version mask 0x%08x\n",
	       cmd, r.version_mask);
	return 0;
}

int cmd_version(int argc, char *argv[])
{
	struct ec_response_get_version r;
	char *build_string = (char *)ec_inbuf;
	int rv;

	rv = ec_command(EC_CMD_GET_VERSION, 0, NULL, 0, &r, sizeof(r));
	if (rv < 0) {
		fprintf(stderr, "ERROR: EC_CMD_GET_VERSION failed: %d\n", rv);
		return rv;
	}
	rv = ec_command(EC_CMD_GET_BUILD_INFO, 0,
			NULL, 0, ec_inbuf, ec_max_insize);
	if (rv < 0) {
		fprintf(stderr, "ERROR: EC_CMD_GET_BUILD_INFO failed: %d\n",
				rv);
		return rv;
	}

	/* Ensure versions are null-terminated before we print them */
	r.version_string_ro[sizeof(r.version_string_ro) - 1] = '\0';
	r.version_string_rw[sizeof(r.version_string_rw) - 1] = '\0';
	build_string[ec_max_insize - 1] = '\0';

	/* Print versions */
	printf("RO version:    %s\n", r.version_string_ro);
	printf("RW version:    %s\n", r.version_string_rw);
	printf("Firmware copy: %s\n",
	       (r.current_image < ARRAY_SIZE(image_names) ?
		image_names[r.current_image] : "?"));
	printf("Build info:    %s\n", build_string);

	return 0;
}


int cmd_read_test(int argc, char *argv[])
{
	struct ec_params_read_test p;
	struct ec_response_read_test r;
	int offset, size;
	int errors = 0;
	int rv;
	int i;
	char *e;
	char *buf;
	uint32_t *b;

	if (argc < 3) {
		fprintf(stderr, "Usage: %s <pattern_offset> <size>\n", argv[0]);
		return -1;
	}
	offset = strtol(argv[1], &e, 0);
	size = strtol(argv[2], &e, 0);
	if ((e && *e) || size <= 0 || size > 0x100000) {
		fprintf(stderr, "Bad size.\n");
		return -1;
	}
	printf("Reading %d bytes with pattern offset 0x%x...\n", size, offset);

	buf = (char *)malloc(size);
	if (!buf) {
		fprintf(stderr, "Unable to allocate buffer.\n");
		return -1;
	}

	/* Read data in chunks */
	for (i = 0; i < size; i += sizeof(r.data)) {
		p.offset = offset + i / sizeof(uint32_t);
		p.size = MIN(size - i, sizeof(r.data));
		rv = ec_command(EC_CMD_READ_TEST, 0, &p, sizeof(p),
				&r, sizeof(r));
		if (rv < 0) {
			fprintf(stderr, "Read error at offset %d\n", i);
			free(buf);
			return rv;
		}
		memcpy(buf + i, r.data, p.size);
	}

	/* Check data */
	for (i = 0, b = (uint32_t *)buf; i < size / 4; i++, b++) {
		if (*b != i + offset) {
			printf("Mismatch at byte offset 0x%x: "
			       "expected 0x%08x, got 0x%08x\n",
			       (int)(i * sizeof(uint32_t)), i + offset, *b);
			errors++;
		}
	}

	free(buf);
	if (errors) {
		printf("Found %d errors\n", errors);
		return -1;
	}

	printf("done.\n");
	return 0;
}


int cmd_reboot_ec(int argc, char *argv[])
{
	struct ec_params_reboot_ec p;
	int rv, i;

	if (argc < 2) {
		/*
		 * No params specified so tell the EC to reboot immediately.
		 * That reboots the AP as well, so unlikely we'll be around
		 * to see a return code from this...
		 */
		rv = ec_command(EC_CMD_REBOOT, 0, NULL, 0, NULL, 0);
		return (rv < 0 ? rv : 0);
	}

	/* Parse command */
	if (!strcmp(argv[1], "cancel"))
		p.cmd = EC_REBOOT_CANCEL;
	else if (!strcmp(argv[1], "RO"))
		p.cmd = EC_REBOOT_JUMP_RO;
	else if (!strcmp(argv[1], "RW") || !strcmp(argv[1], "A")) {
		/*
		 * TODO(crosbug.com/p/11149): remove "A" once all scripts are
		 * updated to use "RW".
		 */
		p.cmd = EC_REBOOT_JUMP_RW;
	} else if (!strcmp(argv[1], "cold"))
		p.cmd = EC_REBOOT_COLD;
	else if (!strcmp(argv[1], "disable-jump"))
		p.cmd = EC_REBOOT_DISABLE_JUMP;
	else if (!strcmp(argv[1], "hibernate"))
		p.cmd = EC_REBOOT_HIBERNATE;
	else {
		fprintf(stderr, "Unknown command: %s\n", argv[1]);
		return -1;
	}

	/* Parse flags, if any */
	p.flags = 0;
	for (i = 2; i < argc; i++) {
		if (!strcmp(argv[i], "at-shutdown"))
			p.flags |= EC_REBOOT_FLAG_ON_AP_SHUTDOWN;
		else {
			fprintf(stderr, "Unknown flag: %s\n", argv[i]);
			return -1;
		}
	}

	rv = ec_command(EC_CMD_REBOOT_EC, 0, &p, sizeof(p), NULL, 0);
	return (rv < 0 ? rv : 0);
}


int cmd_flash_info(int argc, char *argv[])
{
	struct ec_response_flash_info_1 r;
	int cmdver = 1;
	int rsize = sizeof(r);
	int rv;

	memset(&r, 0, sizeof(r));

	if (!ec_cmd_version_supported(EC_CMD_FLASH_INFO, cmdver)) {
		/* Fall back to version 0 command */
		cmdver = 0;
		rsize = sizeof(struct ec_response_flash_info);
	}

	rv = ec_command(EC_CMD_FLASH_INFO, cmdver, NULL, 0, &r, rsize);
	if (rv < 0)
		return rv;

	printf("FlashSize %d\nWriteSize %d\nEraseSize %d\nProtectSize %d\n",
	       r.flash_size, r.write_block_size, r.erase_block_size,
	       r.protect_block_size);

	if (cmdver >= 1) {
		/* Fields added in ver.1 available */
		printf("WriteIdealSize %d\nFlags 0x%x\n",
		       r.write_ideal_size, r.flags);
	}

	return 0;
}

int cmd_flash_read(int argc, char *argv[])
{
	int offset, size;
	int rv;
	char *e;
	char *buf;

	if (argc < 4) {
		fprintf(stderr,
			"Usage: %s <offset> <size> <filename>\n", argv[0]);
		return -1;
	}
	offset = strtol(argv[1], &e, 0);
	if ((e && *e) || offset < 0 || offset > 0x100000) {
		fprintf(stderr, "Bad offset.\n");
		return -1;
	}
	size = strtol(argv[2], &e, 0);
	if ((e && *e) || size <= 0 || size > 0x100000) {
		fprintf(stderr, "Bad size.\n");
		return -1;
	}
	printf("Reading %d bytes at offset %d...\n", size, offset);

	buf = (char *)malloc(size);
	if (!buf) {
		fprintf(stderr, "Unable to allocate buffer.\n");
		return -1;
	}

	/* Read data in chunks */
	rv = ec_flash_read(buf, offset, size);
	if (rv < 0) {
		free(buf);
		return rv;
	}

	rv = write_file(argv[3], buf, size);
	free(buf);
	if (rv)
		return rv;

	printf("done.\n");
	return 0;
}

int cmd_flash_write(int argc, char *argv[])
{
	int offset, size;
	int rv;
	char *e;
	char *buf;

	if (argc < 3) {
		fprintf(stderr, "Usage: %s <offset> <filename>\n", argv[0]);
		return -1;
	}

	offset = strtol(argv[1], &e, 0);
	if ((e && *e) || offset < 0 || offset > 0x100000) {
		fprintf(stderr, "Bad offset.\n");
		return -1;
	}

	/* Read the input file */
	buf = read_file(argv[2], &size);
	if (!buf)
		return -1;

	printf("Writing to offset %d...\n", offset);

	/* Write data in chunks */
	rv = ec_flash_write(buf, offset, size);

	free(buf);

	if (rv < 0)
		return rv;

	printf("done.\n");
	return 0;
}

int cmd_flash_erase(int argc, char *argv[])
{
	int offset, size;
	char *e;
	int rv;

	if (argc < 3) {
		fprintf(stderr, "Usage: %s <offset> <size>\n", argv[0]);
		return -1;
	}

	offset = strtol(argv[1], &e, 0);
	if ((e && *e) || offset < 0 || offset > 0x100000) {
		fprintf(stderr, "Bad offset.\n");
		return -1;
	}

	size = strtol(argv[2], &e, 0);
	if ((e && *e) || size <= 0 || size > 0x100000) {
		fprintf(stderr, "Bad size.\n");
		return -1;
	}

	printf("Erasing %d bytes at offset %d...\n", size, offset);
	rv = ec_flash_erase(offset, size);
	if (rv < 0)
		return rv;

	printf("done.\n");
	return 0;
}


static void print_flash_protect_flags(const char *desc, uint32_t flags)
{
	printf("%s 0x%08x", desc, flags);
	if (flags & EC_FLASH_PROTECT_GPIO_ASSERTED)
		printf(" wp_gpio_asserted");
	if (flags & EC_FLASH_PROTECT_RO_AT_BOOT)
		printf(" ro_at_boot");
	if (flags & EC_FLASH_PROTECT_ALL_AT_BOOT)
		printf(" all_at_boot");
	if (flags & EC_FLASH_PROTECT_RO_NOW)
		printf(" ro_now");
	if (flags & EC_FLASH_PROTECT_ALL_NOW)
		printf(" all_now");
	if (flags & EC_FLASH_PROTECT_ERROR_STUCK)
		printf(" STUCK");
	if (flags & EC_FLASH_PROTECT_ERROR_INCONSISTENT)
		printf(" INCONSISTENT");
	printf("\n");
}


int cmd_flash_protect(int argc, char *argv[])
{
	struct ec_params_flash_protect p;
	struct ec_response_flash_protect r;
	int rv, i;

	/*
	 * Set up requested flags.  If no flags were specified, p.mask will
	 * be 0 and nothing will change.
	 */
	p.mask = p.flags = 0;
	for (i = 1; i < argc; i++) {
		if (!strcasecmp(argv[i], "now")) {
			p.mask |= EC_FLASH_PROTECT_ALL_NOW;
			p.flags |= EC_FLASH_PROTECT_ALL_NOW;
		} else if (!strcasecmp(argv[i], "enable")) {
			p.mask |= EC_FLASH_PROTECT_RO_AT_BOOT;
			p.flags |= EC_FLASH_PROTECT_RO_AT_BOOT;
		} else if (!strcasecmp(argv[i], "disable"))
			p.mask |= EC_FLASH_PROTECT_RO_AT_BOOT;
	}

	rv = ec_command(EC_CMD_FLASH_PROTECT, EC_VER_FLASH_PROTECT,
			&p, sizeof(p), &r, sizeof(r));
	if (rv < 0)
		return rv;
	if (rv < sizeof(r)) {
		fprintf(stderr, "Too little data returned.\n");
		return -1;
	}

	/* Print returned flags */
	print_flash_protect_flags("Flash protect flags:", r.flags);
	print_flash_protect_flags("Valid flags:        ", r.valid_flags);
	print_flash_protect_flags("Writable flags:     ", r.writable_flags);

	/* Check if we got all the flags we asked for */
	if ((r.flags & p.mask) != (p.flags & p.mask)) {
		fprintf(stderr, "Unable to set requested flags "
			"(wanted mask 0x%08x flags 0x%08x)\n",
			p.mask, p.flags);
		if (p.mask & ~r.writable_flags)
			fprintf(stderr, "Which is expected, because writable "
				"mask is 0x%08x.\n", r.writable_flags);

		return -1;
	}

	return 0;
}


int cmd_serial_test(int argc, char *argv[])
{
	const char *c = "COM2 sample serial output from host!\r\n";

	printf("Writing sample serial output to COM2\n");

	while (*c) {
		/* Wait for space in transmit FIFO */
		while (!(inb(0x2fd) & 0x20))
			;

		/* Put the next character */
		outb(*c++, 0x2f8);
	}

	printf("done.\n");
	return 0;
}


int read_mapped_temperature(int id)
{
	int rv;

	if (!read_mapped_mem8(EC_MEMMAP_THERMAL_VERSION)) {
		/*
		 *  The temp_sensor_init() is not called, which implies no
		 * temp sensor is defined.
		 */
		rv = EC_TEMP_SENSOR_NOT_PRESENT;
	} else if (id < EC_TEMP_SENSOR_ENTRIES)
		rv = read_mapped_mem8(EC_MEMMAP_TEMP_SENSOR + id);
	else if (read_mapped_mem8(EC_MEMMAP_THERMAL_VERSION) >= 2)
		rv = read_mapped_mem8(EC_MEMMAP_TEMP_SENSOR_B +
				      id - EC_TEMP_SENSOR_ENTRIES);
	else {
		/* Sensor in second bank, but second bank isn't supported */
		rv = EC_TEMP_SENSOR_NOT_PRESENT;
	}
	return rv;
}


int cmd_temperature(int argc, char *argv[])
{
	int rv;
	int id;
	char *e;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <sensorid> | all\n", argv[0]);
		return -1;
	}

	if (strcmp(argv[1], "all") == 0) {
		for (id = 0;
		     id < EC_TEMP_SENSOR_ENTRIES + EC_TEMP_SENSOR_B_ENTRIES;
		     id++) {
			rv = read_mapped_temperature(id);
			switch (rv) {
			case EC_TEMP_SENSOR_NOT_PRESENT:
				break;
			case EC_TEMP_SENSOR_ERROR:
				fprintf(stderr, "Sensor %d error\n", id);
				break;
			case EC_TEMP_SENSOR_NOT_POWERED:
				fprintf(stderr, "Sensor %d disabled\n", id);
				break;
			case EC_TEMP_SENSOR_NOT_CALIBRATED:
				fprintf(stderr, "Sensor %d not calibrated\n",
					id);
				break;
			default:
				printf("%d: %d\n", id,
				       rv + EC_TEMP_SENSOR_OFFSET);
			}
		}
		return 0;
	}

	id = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad sensor ID.\n");
		return -1;
	}

	if (id < 0 ||
	    id >= EC_TEMP_SENSOR_ENTRIES + EC_TEMP_SENSOR_B_ENTRIES) {
		printf("Sensor ID invalid.\n");
		return -1;
	}

	printf("Reading temperature...");
	rv = read_mapped_temperature(id);

	switch (rv) {
	case EC_TEMP_SENSOR_NOT_PRESENT:
		printf("Sensor not present\n");
		return -1;
	case EC_TEMP_SENSOR_ERROR:
		printf("Error\n");
		return -1;
	case EC_TEMP_SENSOR_NOT_POWERED:
		printf("Sensor disabled/unpowered\n");
		return -1;
	case EC_TEMP_SENSOR_NOT_CALIBRATED:
		fprintf(stderr, "Sensor not calibrated\n");
		return -1;
	default:
		printf("%d\n", rv + EC_TEMP_SENSOR_OFFSET);
		return 0;
	}
}


int cmd_temp_sensor_info(int argc, char *argv[])
{
	struct ec_params_temp_sensor_get_info p;
	struct ec_response_temp_sensor_get_info r;
	int rv;
	char *e;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <sensorid> | all\n", argv[0]);
		return -1;
	}

	if (strcmp(argv[1], "all") == 0) {
		for (p.id = 0;
		     p.id < EC_TEMP_SENSOR_ENTRIES + EC_TEMP_SENSOR_B_ENTRIES;
		     p.id++) {
			rv = ec_command(EC_CMD_TEMP_SENSOR_GET_INFO, 0,
					&p, sizeof(p), &r, sizeof(r));
			if (rv < 0)
				continue;
			printf("%d: %d %s\n", p.id, r.sensor_type,
			       r.sensor_name);
		}
		return 0;
	}

	p.id = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad sensor ID.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_TEMP_SENSOR_GET_INFO, 0,
			&p, sizeof(p), &r, sizeof(r));
	if (rv < 0)
		return rv;

	printf("Sensor name: %s\n", r.sensor_name);
	printf("Sensor type: %d\n", r.sensor_type);

	return 0;
}


int cmd_thermal_get_threshold_v0(int argc, char *argv[])
{
	struct ec_params_thermal_get_threshold p;
	struct ec_response_thermal_get_threshold r;
	char *e;
	int rv;

	if (argc != 3) {
		fprintf(stderr,
			"Usage: %s <sensortypeid> <thresholdid>\n", argv[0]);
		return -1;
	}

	p.sensor_type = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad sensor type ID.\n");
		return -1;
	}

	p.threshold_id = strtol(argv[2], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad threshold ID.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_THERMAL_GET_THRESHOLD, 0,
			&p, sizeof(p), &r, sizeof(r));
	if (rv < 0)
		return rv;

	if (r.value < 0)
		return -1;

	printf("Threshold %d for sensor type %d is %d K.\n",
			p.threshold_id, p.sensor_type, r.value);

	return 0;
}


int cmd_thermal_set_threshold_v0(int argc, char *argv[])
{
	struct ec_params_thermal_set_threshold p;
	char *e;
	int rv;

	if (argc != 4) {
		fprintf(stderr,
			"Usage: %s <sensortypeid> <thresholdid> <value>\n",
			argv[0]);
		return -1;
	}

	p.sensor_type = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad sensor type ID.\n");
		return -1;
	}

	p.threshold_id = strtol(argv[2], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad threshold ID.\n");
		return -1;
	}

	p.value = strtol(argv[3], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad threshold value.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_THERMAL_SET_THRESHOLD, 0,
			&p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("Threshold %d for sensor type %d set to %d.\n",
			p.threshold_id, p.sensor_type, p.value);

	return 0;
}


int cmd_thermal_get_threshold_v1(int argc, char *argv[])
{
	struct ec_params_thermal_get_threshold_v1 p;
	struct ec_thermal_config r;
	struct ec_params_temp_sensor_get_info pi;
	struct ec_response_temp_sensor_get_info ri;
	int rv;
	int i;

	printf("sensor  warn  high  halt   fan_off fan_max   name\n");
	for (i = 0; i < 99; i++) {	/* number of sensors is unknown */

		/* ask for one */
		p.sensor_num = i;
		rv = ec_command(EC_CMD_THERMAL_GET_THRESHOLD, 1,
				&p, sizeof(p), &r, sizeof(r));
		if (rv <= 0)		/* stop on first failure */
			break;

		/* ask for its name, too */
		pi.id = i;
		rv = ec_command(EC_CMD_TEMP_SENSOR_GET_INFO, 0,
				&pi, sizeof(pi), &ri, sizeof(ri));

		/* print what we know */
		printf(" %2d      %3d   %3d    %3d    %3d     %3d     %s\n",
		       i,
		       r.temp_host[EC_TEMP_THRESH_WARN],
		       r.temp_host[EC_TEMP_THRESH_HIGH],
		       r.temp_host[EC_TEMP_THRESH_HALT],
		       r.temp_fan_off, r.temp_fan_max,
		       rv > 0 ? ri.sensor_name : "?");
	}
	if (i)
		printf("(all temps in degrees Kelvin)\n");

	return 0;
}

int cmd_thermal_set_threshold_v1(int argc, char *argv[])
{
	struct ec_params_thermal_get_threshold_v1 p;
	struct ec_thermal_config r;
	struct ec_params_thermal_set_threshold_v1 s;
	int i, n, val, rv;
	char *e;

	if (argc < 3 || argc > 7) {
		printf("Usage: %s"
		       " sensor warn [high [shutdown [fan_off [fan_max]]]]\n",
		       argv[0]);
		return 1;
	}

	n = strtod(argv[1], &e);
	if (e && *e) {
		printf("arg %d is invalid\n", 1);
		return 1;
	}

	p.sensor_num = n;
	rv = ec_command(EC_CMD_THERMAL_GET_THRESHOLD, 1,
			&p, sizeof(p), &r, sizeof(r));
	if (rv <= 0)
		return rv;

	s.sensor_num = n;
	s.cfg = r;

	for (i = 2; i < argc; i++) {
		val = strtod(argv[i], &e);
		if (e && *e) {
			printf("arg %d is invalid\n", i);
			return 1;
		}

		if (val < 0)
			continue;
		switch (i) {
		case 2:
		case 3:
		case 4:
			s.cfg.temp_host[i-2] = val;
			break;
		case 5:
			s.cfg.temp_fan_off = val;
			break;
		case 6:
			s.cfg.temp_fan_max = val;
			break;
		}
	}

	rv = ec_command(EC_CMD_THERMAL_SET_THRESHOLD, 1,
			&s, sizeof(s), NULL, 0);

	return rv;
}

/**
 * Detect the version of EC_CMD_THERMAL_GET_THRESHOLD that the EC supports.
 *
 * @return The version, or -1 if error.
 */
static int thermal_threshold_version(void)
{
	struct ec_params_thermal_get_threshold v0_p;
	struct ec_response_thermal_get_threshold v0_r;
	struct ec_params_thermal_get_threshold_v1 v1_p;
	struct ec_thermal_config v1_r;
	int rv;

	v1_p.sensor_num = 0;
	rv = ec_command(EC_CMD_THERMAL_GET_THRESHOLD, 1,
			&v1_p, sizeof(v1_p), &v1_r, sizeof(v1_r));

	/*
	 * TODO(crosbug.com/p/23828): Version 1 of the threshold command will
	 * only return EC_RES_SUCCESS or EC_RES_INVALID_PARAM?
	 */
	if (rv > 0)
		return 1;

	v0_p.sensor_type = 0;
	v0_p.threshold_id = 0;
	rv = ec_command(EC_CMD_THERMAL_GET_THRESHOLD, 0,
			&v0_p, sizeof(v0_p), &v0_r, sizeof(v0_r));
	/*
	 * TODO(crosbug.com/p/23828): Version 0 of the threshold command will
	 * only return EC_RES_SUCCESS or EC_RES_ERROR?
	 */
	if (rv > 0)
		return 0;

	/*
	 * Anything else is most likely EC_RES_INVALID_COMMAND, but we don't
	 * care because it's nothing we can use.
	 */
	return -1;
}

int cmd_thermal_get_threshold(int argc, char *argv[])
{
	switch (thermal_threshold_version()) {
	case 0:
		return cmd_thermal_get_threshold_v0(argc, argv);
	case 1:
		return cmd_thermal_get_threshold_v1(argc, argv);
	default:
		printf("I got nuthin.\n");
		return -1;
	}
}

int cmd_thermal_set_threshold(int argc, char *argv[])
{
	switch (thermal_threshold_version()) {
	case 0:
		return cmd_thermal_set_threshold_v0(argc, argv);
	case 1:
		return cmd_thermal_set_threshold_v1(argc, argv);
	default:
		printf("I got nuthin.\n");
		return -1;
	}
}


int cmd_thermal_auto_fan_ctrl(int argc, char *argv[])
{
	int rv;

	rv = ec_command(EC_CMD_THERMAL_AUTO_FAN_CTRL, 0, NULL, 0, NULL, 0);
	if (rv < 0)
		return rv;

	printf("Automatic fan control is now on.\n");
	return 0;
}

static int print_fan(int idx)
{
	int rv = read_mapped_mem16(EC_MEMMAP_FAN + 2 * idx);

	switch (rv) {
	case EC_FAN_SPEED_NOT_PRESENT:
		return -1;
	case EC_FAN_SPEED_STALLED:
		printf("Fan %d stalled!\n", idx);
		break;
	default:
		printf("Fan %d RPM: %d\n", idx, rv);
		break;
	}

	return 0;
}

int cmd_pwm_get_fan_rpm(int argc, char *argv[])
{
	int i;

	if (argc < 2 || !strcasecmp(argv[1], "all")) {
		/* Print all the fan speeds */
		for (i = 0; i < EC_FAN_SPEED_ENTRIES; i++) {
			if (print_fan(i))
				break;  /* Stop at first not-present fan */
		}

	} else {
		char *e;
		int idx;

		idx = strtol(argv[1], &e, 0);
		if ((e && *e) || idx < 0 || idx >= EC_FAN_SPEED_ENTRIES) {
			fprintf(stderr, "Bad index.\n");
			return -1;
		}

		return print_fan(idx);
	}

	return 0;
}


int cmd_pwm_set_fan_rpm(int argc, char *argv[])
{
	struct ec_params_pwm_set_fan_target_rpm p;
	char *e;
	int rv;

	if (argc != 2) {
		fprintf(stderr,
			"Usage: %s <targetrpm>\n", argv[0]);
		return -1;
	}
	p.rpm = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad RPM.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_PWM_SET_FAN_TARGET_RPM, 0,
			&p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("Fan target RPM set.\n");
	return 0;
}


int cmd_pwm_get_keyboard_backlight(int argc, char *argv[])
{
	struct ec_response_pwm_get_keyboard_backlight r;
	int rv;

	rv = ec_command(EC_CMD_PWM_GET_KEYBOARD_BACKLIGHT, 0,
			NULL, 0, &r, sizeof(r));
	if (rv < 0)
		return rv;

	if (r.enabled == 1)
		printf("Current keyboard backlight percent: %d\n", r.percent);
	else
		printf("Keyboard backlight disabled.\n");

	return 0;
}


int cmd_pwm_set_keyboard_backlight(int argc, char *argv[])
{
	struct ec_params_pwm_set_keyboard_backlight p;
	char *e;
	int rv;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <percent>\n", argv[0]);
		return -1;
	}
	p.percent = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad percent.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_PWM_SET_KEYBOARD_BACKLIGHT, 0,
			&p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("Keyboard backlight set.\n");
	return 0;
}

int cmd_fanduty(int argc, char *argv[])
{
	struct ec_params_pwm_set_fan_duty p;
	char *e;
	int rv;

	if (argc != 2) {
		fprintf(stderr,
			"Usage: %s <targetrpm>\n", argv[0]);
		return -1;
	}
	p.percent = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad percent arg.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_PWM_SET_FAN_DUTY, 0, &p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("Fan duty cycle set.\n");
	return 0;
}

#define LBMSG(state) #state
#include "lightbar_msg_list.h"
static const char * const lightbar_cmds[] = {
	LIGHTBAR_MSG_LIST
};
#undef LBMSG

/* This needs to match the values defined in lightbar.h. I'd like to
 * define this in one and only one place, but I can't think of a good way to do
 * that without adding bunch of complexity. This will do for now.
 */
#define LB_SIZES(SUBCMD) { \
		sizeof(((struct ec_params_lightbar *)0)->SUBCMD) \
		+ sizeof(((struct ec_params_lightbar *)0)->cmd), \
		sizeof(((struct ec_response_lightbar *)0)->SUBCMD) }
static const struct {
	uint8_t insize;
	uint8_t outsize;
} lb_command_paramcount[] = {
	LB_SIZES(dump),
	LB_SIZES(off),
	LB_SIZES(on),
	LB_SIZES(init),
	LB_SIZES(set_brightness),
	LB_SIZES(seq),
	LB_SIZES(reg),
	LB_SIZES(set_rgb),
	LB_SIZES(get_seq),
	LB_SIZES(demo),
	LB_SIZES(get_params_v0),
	LB_SIZES(set_params_v0),
	LB_SIZES(version),
	LB_SIZES(get_brightness),
	LB_SIZES(get_rgb),
	LB_SIZES(get_demo),
	LB_SIZES(get_params_v1),
	LB_SIZES(set_params_v1),
};
#undef LB_SIZES

static int lb_help(const char *cmd)
{
	printf("Usage:\n");
	printf("  %s                       - dump all regs\n", cmd);
	printf("  %s off                   - enter standby\n", cmd);
	printf("  %s on                    - leave standby\n", cmd);
	printf("  %s init                  - load default vals\n", cmd);
	printf("  %s brightness [NUM]      - get/set intensity (0-ff)\n", cmd);
	printf("  %s seq [NUM|SEQUENCE]    - run given pattern"
		 " (no arg for list)\n", cmd);
	printf("  %s CTRL REG VAL          - set LED controller regs\n", cmd);
	printf("  %s LED RED GREEN BLUE    - set color manually"
		 " (LED=4 for all)\n", cmd);
	printf("  %s LED                   - get current LED color\n", cmd);
	printf("  %s demo [0|1]            - turn demo mode on & off\n", cmd);
	printf("  %s params [setfile]      - get params"
	       " (or set from file)\n", cmd);
	return 0;
}

static uint8_t lb_find_msg_by_name(const char *str)
{
	uint8_t i;
	for (i = 0; i < LIGHTBAR_NUM_SEQUENCES; i++)
		if (!strcasecmp(str, lightbar_cmds[i]))
			return i;

	return LIGHTBAR_NUM_SEQUENCES;
}

static int lb_do_cmd(enum lightbar_command cmd,
		     struct ec_params_lightbar *in,
		     struct ec_response_lightbar *out)
{
	int rv;
	in->cmd = cmd;
	rv = ec_command(EC_CMD_LIGHTBAR_CMD, 0,
			in, lb_command_paramcount[cmd].insize,
			out, lb_command_paramcount[cmd].outsize);
	return (rv < 0 ? rv : 0);
}

static int lb_show_msg_names(void)
{
	int i, current_state;
	struct ec_params_lightbar param;
	struct ec_response_lightbar resp;

	i = lb_do_cmd(LIGHTBAR_CMD_GET_SEQ, &param, &resp);
	if (i < 0)
		return i;
	current_state = resp.get_seq.num;

	printf("sequence names:");
	for (i = 0; i < LIGHTBAR_NUM_SEQUENCES; i++)
		printf(" %s", lightbar_cmds[i]);
	printf("\nCurrent = 0x%x %s\n", current_state,
	       lightbar_cmds[current_state]);

	return 0;
}

static int lb_read_params_v0_from_file(const char *filename,
				       struct lightbar_params_v0 *p)
{
	FILE *fp;
	char buf[80];
	int val[4];
	int r = 1;
	int line = 0;
	int want, got;
	int i;

	fp = fopen(filename, "rb");
	if (!fp) {
		fprintf(stderr, "Can't open %s: %s\n",
			filename, strerror(errno));
		return 1;
	}

	/* We must read the correct number of params from each line */
#define READ(N) do {							\
		line++;							\
		want = (N);						\
		got = -1;						\
		if (!fgets(buf, sizeof(buf), fp))			\
			goto done;					\
		got = sscanf(buf, "%i %i %i %i",			\
			     &val[0], &val[1], &val[2], &val[3]);	\
		if (want != got)					\
			goto done;					\
	} while (0)


	/* Do it */
	READ(1); p->google_ramp_up = val[0];
	READ(1); p->google_ramp_down = val[0];
	READ(1); p->s3s0_ramp_up = val[0];
	READ(1); p->s0_tick_delay[0] = val[0];
	READ(1); p->s0_tick_delay[1] = val[0];
	READ(1); p->s0a_tick_delay[0] = val[0];
	READ(1); p->s0a_tick_delay[1] = val[0];
	READ(1); p->s0s3_ramp_down = val[0];
	READ(1); p->s3_sleep_for = val[0];
	READ(1); p->s3_ramp_up = val[0];
	READ(1); p->s3_ramp_down = val[0];
	READ(1); p->new_s0 = val[0];

	READ(2);
	p->osc_min[0] = val[0];
	p->osc_min[1] = val[1];
	READ(2);
	p->osc_max[0] = val[0];
	p->osc_max[1] = val[1];
	READ(2);
	p->w_ofs[0] = val[0];
	p->w_ofs[1] = val[1];

	READ(2);
	p->bright_bl_off_fixed[0] = val[0];
	p->bright_bl_off_fixed[1] = val[1];

	READ(2);
	p->bright_bl_on_min[0] = val[0];
	p->bright_bl_on_min[1] = val[1];

	READ(2);
	p->bright_bl_on_max[0] = val[0];
	p->bright_bl_on_max[1] = val[1];

	READ(3);
	p->battery_threshold[0] = val[0];
	p->battery_threshold[1] = val[1];
	p->battery_threshold[2] = val[2];

	READ(4);
	p->s0_idx[0][0] = val[0];
	p->s0_idx[0][1] = val[1];
	p->s0_idx[0][2] = val[2];
	p->s0_idx[0][3] = val[3];

	READ(4);
	p->s0_idx[1][0] = val[0];
	p->s0_idx[1][1] = val[1];
	p->s0_idx[1][2] = val[2];
	p->s0_idx[1][3] = val[3];

	READ(4);
	p->s3_idx[0][0] = val[0];
	p->s3_idx[0][1] = val[1];
	p->s3_idx[0][2] = val[2];
	p->s3_idx[0][3] = val[3];

	READ(4);
	p->s3_idx[1][0] = val[0];
	p->s3_idx[1][1] = val[1];
	p->s3_idx[1][2] = val[2];
	p->s3_idx[1][3] = val[3];

	for (i = 0; i < ARRAY_SIZE(p->color); i++) {
		READ(3);
		p->color[i].r = val[0];
		p->color[i].g = val[1];
		p->color[i].b = val[2];
	}

#undef READ

	/* Yay */
	r = 0;
done:
	if (r)
		fprintf(stderr, "problem with line %d: wanted %d, got %d\n",
			line, want, got);
	fclose(fp);
	return r;
}

static void lb_show_params_v0(const struct lightbar_params_v0 *p)
{
	int i;

	printf("%d\t\t# .google_ramp_up\n", p->google_ramp_up);
	printf("%d\t\t# .google_ramp_down\n", p->google_ramp_down);
	printf("%d\t\t# .s3s0_ramp_up\n", p->s3s0_ramp_up);
	printf("%d\t\t# .s0_tick_delay (battery)\n", p->s0_tick_delay[0]);
	printf("%d\t\t# .s0_tick_delay (AC)\n", p->s0_tick_delay[1]);
	printf("%d\t\t# .s0a_tick_delay (battery)\n", p->s0a_tick_delay[0]);
	printf("%d\t\t# .s0a_tick_delay (AC)\n", p->s0a_tick_delay[1]);
	printf("%d\t\t# .s0s3_ramp_down\n", p->s0s3_ramp_down);
	printf("%d\t# .s3_sleep_for\n", p->s3_sleep_for);
	printf("%d\t\t# .s3_ramp_up\n", p->s3_ramp_up);
	printf("%d\t\t# .s3_ramp_down\n", p->s3_ramp_down);
	printf("%d\t\t# .new_s0\n", p->new_s0);
	printf("0x%02x 0x%02x\t# .osc_min (battery, AC)\n",
	       p->osc_min[0], p->osc_min[1]);
	printf("0x%02x 0x%02x\t# .osc_max (battery, AC)\n",
	       p->osc_max[0], p->osc_max[1]);
	printf("%d %d\t\t# .w_ofs (battery, AC)\n",
	       p->w_ofs[0], p->w_ofs[1]);
	printf("0x%02x 0x%02x\t# .bright_bl_off_fixed (battery, AC)\n",
	       p->bright_bl_off_fixed[0], p->bright_bl_off_fixed[1]);
	printf("0x%02x 0x%02x\t# .bright_bl_on_min (battery, AC)\n",
	       p->bright_bl_on_min[0], p->bright_bl_on_min[1]);
	printf("0x%02x 0x%02x\t# .bright_bl_on_max (battery, AC)\n",
	       p->bright_bl_on_max[0], p->bright_bl_on_max[1]);
	printf("%d %d %d\t\t# .battery_threshold\n",
	       p->battery_threshold[0],
	       p->battery_threshold[1],
	       p->battery_threshold[2]);
	printf("%d %d %d %d\t\t# .s0_idx[] (battery)\n",
	       p->s0_idx[0][0], p->s0_idx[0][1],
	       p->s0_idx[0][2], p->s0_idx[0][3]);
	printf("%d %d %d %d\t\t# .s0_idx[] (AC)\n",
	       p->s0_idx[1][0], p->s0_idx[1][1],
	       p->s0_idx[1][2], p->s0_idx[1][3]);
	printf("%d %d %d %d\t# .s3_idx[] (battery)\n",
	       p->s3_idx[0][0], p->s3_idx[0][1],
	       p->s3_idx[0][2], p->s3_idx[0][3]);
	printf("%d %d %d %d\t# .s3_idx[] (AC)\n",
	       p->s3_idx[1][0], p->s3_idx[1][1],
	       p->s3_idx[1][2], p->s3_idx[1][3]);
	for (i = 0; i < ARRAY_SIZE(p->color); i++)
		printf("0x%02x 0x%02x 0x%02x\t# color[%d]\n",
		       p->color[i].r,
		       p->color[i].g,
		       p->color[i].b, i);
}

static int lb_read_params_v1_from_file(const char *filename,
				       struct lightbar_params_v1 *p)
{
	FILE *fp;
	char buf[80];
	int val[4];
	int r = 1;
	int line = 0;
	int want, got;
	int i;

	fp = fopen(filename, "rb");
	if (!fp) {
		fprintf(stderr, "Can't open %s: %s\n",
			filename, strerror(errno));
		return 1;
	}

	/* We must read the correct number of params from each line */
#define READ(N) do {							\
		line++;							\
		want = (N);						\
		got = -1;						\
		if (!fgets(buf, sizeof(buf), fp))			\
			goto done;					\
		got = sscanf(buf, "%i %i %i %i",			\
			     &val[0], &val[1], &val[2], &val[3]);	\
		if (want != got)					\
			goto done;					\
	} while (0)


	/* Do it */
	READ(1); p->google_ramp_up = val[0];
	READ(1); p->google_ramp_down = val[0];
	READ(1); p->s3s0_ramp_up = val[0];
	READ(1); p->s0_tick_delay[0] = val[0];
	READ(1); p->s0_tick_delay[1] = val[0];
	READ(1); p->s0a_tick_delay[0] = val[0];
	READ(1); p->s0a_tick_delay[1] = val[0];
	READ(1); p->s0s3_ramp_down = val[0];
	READ(1); p->s3_sleep_for = val[0];
	READ(1); p->s3_ramp_up = val[0];
	READ(1); p->s3_ramp_down = val[0];
	READ(1); p->tap_tick_delay = val[0];
	READ(1); p->tap_display_time = val[0];

	READ(1); p->tap_pct_red = val[0];
	READ(1); p->tap_pct_green = val[0];
	READ(1); p->tap_seg_min_on = val[0];
	READ(1); p->tap_seg_max_on = val[0];
	READ(1); p->tap_seg_osc = val[0];
	READ(3);
	p->tap_idx[0] = val[0];
	p->tap_idx[1] = val[1];
	p->tap_idx[2] = val[2];

	READ(2);
	p->osc_min[0] = val[0];
	p->osc_min[1] = val[1];
	READ(2);
	p->osc_max[0] = val[0];
	p->osc_max[1] = val[1];
	READ(2);
	p->w_ofs[0] = val[0];
	p->w_ofs[1] = val[1];

	READ(2);
	p->bright_bl_off_fixed[0] = val[0];
	p->bright_bl_off_fixed[1] = val[1];

	READ(2);
	p->bright_bl_on_min[0] = val[0];
	p->bright_bl_on_min[1] = val[1];

	READ(2);
	p->bright_bl_on_max[0] = val[0];
	p->bright_bl_on_max[1] = val[1];

	READ(3);
	p->battery_threshold[0] = val[0];
	p->battery_threshold[1] = val[1];
	p->battery_threshold[2] = val[2];

	READ(4);
	p->s0_idx[0][0] = val[0];
	p->s0_idx[0][1] = val[1];
	p->s0_idx[0][2] = val[2];
	p->s0_idx[0][3] = val[3];

	READ(4);
	p->s0_idx[1][0] = val[0];
	p->s0_idx[1][1] = val[1];
	p->s0_idx[1][2] = val[2];
	p->s0_idx[1][3] = val[3];

	READ(4);
	p->s3_idx[0][0] = val[0];
	p->s3_idx[0][1] = val[1];
	p->s3_idx[0][2] = val[2];
	p->s3_idx[0][3] = val[3];

	READ(4);
	p->s3_idx[1][0] = val[0];
	p->s3_idx[1][1] = val[1];
	p->s3_idx[1][2] = val[2];
	p->s3_idx[1][3] = val[3];

	for (i = 0; i < ARRAY_SIZE(p->color); i++) {
		READ(3);
		p->color[i].r = val[0];
		p->color[i].g = val[1];
		p->color[i].b = val[2];
	}

#undef READ

	/* Yay */
	r = 0;
done:
	if (r)
		fprintf(stderr, "problem with line %d: wanted %d, got %d\n",
			line, want, got);
	fclose(fp);
	return r;
}

static void lb_show_params_v1(const struct lightbar_params_v1 *p)
{
	int i;

	printf("%d\t\t# .google_ramp_up\n", p->google_ramp_up);
	printf("%d\t\t# .google_ramp_down\n", p->google_ramp_down);
	printf("%d\t\t# .s3s0_ramp_up\n", p->s3s0_ramp_up);
	printf("%d\t\t# .s0_tick_delay (battery)\n", p->s0_tick_delay[0]);
	printf("%d\t\t# .s0_tick_delay (AC)\n", p->s0_tick_delay[1]);
	printf("%d\t\t# .s0a_tick_delay (battery)\n", p->s0a_tick_delay[0]);
	printf("%d\t\t# .s0a_tick_delay (AC)\n", p->s0a_tick_delay[1]);
	printf("%d\t\t# .s0s3_ramp_down\n", p->s0s3_ramp_down);
	printf("%d\t\t# .s3_sleep_for\n", p->s3_sleep_for);
	printf("%d\t\t# .s3_ramp_up\n", p->s3_ramp_up);
	printf("%d\t\t# .s3_ramp_down\n", p->s3_ramp_down);
	printf("%d\t\t# .tap_tick_delay\n", p->tap_tick_delay);
	printf("%d\t\t# .tap_display_time\n", p->tap_display_time);
	printf("%d\t\t# .tap_pct_red\n", p->tap_pct_red);
	printf("%d\t\t# .tap_pct_green\n", p->tap_pct_green);
	printf("%d\t\t# .tap_seg_min_on\n", p->tap_seg_min_on);
	printf("%d\t\t# .tap_seg_max_on\n", p->tap_seg_max_on);
	printf("%d\t\t# .tap_seg_osc\n", p->tap_seg_osc);
	printf("%d %d %d\t\t# .tap_idx\n",
	       p->tap_idx[0], p->tap_idx[1], p->tap_idx[2]);
	printf("0x%02x 0x%02x\t# .osc_min (battery, AC)\n",
	       p->osc_min[0], p->osc_min[1]);
	printf("0x%02x 0x%02x\t# .osc_max (battery, AC)\n",
	       p->osc_max[0], p->osc_max[1]);
	printf("%d %d\t\t# .w_ofs (battery, AC)\n",
	       p->w_ofs[0], p->w_ofs[1]);
	printf("0x%02x 0x%02x\t# .bright_bl_off_fixed (battery, AC)\n",
	       p->bright_bl_off_fixed[0], p->bright_bl_off_fixed[1]);
	printf("0x%02x 0x%02x\t# .bright_bl_on_min (battery, AC)\n",
	       p->bright_bl_on_min[0], p->bright_bl_on_min[1]);
	printf("0x%02x 0x%02x\t# .bright_bl_on_max (battery, AC)\n",
	       p->bright_bl_on_max[0], p->bright_bl_on_max[1]);
	printf("%d %d %d\t# .battery_threshold\n",
	       p->battery_threshold[0],
	       p->battery_threshold[1],
	       p->battery_threshold[2]);
	printf("%d %d %d %d\t\t# .s0_idx[] (battery)\n",
	       p->s0_idx[0][0], p->s0_idx[0][1],
	       p->s0_idx[0][2], p->s0_idx[0][3]);
	printf("%d %d %d %d\t\t# .s0_idx[] (AC)\n",
	       p->s0_idx[1][0], p->s0_idx[1][1],
	       p->s0_idx[1][2], p->s0_idx[1][3]);
	printf("%d %d %d %d\t# .s3_idx[] (battery)\n",
	       p->s3_idx[0][0], p->s3_idx[0][1],
	       p->s3_idx[0][2], p->s3_idx[0][3]);
	printf("%d %d %d %d\t# .s3_idx[] (AC)\n",
	       p->s3_idx[1][0], p->s3_idx[1][1],
	       p->s3_idx[1][2], p->s3_idx[1][3]);
	for (i = 0; i < ARRAY_SIZE(p->color); i++)
		printf("0x%02x 0x%02x 0x%02x\t# color[%d]\n",
		       p->color[i].r,
		       p->color[i].g,
		       p->color[i].b, i);
}

static int cmd_lightbar_params_v0(int argc, char **argv)
{
	struct ec_params_lightbar param;
	struct ec_response_lightbar resp;
	int r;

	if (argc > 2) {
		r = lb_read_params_v0_from_file(argv[2],
						&param.set_params_v0);
		if (r)
			return r;
		return lb_do_cmd(LIGHTBAR_CMD_SET_PARAMS_V0,
				 &param, &resp);
	}
	r = lb_do_cmd(LIGHTBAR_CMD_GET_PARAMS_V0, &param, &resp);
	if (!r)
		lb_show_params_v0(&resp.get_params_v0);
	return r;
}

static int cmd_lightbar_params_v1(int argc, char **argv)
{
	struct ec_params_lightbar param;
	struct ec_response_lightbar resp;
	int r;

	if (argc > 2) {
		r = lb_read_params_v1_from_file(argv[2],
						&param.set_params_v1);
		if (r)
			return r;
		return lb_do_cmd(LIGHTBAR_CMD_SET_PARAMS_V1,
				 &param, &resp);
	}
	r = lb_do_cmd(LIGHTBAR_CMD_GET_PARAMS_V1, &param, &resp);
	if (!r)
		lb_show_params_v1(&resp.get_params_v1);
	return r;
}

static int cmd_lightbar(int argc, char **argv)
{
	int i, r;
	struct ec_params_lightbar param;
	struct ec_response_lightbar resp;

	if (1 == argc) {		/* no args = dump 'em all */
		r = lb_do_cmd(LIGHTBAR_CMD_DUMP, &param, &resp);
		if (r)
			return r;
		for (i = 0; i < ARRAY_SIZE(resp.dump.vals); i++) {
			printf(" %02x     %02x     %02x\n",
			       resp.dump.vals[i].reg,
			       resp.dump.vals[i].ic0,
			       resp.dump.vals[i].ic1);
		}
		return 0;
	}

	if (argc == 2 && !strcasecmp(argv[1], "init"))
		return lb_do_cmd(LIGHTBAR_CMD_INIT, &param, &resp);

	if (argc == 2 && !strcasecmp(argv[1], "off"))
		return lb_do_cmd(LIGHTBAR_CMD_OFF, &param, &resp);

	if (argc == 2 && !strcasecmp(argv[1], "on"))
		return lb_do_cmd(LIGHTBAR_CMD_ON, &param, &resp);

	if (!strcasecmp(argv[1], "params0"))
		return cmd_lightbar_params_v0(argc, argv);

	if (!strcasecmp(argv[1], "params1"))
		return cmd_lightbar_params_v1(argc, argv);

	if (!strcasecmp(argv[1], "params")) {
		/* Just try them both */
		fprintf(stderr, "trying params1 ...\n");
		if (0 == cmd_lightbar_params_v1(argc, argv))
			return 0;
		fprintf(stderr, "trying params0 ...\n");
		return cmd_lightbar_params_v0(argc, argv);
	}

	if (!strcasecmp(argv[1], "version")) {
		r = lb_do_cmd(LIGHTBAR_CMD_VERSION, &param, &resp);
		if (!r)
			printf("version %d flags 0x%x\n",
			       resp.version.num, resp.version.flags);
		return r;
	}

	if (argc > 1 && !strcasecmp(argv[1], "brightness")) {
		char *e;
		int rv;
		if (argc > 2) {
			param.set_brightness.num = 0xff &
				strtoul(argv[2], &e, 16);
			return lb_do_cmd(LIGHTBAR_CMD_SET_BRIGHTNESS,
					 &param, &resp);
		}
		rv = lb_do_cmd(LIGHTBAR_CMD_GET_BRIGHTNESS,
			       &param, &resp);
		if (rv)
			return rv;
		printf("%02x\n", resp.get_brightness.num);
		return 0;
	}

	if (argc > 1 && !strcasecmp(argv[1], "demo")) {
		int rv;
		if (argc > 2) {
			if (!strcasecmp(argv[2], "on") || argv[2][0] == '1')
				param.demo.num = 1;
			else if (!strcasecmp(argv[2], "off") ||
				 argv[2][0] == '0')
				param.demo.num = 0;
			else {
				fprintf(stderr, "Invalid arg\n");
				return -1;
			}
			return lb_do_cmd(LIGHTBAR_CMD_DEMO, &param, &resp);
		}

		rv = lb_do_cmd(LIGHTBAR_CMD_GET_DEMO, &param, &resp);
		if (rv)
			return rv;
		printf("%s\n", resp.get_demo.num ? "on" : "off");
		return 0;
	}

	if (argc >= 2 && !strcasecmp(argv[1], "seq")) {
		char *e;
		uint8_t num;
		if (argc == 2)
			return lb_show_msg_names();
		num = 0xff & strtoul(argv[2], &e, 16);
		if (e && *e)
			num = lb_find_msg_by_name(argv[2]);
		if (num >= LIGHTBAR_NUM_SEQUENCES) {
			fprintf(stderr, "Invalid arg\n");
			return -1;
		}
		param.seq.num = num;
		return lb_do_cmd(LIGHTBAR_CMD_SEQ, &param, &resp);
	}

	if (argc == 4) {
		char *e;
		param.reg.ctrl = 0xff & strtoul(argv[1], &e, 16);
		param.reg.reg = 0xff & strtoul(argv[2], &e, 16);
		param.reg.value = 0xff & strtoul(argv[3], &e, 16);
		return lb_do_cmd(LIGHTBAR_CMD_REG, &param, &resp);
	}

	if (argc == 5) {
		char *e;
		param.set_rgb.led = strtoul(argv[1], &e, 16);
		param.set_rgb.red = strtoul(argv[2], &e, 16);
		param.set_rgb.green = strtoul(argv[3], &e, 16);
		param.set_rgb.blue = strtoul(argv[4], &e, 16);
		return lb_do_cmd(LIGHTBAR_CMD_SET_RGB, &param, &resp);
	}

	/* Only thing left is to try to read an LED value */
	if (argc == 2) {
		char *e;
		param.get_rgb.led = strtoul(argv[1], &e, 0);
		if (!(e && *e)) {
			r = lb_do_cmd(LIGHTBAR_CMD_GET_RGB, &param, &resp);
			if (r)
				return r;
			printf("%02x %02x %02x\n",
			       resp.get_rgb.red,
			       resp.get_rgb.green,
			       resp.get_rgb.blue);
			return 0;
		}
	}

	return lb_help(argv[0]);
}

/* Create an array to store sizes of motion sense param and response structs. */
#define MS_SIZES(SUBCMD) { \
		sizeof(((struct ec_params_motion_sense *)0)->SUBCMD) \
		+ sizeof(((struct ec_params_motion_sense *)0)->cmd), \
		sizeof(((struct ec_response_motion_sense *)0)->SUBCMD) }
static const struct {
	uint8_t insize;
	uint8_t outsize;
} ms_command_sizes[] = {
	MS_SIZES(dump),
	MS_SIZES(info),
	MS_SIZES(ec_rate),
	MS_SIZES(sensor_odr),
	MS_SIZES(sensor_range),
	MS_SIZES(kb_wake_angle),
};
BUILD_ASSERT(ARRAY_SIZE(ms_command_sizes) == MOTIONSENSE_NUM_CMDS);
#undef MS_SIZES

static int ms_help(const char *cmd)
{
	printf("Usage:\n");
	printf("  %s                            - dump all motion data\n", cmd);
	printf("  %s active                     - print active flag\n", cmd);
	printf("  %s info NUM                   - print sensor info\n", cmd);
	printf("  %s ec_rate [RATE_MS]          - set/get sample rate\n", cmd);
	printf("  %s odr NUM [ODR [ROUNDUP]]    - set/get sensor ODR\n", cmd);
	printf("  %s range NUM [RANGE [ROUNDUP]]- set/get sensor range\n", cmd);
	printf("  %s kb_wake NUM                - set/get KB wake ang\n", cmd);

	return 0;
}

static int cmd_motionsense(int argc, char **argv)
{
	int i, rv;
	struct ec_params_motion_sense param;
	struct ec_response_motion_sense resp;
	char *e;

	/* No motionsense command has more than 5 args. */
	if (argc > 5)
		return ms_help(argv[0]);

	if (argc == 1) {
		/* No args, dump motion data. */
		param.cmd = MOTIONSENSE_CMD_DUMP;
		rv = ec_command(EC_CMD_MOTION_SENSE_CMD, 0,
				&param, ms_command_sizes[param.cmd].insize,
				&resp, ms_command_sizes[param.cmd].outsize);

		if (rv < 0)
			return rv;

		if (resp.dump.module_flags & MOTIONSENSE_MODULE_FLAG_ACTIVE)
			printf("Motion sensing active\n");
		else
			printf("Motion sensing inactive\n");

		for (i = 0; i < EC_MOTION_SENSOR_COUNT; i++) {
			printf("Sensor %d: ", i);
			if (resp.dump.sensor_flags[i] &
					MOTIONSENSE_SENSOR_FLAG_PRESENT)
				printf("%d\t%d\t%d\n", resp.dump.data[3*i],
							resp.dump.data[3*i+1],
							resp.dump.data[3*i+2]);
			else
				/*
				 * Warning: the following string printed out
				 * is read by an autotest. Do not change string
				 * without consulting autotest for
				 * kernel_CrosECSysfsAccel.
				 */
				printf("None\n");
		}

		return 0;
	}

	if (argc == 2 && !strcasecmp(argv[1], "active")) {
		param.cmd = MOTIONSENSE_CMD_DUMP;
		rv = ec_command(EC_CMD_MOTION_SENSE_CMD, 0,
				&param, ms_command_sizes[param.cmd].insize,
				&resp, ms_command_sizes[param.cmd].outsize);

		/*
		 * Warning: the following strings printed out are read in an
		 * autotest. Do not change string without consulting autotest
		 * for kernel_CrosECSysfsAccel.
		 */
		if (resp.dump.module_flags & MOTIONSENSE_MODULE_FLAG_ACTIVE)
			printf("1\n");
		else
			printf("0\n");

		return 0;
	}

	if (argc == 3 && !strcasecmp(argv[1], "info")) {
		param.cmd = MOTIONSENSE_CMD_INFO;

		param.sensor_odr.sensor_num = strtol(argv[2], &e, 0);
		if (e && *e) {
			fprintf(stderr, "Bad %s arg.\n", argv[1]);
			return -1;
		}

		rv = ec_command(EC_CMD_MOTION_SENSE_CMD, 0,
				&param, ms_command_sizes[param.cmd].insize,
				&resp, ms_command_sizes[param.cmd].outsize);

		if (rv < 0)
			return rv;

		printf("Type:     ");
		switch (resp.info.type) {
		case MOTIONSENSE_TYPE_ACCEL:
			printf("accel\n");
			break;
		case MOTIONSENSE_TYPE_GYRO:
			printf("gyro\n");
			break;
		default:
			printf("unknown\n");
		}

		printf("Location: ");
		switch (resp.info.location) {
		case MOTIONSENSE_LOC_BASE:
			printf("base\n");
			break;
		case MOTIONSENSE_LOC_LID:
			printf("lid\n");
			break;
		default:
			printf("unknown\n");
		}

		printf("Chip:     ");
		switch (resp.info.chip) {
		case MOTIONSENSE_CHIP_KXCJ9:
			printf("kxcj9\n");
			break;
		default:
			printf("unknown\n");
		}

		return 0;
	}

	if (argc < 4 && !strcasecmp(argv[1], "ec_rate")) {
		param.cmd = MOTIONSENSE_CMD_EC_RATE;
		param.ec_rate.data = EC_MOTION_SENSE_NO_VALUE;

		if (argc == 3) {
			param.ec_rate.data = strtol(argv[2], &e, 0);
			if (e && *e) {
				fprintf(stderr, "Bad %s arg.\n", argv[1]);
				return -1;
			}
		}

		rv = ec_command(EC_CMD_MOTION_SENSE_CMD, 0,
				&param, ms_command_sizes[param.cmd].insize,
				&resp, ms_command_sizes[param.cmd].outsize);

		if (rv < 0)
			return rv;

		printf("%d\n", resp.ec_rate.ret);
		return 0;
	}

	if (argc > 2 && !strcasecmp(argv[1], "odr")) {
		param.cmd = MOTIONSENSE_CMD_SENSOR_ODR;
		param.sensor_odr.data = EC_MOTION_SENSE_NO_VALUE;
		param.sensor_odr.roundup = 1;

		param.sensor_odr.sensor_num = strtol(argv[2], &e, 0);
		if (e && *e) {
			fprintf(stderr, "Bad %s arg.\n", argv[1]);
			return -1;
		}

		if (argc >= 4) {
			param.sensor_odr.data = strtol(argv[3], &e, 0);
			if (e && *e) {
				fprintf(stderr, "Bad %s arg.\n", argv[1]);
				return -1;
			}
		}

		if (argc == 5) {
			param.sensor_odr.roundup = strtol(argv[4], &e, 0);
			if (e && *e) {
				fprintf(stderr, "Bad %s arg.\n", argv[1]);
				return -1;
			}
		}

		rv = ec_command(EC_CMD_MOTION_SENSE_CMD, 0,
				&param, ms_command_sizes[param.cmd].insize,
				&resp, ms_command_sizes[param.cmd].outsize);

		if (rv < 0)
			return rv;

		printf("%d\n", resp.sensor_odr.ret);
		return 0;
	}

	if (argc > 2 && !strcasecmp(argv[1], "range")) {
		param.cmd = MOTIONSENSE_CMD_SENSOR_RANGE;
		param.sensor_range.data = EC_MOTION_SENSE_NO_VALUE;
		param.sensor_odr.roundup = 1;

		param.sensor_range.sensor_num = strtol(argv[2], &e, 0);
		if (e && *e) {
			fprintf(stderr, "Bad %s arg.\n", argv[1]);
			return -1;
		}

		if (argc >= 4) {
			param.sensor_range.data = strtol(argv[3], &e, 0);
			if (e && *e) {
				fprintf(stderr, "Bad %s arg.\n", argv[1]);
				return -1;
			}
		}

		if (argc == 5) {
			param.sensor_odr.roundup = strtol(argv[4], &e, 0);
			if (e && *e) {
				fprintf(stderr, "Bad %s arg.\n", argv[1]);
				return -1;
			}
		}

		rv = ec_command(EC_CMD_MOTION_SENSE_CMD, 0,
				&param, ms_command_sizes[param.cmd].insize,
				&resp, ms_command_sizes[param.cmd].outsize);

		if (rv < 0)
			return rv;

		printf("%d\n", resp.sensor_range.ret);
		return 0;
	}

	if (argc < 4 && !strcasecmp(argv[1], "kb_wake")) {
		param.cmd = MOTIONSENSE_CMD_KB_WAKE_ANGLE;
		param.kb_wake_angle.data = EC_MOTION_SENSE_NO_VALUE;

		if (argc == 3) {
			param.kb_wake_angle.data = strtol(argv[2], &e, 0);
			if (e && *e) {
				fprintf(stderr, "Bad %s arg.\n", argv[1]);
				return -1;
			}
		}

		rv = ec_command(EC_CMD_MOTION_SENSE_CMD, 0,
				&param, ms_command_sizes[param.cmd].insize,
				&resp, ms_command_sizes[param.cmd].outsize);

		if (rv < 0)
			return rv;

		printf("%d\n", resp.kb_wake_angle.ret);
		return 0;
	}

	return ms_help(argv[0]);
}

static int find_led_color_by_name(const char *color)
{
	int i;

	for (i = 0; i < EC_LED_COLOR_COUNT; ++i)
		if (!strcasecmp(color, led_color_names[i]))
			return i;

	return -1;
}

static int find_led_id_by_name(const char *led)
{
	int i;

	for (i = 0; i < EC_LED_ID_COUNT; ++i)
		if (!strcasecmp(led, led_names[i]))
			return i;

	return -1;
}

int cmd_led(int argc, char *argv[])
{
	struct ec_params_led_control p;
	struct ec_response_led_control r;
	char *e, *ptr;
	int rv, i, j;

	memset(p.brightness, 0, sizeof(p.brightness));
	p.flags = 0;

	if (argc < 3) {
		fprintf(stderr,
			"Usage: %s <name> <query | auto | "
			"off | <color> | <color>=<value>...>\n", argv[0]);
		return -1;
	}

	p.led_id = find_led_id_by_name(argv[1]);
	if (p.led_id == (uint8_t)-1) {
		fprintf(stderr, "Bad LED name: %s\n", argv[1]);
		fprintf(stderr, "Valid LED names: ");
		for (i = 0; i < EC_LED_ID_COUNT; i++)
			fprintf(stderr, "%s ", led_names[i]);
		fprintf(stderr, "\n");
		return -1;
	}

	if (!strcasecmp(argv[2], "query")) {
		p.flags = EC_LED_FLAGS_QUERY;
		rv = ec_command(EC_CMD_LED_CONTROL, 1, &p, sizeof(p),
				&r, sizeof(r));
		printf("Brightness range for LED %d:\n", p.led_id);
		if (rv < 0) {
			fprintf(stderr, "Error: Unsupported LED.\n");
			return rv;
		}
		for (i = 0; i < EC_LED_COLOR_COUNT; ++i)
			printf("\t%s\t: 0x%x\n",
			       led_color_names[i],
			       r.brightness_range[i]);
		return 0;
	}

	if (!strcasecmp(argv[2], "off")) {
		/* Brightness initialized to 0 for each color. */
	} else if (!strcasecmp(argv[2], "auto")) {
		p.flags = EC_LED_FLAGS_AUTO;
	} else if ((i = find_led_color_by_name(argv[2])) != -1) {
		p.brightness[i] = 0xff;
	} else {
		for (i = 2; i < argc; ++i) {
			ptr = strtok(argv[i], "=");
			j = find_led_color_by_name(ptr);
			if (j == -1) {
				fprintf(stderr, "Bad color name: %s\n", ptr);
				fprintf(stderr, "Valid colors: ");
				for (j = 0; j < EC_LED_COLOR_COUNT; j++)
					fprintf(stderr, "%s ",
						led_color_names[j]);
				fprintf(stderr, "\n");
				return -1;
			}
			ptr = strtok(NULL, "=");
			if (ptr == NULL) {
				fprintf(stderr, "Missing brightness value\n");
				return -1;
			}
			p.brightness[j] = strtol(ptr, &e, 0);
			if (e && *e) {
				fprintf(stderr, "Bad brightness: %s\n", ptr);
				return -1;
			}
		}
	}

	rv = ec_command(EC_CMD_LED_CONTROL, 1, &p, sizeof(p), &r, sizeof(r));
	return (rv < 0 ? rv : 0);
}


int cmd_usb_charge_set_mode(int argc, char *argv[])
{
	struct ec_params_usb_charge_set_mode p;
	char *e;
	int rv;

	if (argc != 3) {
		fprintf(stderr,
			"Usage: %s <port_id> <mode_id>\n", argv[0]);
		return -1;
	}
	p.usb_port_id = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad port ID.\n");
		return -1;
	}
	p.mode = strtol(argv[2], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad mode ID.\n");
		return -1;
	}

	printf("Setting port %d to mode %d...\n", p.usb_port_id, p.mode);

	rv = ec_command(EC_CMD_USB_CHARGE_SET_MODE, 0,
			&p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("USB charging mode set.\n");
	return 0;
}


int cmd_usb_mux(int argc, char *argv[])
{
	struct ec_params_usb_mux p;
	char *e;
	int rv;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <mux>\n", argv[0]);
		return -1;
	}

	p.mux = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad mux value.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_USB_MUX, 0,
			&p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("Set USB mux to 0x%x.\n", p.mux);

	return 0;
}


int cmd_usb_pd(int argc, char *argv[])
{
	const char *role_str[] = {"", "toggle", "toggle-off", "sink", "source"};
	const char *mux_str[] = {"", "none", "usb", "dp", "dock"};
	struct ec_params_usb_pd_control p;
	int rv, i, j;
	int option_ok;
	char *e;

	p.role = USB_PD_CTRL_ROLE_NO_CHANGE;
	p.mux = USB_PD_CTRL_MUX_NO_CHANGE;

	if (argc <= 2) {
		fprintf(stderr, "No option specified.\n");
		return -1;
	}

	p.port = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Invalid param (port)\n");
		return -1;
	}

	for (i = 2; i < argc; ++i) {
		option_ok = 0;
		if (!strcmp(argv[i], "auto")) {
			if (argc != 3) {
				fprintf(stderr, "\"auto\" may not be used "
						"with other options.\n");
				return -1;
			}
			p.role = USB_PD_CTRL_ROLE_TOGGLE_ON;
			p.mux = USB_PD_CTRL_MUX_AUTO;
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(role_str); ++j) {
			if (!strcmp(argv[i], role_str[j])) {
				if (p.role != USB_PD_CTRL_ROLE_NO_CHANGE) {
					fprintf(stderr,
						"Only one role allowed.\n");
					return -1;
				}
				p.role = j;
				option_ok = 1;
				break;
			}
		}
		if (option_ok)
			continue;

		for (j = 0; j < ARRAY_SIZE(mux_str); ++j) {
			if (!strcmp(argv[i], mux_str[j])) {
				if (p.mux != USB_PD_CTRL_MUX_NO_CHANGE) {
					fprintf(stderr,
						"Only one mux type allowed.\n");
					return -1;
				}
				p.mux = j;
				option_ok = 1;
				break;
			}
		}

		if (!option_ok) {
			fprintf(stderr, "Unknown option: %s\n", argv[i]);
			return -1;
		}
	}

	rv = ec_command(EC_CMD_USB_PD_CONTROL, 0, &p, sizeof(p), NULL, 0);
	return (rv < 0 ? rv : 0);
}


int cmd_kbpress(int argc, char *argv[])
{
	struct ec_params_mkbp_simulate_key p;
	char *e;
	int rv;

	if (argc != 4) {
		fprintf(stderr,
			"Usage: %s <row> <col> <0|1>\n", argv[0]);
		return -1;
	}
	p.row = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad row.\n");
		return -1;
	}
	p.col = strtol(argv[2], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad column.\n");
		return -1;
	}
	p.pressed = strtol(argv[3], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad pressed flag.\n");
		return -1;
	}

	printf("%s row %d col %d.\n", p.pressed ? "Pressing" : "Releasing",
				      p.row,
				      p.col);

	rv = ec_command(EC_CMD_MKBP_SIMULATE_KEY, 0,
			&p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;
	printf("Done.\n");
	return 0;
}


static void print_panic_reg(int regnum, const uint32_t *regs, int index)
{
	static const char * const regname[] = {
		"r0 ", "r1 ", "r2 ", "r3 ", "r4 ",
		"r5 ", "r6 ", "r7 ", "r8 ", "r9 ",
		"r10", "r11", "r12", "sp ", "lr ",
		"pc "};

	printf("%s:", regname[regnum]);
	if (regs)
		printf("%08x", regs[index]);
	else
		printf("        ");
	printf((regnum & 3) == 3 ? "\n" : " ");
}


int cmd_panic_info(int argc, char *argv[])
{
	int rv;
	struct panic_data *pdata = (struct panic_data *)ec_inbuf;
	const uint32_t *lregs = pdata->cm.regs;
	const uint32_t *sregs = NULL;
	enum {
		ORIG_UNKNOWN = 0,
		ORIG_PROCESS,
		ORIG_HANDLER
	} origin = ORIG_UNKNOWN;
	int i;
	const char *panic_origins[3] = {"", "PROCESS", "HANDLER"};

	rv = ec_command(EC_CMD_GET_PANIC_INFO, 0, NULL, 0,
			ec_inbuf, ec_max_insize);
	if (rv < 0)
		return rv;

	if (rv == 0) {
		printf("No panic data.\n");
		return 0;
	}

	/*
	 * We only understand panic data with version <= 2. Warn the user
	 * of higher versions.
	 */
	if (pdata->struct_version > 2)
		fprintf(stderr,
			"Unknown panic data version (%d). "
			"Following data may be incorrect!\n",
			pdata->struct_version);

	if (pdata->arch != PANIC_ARCH_CORTEX_M)
		fprintf(stderr, "Unknown architecture (%d). "
			"CPU specific data will be incorrect!\n",
			pdata->arch);

	printf("Saved panic data:%s\n",
	       (pdata->flags & PANIC_DATA_FLAG_OLD_HOSTCMD ? "" : " (NEW)"));

	if (pdata->struct_version == 2)
		origin = ((lregs[11] & 0xf) == 1 || (lregs[11] & 0xf) == 9) ?
			 ORIG_HANDLER : ORIG_PROCESS;

	/*
	 * In pdata struct, 'regs', which is allocated before 'frame', has
	 * one less elements in version 1. Therefore, if the data is from
	 * version 1, shift 'sregs' by one element to align with 'frame' in
	 * version 1.
	 */
	if (pdata->flags & PANIC_DATA_FLAG_FRAME_VALID)
		sregs = pdata->cm.frame - (pdata->struct_version == 1 ? 1 : 0);

	printf("=== %s EXCEPTION: %02x ====== xPSR: %08x ===\n",
	       panic_origins[origin],
	       lregs[1] & 0xff, sregs ? sregs[7] : -1);
	for (i = 0; i < 4; ++i)
		print_panic_reg(i, sregs, i);
	for (i = 4; i < 10; ++i)
		print_panic_reg(i, lregs, i - 1);
	print_panic_reg(10, lregs, 9);
	print_panic_reg(11, lregs, 10);
	print_panic_reg(12, sregs, 4);
	print_panic_reg(13, lregs, origin == ORIG_HANDLER ? 2 : 0);
	print_panic_reg(14, sregs, 5);
	print_panic_reg(15, sregs, 6);

	return 0;
}


int cmd_power_info(int argc, char *argv[])
{
	struct ec_response_power_info r;
	int rv;

	rv = ec_command(EC_CMD_POWER_INFO, 0, NULL, 0, &r, sizeof(r));
	if (rv < 0)
		return rv;

	printf("AC Voltage: %d mV\n", r.voltage_ac);
	printf("System Voltage: %d mV\n", r.voltage_system);
	printf("System Current: %d mA\n", r.current_system);
	printf("System Power: %d mW\n",
			r.voltage_system * r.current_system / 1000);
	printf("USB Device Type: 0x%x\n", r.usb_dev_type);
	printf("USB Current Limit: %d mA\n", r.usb_current_limit);
	return 0;
}


int cmd_pstore_info(int argc, char *argv[])
{
	struct ec_response_pstore_info r;
	int rv;

	rv = ec_command(EC_CMD_PSTORE_INFO, 0, NULL, 0, &r, sizeof(r));
	if (rv < 0)
		return rv;

	printf("PstoreSize %d\nAccessSize %d\n", r.pstore_size, r.access_size);
	return 0;
}


int cmd_pstore_read(int argc, char *argv[])
{
	struct ec_params_pstore_read p;
	uint8_t rdata[EC_PSTORE_SIZE_MAX];
	int offset, size;
	int rv;
	int i;
	char *e;
	char *buf;

	if (argc < 4) {
		fprintf(stderr,
			"Usage: %s <offset> <size> <filename>\n", argv[0]);
		return -1;
	}
	offset = strtol(argv[1], &e, 0);
	if ((e && *e) || offset < 0 || offset > 0x10000) {
		fprintf(stderr, "Bad offset.\n");
		return -1;
	}
	size = strtol(argv[2], &e, 0);
	if ((e && *e) || size <= 0 || size > 0x10000) {
		fprintf(stderr, "Bad size.\n");
		return -1;
	}
	printf("Reading %d bytes at offset %d...\n", size, offset);

	buf = (char *)malloc(size);
	if (!buf) {
		fprintf(stderr, "Unable to allocate buffer.\n");
		return -1;
	}

	/* Read data in chunks */
	for (i = 0; i < size; i += EC_PSTORE_SIZE_MAX) {
		p.offset = offset + i;
		p.size = MIN(size - i, EC_PSTORE_SIZE_MAX);
		rv = ec_command(EC_CMD_PSTORE_READ, 0,
				&p, sizeof(p), rdata, sizeof(rdata));
		if (rv < 0) {
			fprintf(stderr, "Read error at offset %d\n", i);
			free(buf);
			return rv;
		}
		memcpy(buf + i, rdata, p.size);
	}

	rv = write_file(argv[3], buf, size);
	free(buf);
	if (rv)
		return rv;

	printf("done.\n");
	return 0;
}


int cmd_pstore_write(int argc, char *argv[])
{
	struct ec_params_pstore_write p;
	int offset, size;
	int rv;
	int i;
	char *e;
	char *buf;

	if (argc < 3) {
		fprintf(stderr, "Usage: %s <offset> <filename>\n", argv[0]);
		return -1;
	}
	offset = strtol(argv[1], &e, 0);
	if ((e && *e) || offset < 0 || offset > 0x10000) {
		fprintf(stderr, "Bad offset.\n");
		return -1;
	}

	/* Read the input file */
	buf = read_file(argv[2], &size);
	if (!buf)
		return -1;

	printf("Writing to offset %d...\n", offset);

	/* Write data in chunks */
	for (i = 0; i < size; i += EC_PSTORE_SIZE_MAX) {
		p.offset = offset + i;
		p.size = MIN(size - i, EC_PSTORE_SIZE_MAX);
		memcpy(p.data, buf + i, p.size);
		rv = ec_command(EC_CMD_PSTORE_WRITE, 0,
				&p, sizeof(p), NULL, 0);
		if (rv < 0) {
			fprintf(stderr, "Write error at offset %d\n", i);
			free(buf);
			return rv;
		}
	}

	free(buf);
	printf("done.\n");
	return 0;
}


int cmd_host_event_get_raw(int argc, char *argv[])
{
	uint32_t events = read_mapped_mem32(EC_MEMMAP_HOST_EVENTS);

	if (events & EC_HOST_EVENT_MASK(EC_HOST_EVENT_INVALID)) {
		printf("Current host events: invalid\n");
		return -1;
	}

	printf("Current host events: 0x%08x\n", events);
	return 0;
}


int cmd_host_event_get_b(int argc, char *argv[])
{
	struct ec_response_host_event_mask r;
	int rv;

	rv = ec_command(EC_CMD_HOST_EVENT_GET_B, 0,
			NULL, 0, &r, sizeof(r));
	if (rv < 0)
		return rv;
	if (rv < sizeof(r)) {
		fprintf(stderr, "Insufficient data received.\n");
		return -1;
	}

	if (r.mask & EC_HOST_EVENT_MASK(EC_HOST_EVENT_INVALID)) {
		printf("Current host events-B: invalid\n");
		return -1;
	}

	printf("Current host events-B: 0x%08x\n", r.mask);
	return 0;
}


int cmd_host_event_get_smi_mask(int argc, char *argv[])
{
	struct ec_response_host_event_mask r;
	int rv;

	rv = ec_command(EC_CMD_HOST_EVENT_GET_SMI_MASK, 0,
			NULL, 0, &r, sizeof(r));
	if (rv < 0)
		return rv;

	printf("Current host event SMI mask: 0x%08x\n", r.mask);
	return 0;
}


int cmd_host_event_get_sci_mask(int argc, char *argv[])
{
	struct ec_response_host_event_mask r;
	int rv;

	rv = ec_command(EC_CMD_HOST_EVENT_GET_SCI_MASK, 0,
			NULL, 0, &r, sizeof(r));
	if (rv < 0)
		return rv;

	printf("Current host event SCI mask: 0x%08x\n", r.mask);
	return 0;
}


int cmd_host_event_get_wake_mask(int argc, char *argv[])
{
	struct ec_response_host_event_mask r;
	int rv;

	rv = ec_command(EC_CMD_HOST_EVENT_GET_WAKE_MASK, 0,
			NULL, 0, &r, sizeof(r));
	if (rv < 0)
		return rv;

	printf("Current host event wake mask: 0x%08x\n", r.mask);
	return 0;
}


int cmd_host_event_set_smi_mask(int argc, char *argv[])
{
	struct ec_params_host_event_mask p;
	char *e;
	int rv;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <mask>\n", argv[0]);
		return -1;
	}
	p.mask = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad mask.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_HOST_EVENT_SET_SMI_MASK, 0,
			&p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("Mask set.\n");
	return 0;
}


int cmd_host_event_set_sci_mask(int argc, char *argv[])
{
	struct ec_params_host_event_mask p;
	char *e;
	int rv;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <mask>\n", argv[0]);
		return -1;
	}
	p.mask = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad mask.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_HOST_EVENT_SET_SCI_MASK, 0,
			&p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("Mask set.\n");
	return 0;
}


int cmd_host_event_set_wake_mask(int argc, char *argv[])
{
	struct ec_params_host_event_mask p;
	char *e;
	int rv;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <mask>\n", argv[0]);
		return -1;
	}
	p.mask = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad mask.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_HOST_EVENT_SET_WAKE_MASK, 0,
			&p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("Mask set.\n");
	return 0;
}


int cmd_host_event_clear(int argc, char *argv[])
{
	struct ec_params_host_event_mask p;
	char *e;
	int rv;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <mask>\n", argv[0]);
		return -1;
	}
	p.mask = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad mask.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_HOST_EVENT_CLEAR, 0,
			&p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("Host events cleared.\n");
	return 0;
}


int cmd_host_event_clear_b(int argc, char *argv[])
{
	struct ec_params_host_event_mask p;
	char *e;
	int rv;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <mask>\n", argv[0]);
		return -1;
	}
	p.mask = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad mask.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_HOST_EVENT_CLEAR_B, 0,
			&p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("Host events-B cleared.\n");
	return 0;
}


int cmd_switches(int argc, char *argv[])
{
	uint8_t s = read_mapped_mem8(EC_MEMMAP_SWITCHES);
	printf("Current switches:   0x%02x\n", s);
	printf("Lid switch:         %s\n",
	       (s & EC_SWITCH_LID_OPEN ? "OPEN" : "CLOSED"));
	printf("Power button:       %s\n",
	       (s & EC_SWITCH_POWER_BUTTON_PRESSED ? "DOWN" : "UP"));
	printf("Write protect:      %sABLED\n",
	       (s & EC_SWITCH_WRITE_PROTECT_DISABLED ? "DIS" : "EN"));
	printf("Dedicated recovery: %sABLED\n",
	       (s & EC_SWITCH_DEDICATED_RECOVERY ? "EN" : "DIS"));

	return 0;
}


int cmd_wireless(int argc, char *argv[])
{
	char *e;
	int rv;
	int now_flags;

	if (argc < 2) {
		fprintf(stderr,
			"Usage: %s <flags> [<mask> [<susflags> <susmask>]]\n",
			argv[0]);
		fprintf(stderr, "  0x1 = WLAN radio\n"
				"  0x2 = Bluetooth radio\n"
				"  0x4 = WWAN power\n"
				"  0x8 = WLAN power\n");
		return -1;
	}

	now_flags = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad flags.\n");
		return -1;
	}

	if (argc < 3) {
		/* Old-style - current flags only */
		struct ec_params_switch_enable_wireless_v0 p;

		p.enabled = now_flags;
		rv = ec_command(EC_CMD_SWITCH_ENABLE_WIRELESS, 0,
				&p, sizeof(p), NULL, 0);
		if (rv < 0)
			return rv;

		printf("Success.\n");
	} else {
		/* New-style - masks and suspend flags */
		struct ec_params_switch_enable_wireless_v1 p;
		struct ec_response_switch_enable_wireless_v1 r;

		memset(&p, 0, sizeof(p));

		p.now_flags = now_flags;

		p.now_mask = strtol(argv[2], &e, 0);
		if (e && *e) {
			fprintf(stderr, "Bad mask.\n");
			return -1;
		}

		if (argc > 4) {
			p.suspend_flags = strtol(argv[3], &e, 0);
			if (e && *e) {
				fprintf(stderr, "Bad suspend flags.\n");
				return -1;
			}

			p.suspend_mask = strtol(argv[4], &e, 0);
			if (e && *e) {
				fprintf(stderr, "Bad suspend mask.\n");
				return -1;
			}
		}

		rv = ec_command(EC_CMD_SWITCH_ENABLE_WIRELESS,
				EC_VER_SWITCH_ENABLE_WIRELESS,
				&p, sizeof(p), &r, sizeof(r));
		if (rv < 0)
			return rv;

		printf("Now=0x%x, suspend=0x%x\n",
		       r.now_flags, r.suspend_flags);
	}

	return 0;
}


int cmd_i2c_read(int argc, char *argv[])
{
	struct ec_params_i2c_read p;
	struct ec_response_i2c_read r;
	char *e;
	int rv;

	if (argc != 5) {
		fprintf(stderr, "Usage: %s <8 | 16> <port> <addr> <offset>\n",
				argv[0]);
		return -1;
	}

	p.read_size = strtol(argv[1], &e, 0);
	if ((e && *e) || (p.read_size != 8 && p.read_size != 16)) {
		fprintf(stderr, "Bad read size.\n");
		return -1;
	}

	p.port = strtol(argv[2], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad port.\n");
		return -1;
	}

	p.addr = strtol(argv[3], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad address.\n");
		return -1;
	}

	p.offset = strtol(argv[4], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad offset.\n");
		return -1;
	}

	/*
	 * TODO(crosbug.com/p/23570): use I2C_XFER command if supported, then
	 * fall back to I2C_READ.
	 */

	rv = ec_command(EC_CMD_I2C_READ, 0, &p, sizeof(p), &r, sizeof(r));

	if (rv < 0)
		return rv;

	printf("Read from I2C port %d at 0x%x offset 0x%x = 0x%x\n",
	       p.port, p.addr, p.offset, r.data);
	return 0;
}


int cmd_i2c_write(int argc, char *argv[])
{
	struct ec_params_i2c_write p;
	char *e;
	int rv;

	if (argc != 6) {
		fprintf(stderr,
			"Usage: %s <8 | 16> <port> <addr> <offset> <data>\n",
			argv[0]);
		return -1;
	}

	p.write_size = strtol(argv[1], &e, 0);
	if ((e && *e) || (p.write_size != 8 && p.write_size != 16)) {
		fprintf(stderr, "Bad write size.\n");
		return -1;
	}

	p.port = strtol(argv[2], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad port.\n");
		return -1;
	}

	p.addr = strtol(argv[3], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad address.\n");
		return -1;
	}

	p.offset = strtol(argv[4], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad offset.\n");
		return -1;
	}

	p.data = strtol(argv[5], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad data.\n");
		return -1;
	}

	/*
	 * TODO(crosbug.com/p/23570): use I2C_XFER command if supported, then
	 * fall back to I2C_WRITE.
	 */

	rv = ec_command(EC_CMD_I2C_WRITE, 0, &p, sizeof(p), NULL, 0);

	if (rv < 0)
		return rv;

	printf("Wrote 0x%x to I2C port %d at 0x%x offset 0x%x.\n",
	       p.data, p.port, p.addr, p.offset);
	return 0;
}


int cmd_i2c_xfer(int argc, char *argv[])
{
	struct ec_params_i2c_passthru *p =
		(struct ec_params_i2c_passthru *)ec_outbuf;
	struct ec_response_i2c_passthru *r =
		(struct ec_response_i2c_passthru *)ec_inbuf;
	struct ec_params_i2c_passthru_msg *msg = p->msg;
	unsigned int addr;
	uint8_t *pdata;
	char *e;
	int read_len, write_len;
	int size;
	int rv, i;

	if (argc < 4) {
		fprintf(stderr,
			"Usage: %s <port> <slave_addr> <read_count> "
			"[write bytes...]\n", argv[0]);
		return -1;
	}

	p->port = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad port.\n");
		return -1;
	}

	addr = strtol(argv[2], &e, 0) & 0x7f;
	if (e && *e) {
		fprintf(stderr, "Bad slave address.\n");
		return -1;
	}

	read_len = strtol(argv[3], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad read length.\n");
		return -1;
	}

	/* Skip over params to bytes to write */
	argc -= 4;
	argv += 4;
	write_len = argc;
	p->num_msgs = (read_len != 0) + (write_len != 0);

	size = sizeof(*p) + p->num_msgs * sizeof(*msg);
	if (size + write_len > ec_max_outsize) {
		fprintf(stderr, "Params too large for buffer\n");
		return -1;
	}
	if (sizeof(*r) + read_len > ec_max_insize) {
		fprintf(stderr, "Read length too big for buffer\n");
		return -1;
	}

	pdata = (uint8_t *)p + size;
	if (write_len) {
		msg->addr_flags = addr;
		msg->len = write_len;

		for (i = 0; i < write_len; i++) {
			pdata[i] = strtol(argv[i], &e, 0);
			if (e && *e) {
				fprintf(stderr, "Bad write byte %d\n", i);
				return -1;
			}
		}
		msg++;
	}

	if (read_len) {
		msg->addr_flags = addr | EC_I2C_FLAG_READ;
		msg->len = read_len;
	}

	rv = ec_command(EC_CMD_I2C_PASSTHRU, 0, p, size + write_len,
			r, sizeof(*r) + read_len);
	if (rv < 0)
		return rv;

	/* Parse response */
	if (r->i2c_status & (EC_I2C_STATUS_NAK | EC_I2C_STATUS_TIMEOUT)) {
		fprintf(stderr, "Transfer failed with status=0x%x\n",
			r->i2c_status);
		return -1;
	}

	if (rv < sizeof(*r) + read_len) {
		fprintf(stderr, "Truncated read response\n");
		return -1;
	}

	if (read_len) {
		printf("Read bytes:");
		for (i = 0; i < read_len; i++)
			printf(" %#02x", r->data[i]);
		printf("\n");
	} else {
		printf("Write successful.\n");
	}

	return 0;
}

int cmd_lcd_backlight(int argc, char *argv[])
{
	struct ec_params_switch_enable_backlight p;
	char *e;
	int rv;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <0|1>\n", argv[0]);
		return -1;
	}
	p.enabled = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad value.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_SWITCH_ENABLE_BKLIGHT, 0,
			&p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("Success.\n");
	return 0;
}


int cmd_ext_power_current_limit(int argc, char *argv[])
{
	struct ec_params_ext_power_current_limit p;
	int rv;
	char *e;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <max_current_mA>\n", argv[0]);
		return -1;
	}

	p.limit = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad value.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_EXT_POWER_CURRENT_LIMIT, 0, &p, sizeof(p),
			NULL, 0);
	return rv;
}


int cmd_charge_current_limit(int argc, char *argv[])
{
	struct ec_params_current_limit p;
	int rv;
	char *e;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <max_current_mA>\n", argv[0]);
		return -1;
	}

	p.limit = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad value.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_CHARGE_CURRENT_LIMIT, 0, &p, sizeof(p),
			NULL, 0);
	return rv;
}


int cmd_charge_control(int argc, char *argv[])
{
	struct ec_params_charge_control p;
	int rv;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <normal | idle | discharge>\n",
			argv[0]);
		return -1;
	}

	if (!strcasecmp(argv[1], "normal")) {
		p.mode = CHARGE_CONTROL_NORMAL;
	} else if (!strcasecmp(argv[1], "idle")) {
		p.mode = CHARGE_CONTROL_IDLE;
	} else if (!strcasecmp(argv[1], "discharge")) {
		p.mode = CHARGE_CONTROL_DISCHARGE;
	} else {
		fprintf(stderr, "Bad value.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_CHARGE_CONTROL, 1, &p, sizeof(p), NULL, 0);
	if (rv < 0) {
		fprintf(stderr, "Is AC connected?\n");
		return rv;
	}

	switch (p.mode) {
	case CHARGE_CONTROL_NORMAL:
		printf("Charge state machine normal mode.\n");
		break;
	case CHARGE_CONTROL_IDLE:
		printf("Charge state machine force idle.\n");
		break;
	case CHARGE_CONTROL_DISCHARGE:
		printf("Charge state machine force discharge.\n");
		break;
	default:
		break;
	}
	return 0;
}



/* Table of subcommand sizes for EC_CMD_CHARGE_STATE */
#define CB_SIZES(SUBCMD) { \
		sizeof(((struct ec_params_charge_state *)0)->SUBCMD) \
		+ sizeof(((struct ec_params_charge_state *)0)->cmd), \
		sizeof(((struct ec_response_charge_state *)0)->SUBCMD) }
static const struct {
	uint8_t to_ec_size;
	uint8_t from_ec_size;
} cs_paramcount[] = {
	/* Order must match enum charge_state_command */
	CB_SIZES(get_state),
	CB_SIZES(get_param),
	CB_SIZES(set_param),
};
#undef CB_SIZES
BUILD_ASSERT(ARRAY_SIZE(cs_paramcount) == CHARGE_STATE_NUM_CMDS);

static int cs_do_cmd(struct ec_params_charge_state *to_ec,
		     struct ec_response_charge_state *from_ec)
{
	int rv;
	int cmd = to_ec->cmd;

	rv = ec_command(EC_CMD_CHARGE_STATE, 0,
			to_ec, cs_paramcount[cmd].to_ec_size,
			from_ec, cs_paramcount[cmd].from_ec_size);

	return (rv < 0 ? 1 : 0);
}

static const char * const base_params[] = {
	"chg_voltage",
	"chg_current",
	"chg_input_current",
	"chg_status",
	"chg_option",
};
BUILD_ASSERT(ARRAY_SIZE(base_params) == CS_NUM_BASE_PARAMS);

static int cmd_charge_state(int argc, char **argv)
{
	struct ec_params_charge_state param;
	struct ec_response_charge_state resp;
	uint32_t p, v;
	int i, r;
	char *e;

	if (argc > 1 && !strcasecmp(argv[1], "show")) {
		param.cmd = CHARGE_STATE_CMD_GET_STATE;
		r = cs_do_cmd(&param, &resp);
		if (r)
			return r;
		printf("ac = %d\n", resp.get_state.ac);
		printf("chg_voltage = %dmV\n", resp.get_state.chg_voltage);
		printf("chg_current = %dmA\n", resp.get_state.chg_current);
		printf("chg_input_current = %dmA\n",
		       resp.get_state.chg_input_current);
		printf("batt_state_of_charge = %d%%\n",
		       resp.get_state.batt_state_of_charge);
		return 0;
	}

	if (argc > 1 && !strcasecmp(argv[1], "param")) {
		switch (argc) {
		case 3:
			if (!strcasecmp(argv[2], "help"))
				break;
			param.cmd = CHARGE_STATE_CMD_GET_PARAM;
			p = strtoul(argv[2], &e, 0);
			if (e && *e) {
				fprintf(stderr, "Bad param: %s\n", argv[2]);
				return -1;
			}
			param.get_param.param = p;
			r = cs_do_cmd(&param, &resp);
			if (r)
				return r;
			v = resp.get_param.value;
			if (p < CS_NUM_BASE_PARAMS)
				printf("%d (0x%x)       # %s\n", v, v,
				       base_params[p]);
			else
				printf("%d (0x%x)\n", v, v);
			return 0;
		case 4:
			param.cmd = CHARGE_STATE_CMD_SET_PARAM;
			p = strtoul(argv[2], &e, 0);
			if (e && *e) {
				fprintf(stderr, "Bad param: %s\n", argv[2]);
				return -1;
			}
			v = strtoul(argv[3], &e, 0);
			if (e && *e) {
				fprintf(stderr, "Bad value: %s\n", argv[3]);
				return -1;
			}
			param.set_param.param = p;
			param.set_param.value = v;
			return cs_do_cmd(&param, &resp);
		}

		printf("base params:\n");
		for (i = 0; i < CS_NUM_BASE_PARAMS; i++)
			printf("  %d   %s\n", i, base_params[i]);
		printf("custom profile params:\n");
		printf("  0x%x - 0x%x\n", CS_PARAM_CUSTOM_PROFILE_MIN,
		       CS_PARAM_CUSTOM_PROFILE_MAX);

		return 0;
	}

	printf("Usage:\n");
	printf("  %s show                  - show current state\n", argv[0]);
	printf("  %s param NUM [VALUE]     - get/set param NUM\n", argv[0]);
	printf("  %s param help            - show known param NUMs\n", argv[0]);
	return 0;
}

int cmd_gpio_get(int argc, char *argv[])
{
	struct ec_params_gpio_get_v1 p_v1;
	struct ec_response_gpio_get_v1 r_v1;
	int i, rv, subcmd, num_gpios;
	int cmdver = 1;

	if (!ec_cmd_version_supported(EC_CMD_GPIO_GET, cmdver)) {
		struct ec_params_gpio_get p;
		struct ec_response_gpio_get r;

		/* Fall back to version 0 command */
		cmdver = 0;
		if (argc != 2) {
			fprintf(stderr, "Usage: %s <GPIO name>\n", argv[0]);
			return -1;
		}

		if (strlen(argv[1]) + 1 > sizeof(p.name)) {
			fprintf(stderr, "GPIO name too long.\n");
			return -1;
		}
		strcpy(p.name, argv[1]);

		rv = ec_command(EC_CMD_GPIO_GET, cmdver, &p,
				sizeof(p), &r, sizeof(r));
		if (rv < 0)
			return rv;

		printf("GPIO %s = %d\n", p.name, r.val);
		return 0;
	}

	if (argc > 2 || (argc == 2 && !strcmp(argv[1], "help"))) {
		printf("Usage: %s [<subcmd> <GPIO name>]\n", argv[0]);
		printf("'gpioget <GPIO_NAME>' - Get value by name\n");
		printf("'gpioget count' - Get count of GPIOS\n");
		printf("'gpioget all' - Get info for all GPIOs\n");
		return -1;
	}

	/* Keeping it consistent with console command behavior */
	if (argc == 1)
		subcmd = EC_GPIO_GET_INFO;
	else if (!strcmp(argv[1], "count"))
		subcmd = EC_GPIO_GET_COUNT;
	else if (!strcmp(argv[1], "all"))
		subcmd = EC_GPIO_GET_INFO;
	else
		subcmd = EC_GPIO_GET_BY_NAME;

	if (subcmd == EC_GPIO_GET_BY_NAME) {
		p_v1.subcmd = EC_GPIO_GET_BY_NAME;
		if (strlen(argv[1]) + 1 > sizeof(p_v1.get_value_by_name.name)) {
			fprintf(stderr, "GPIO name too long.\n");
			return -1;
		}
		strcpy(p_v1.get_value_by_name.name, argv[1]);

		rv = ec_command(EC_CMD_GPIO_GET, cmdver, &p_v1,
				sizeof(p_v1), &r_v1, sizeof(r_v1));

		if (rv < 0)
			return rv;

		printf("GPIO %s = %d\n", p_v1.get_value_by_name.name,
			r_v1.get_value_by_name.val);
		return 0;
	}

	/* Need GPIO count for EC_GPIO_GET_COUNT or EC_GPIO_GET_INFO */
	p_v1.subcmd = EC_GPIO_GET_COUNT;
	rv = ec_command(EC_CMD_GPIO_GET, cmdver, &p_v1,
			sizeof(p_v1), &r_v1, sizeof(r_v1));
	if (rv < 0)
		return rv;

	if (subcmd == EC_GPIO_GET_COUNT) {
		printf("GPIO COUNT = %d\n", r_v1.get_count.val);
		return 0;
	}

	/* subcmd EC_GPIO_GET_INFO */
	num_gpios = r_v1.get_count.val;
	p_v1.subcmd = EC_GPIO_GET_INFO;

	for (i = 0; i < num_gpios; i++) {
		p_v1.get_info.index = i;

		rv = ec_command(EC_CMD_GPIO_GET, cmdver, &p_v1,
				sizeof(p_v1), &r_v1, sizeof(r_v1));
		if (rv < 0)
			return rv;

		printf("%2d %-32s 0x%04X\n", r_v1.get_info.val,
			r_v1.get_info.name, r_v1.get_info.flags);
	}

	return 0;
}


int cmd_gpio_set(int argc, char *argv[])
{
	struct ec_params_gpio_set p;
	char *e;
	int rv;

	if (argc != 3) {
		fprintf(stderr, "Usage: %s <GPIO name> <0 | 1>\n", argv[0]);
		return -1;
	}

	if (strlen(argv[1]) + 1 > sizeof(p.name)) {
		fprintf(stderr, "GPIO name too long.\n");
		return -1;
	}
	strcpy(p.name, argv[1]);

	p.val = strtol(argv[2], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad value.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_GPIO_SET, 0, &p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("GPIO %s set to %d\n", p.name, p.val);
	return 0;
}


int cmd_battery(int argc, char *argv[])
{
	char batt_text[EC_MEMMAP_TEXT_MAX];
	int rv, val;

	val = read_mapped_mem8(EC_MEMMAP_BATTERY_VERSION);
	if (val < 1) {
		fprintf(stderr, "Battery version %d is not supported\n", val);
		return -1;
	}

	printf("Battery info:\n");

	rv = read_mapped_string(EC_MEMMAP_BATT_MFGR, batt_text,
			sizeof(batt_text));
	if (rv < 0 || !is_string_printable(batt_text))
		goto cmd_error;
	printf("  OEM name:               %s\n", batt_text);

	rv = read_mapped_string(EC_MEMMAP_BATT_MODEL, batt_text,
			sizeof(batt_text));
	if (rv < 0 || !is_string_printable(batt_text))
		goto cmd_error;
	printf("  Model number:           %s\n", batt_text);

	rv = read_mapped_string(EC_MEMMAP_BATT_TYPE, batt_text,
			sizeof(batt_text));
	if (rv < 0 || !is_string_printable(batt_text))
		goto cmd_error;
	printf("  Chemistry   :           %s\n", batt_text);

	rv = read_mapped_string(EC_MEMMAP_BATT_SERIAL, batt_text,
			sizeof(batt_text));
	printf("  Serial number:          %s\n", batt_text);

	val = read_mapped_mem32(EC_MEMMAP_BATT_DCAP);
	if (!is_battery_range(val))
		goto cmd_error;
	printf("  Design capacity:        %u mAh\n", val);

	val = read_mapped_mem32(EC_MEMMAP_BATT_LFCC);
	if (!is_battery_range(val))
		goto cmd_error;
	printf("  Last full charge:       %u mAh\n", val);

	val = read_mapped_mem32(EC_MEMMAP_BATT_DVLT);
	if (!is_battery_range(val))
		goto cmd_error;
	printf("  Design output voltage   %u mV\n", val);

	val = read_mapped_mem32(EC_MEMMAP_BATT_CCNT);
	if (!is_battery_range(val))
		goto cmd_error;
	printf("  Cycle count             %u\n", val);

	val = read_mapped_mem32(EC_MEMMAP_BATT_VOLT);
	if (!is_battery_range(val))
		goto cmd_error;
	printf("  Present voltage         %u mV\n", val);

	val = read_mapped_mem32(EC_MEMMAP_BATT_RATE);
	if (!is_battery_range(val))
		goto cmd_error;
	printf("  Present current         %u mA\n", val);

	val = read_mapped_mem32(EC_MEMMAP_BATT_CAP);
	if (!is_battery_range(val))
		goto cmd_error;
	printf("  Remaining capacity      %u mAh\n", val);

	val = read_mapped_mem8(EC_MEMMAP_BATT_FLAG);
	printf("  Flags                   0x%02x", val);
	if (val & EC_BATT_FLAG_AC_PRESENT)
		printf(" AC_PRESENT");
	if (val & EC_BATT_FLAG_BATT_PRESENT)
		printf(" BATT_PRESENT");
	if (val & EC_BATT_FLAG_DISCHARGING)
		printf(" DISCHARGING");
	if (val & EC_BATT_FLAG_CHARGING)
		printf(" CHARGING");
	if (val & EC_BATT_FLAG_LEVEL_CRITICAL)
		printf(" LEVEL_CRITICAL");
	printf("\n");

	return 0;
cmd_error:
	fprintf(stderr, "Bad battery info value. Check protocol version.\n");
	return -1;
}

int cmd_battery_cut_off(int argc, char *argv[])
{
	struct ec_params_battery_cutoff p;
	int cmd_version;
	int rv;

	memset(&p, 0, sizeof(p));
	if (ec_cmd_version_supported(EC_CMD_BATTERY_CUT_OFF, 1)) {
		cmd_version = 1;
		if (argc > 1) {
			if (!strcasecmp(argv[1], "at-shutdown")) {
				p.flags = EC_BATTERY_CUTOFF_FLAG_AT_SHUTDOWN;
			} else {
				fprintf(stderr, "Bad parameter: %s\n", argv[1]);
				return -1;
			}
		}
	} else {
		/* Fall back to version 0 command */
		cmd_version = 0;
		if (argc > 1) {
			if (!strcasecmp(argv[1], "at-shutdown")) {
				fprintf(stderr, "Explicit 'at-shutdown' ");
				fprintf(stderr, "parameter not supported.\n");
			} else {
				fprintf(stderr, "Bad parameter: %s\n", argv[1]);
			}
			return -1;
		}
	}

	rv = ec_command(EC_CMD_BATTERY_CUT_OFF, cmd_version, &p, sizeof(p),
			NULL, 0);
	rv = (rv < 0 ? rv : 0);

	if (rv < 0) {
		fprintf(stderr, "Failed to cut off battery, rv=%d\n", rv);
		fprintf(stderr, "It is expected if the rv is -%d "
				"(EC_RES_INVALID_COMMAND) if the battery "
				"doesn't support cut-off function.\n",
				EC_RES_INVALID_COMMAND);
	} else {
		printf("\n");
		printf("SUCCESS. The battery has arranged a cut-off.\n");

		if (cmd_version == 1 &&
		    (p.flags & EC_BATTERY_CUTOFF_FLAG_AT_SHUTDOWN))
			printf("The battery will be cut off after shutdown.\n");
		else
			printf("The system should be shutdown immediately.\n");

		printf("\n");
	}
	return rv;
}

int cmd_battery_vendor_param(int argc, char *argv[])
{
	struct ec_params_battery_vendor_param p;
	struct ec_response_battery_vendor_param r;
	char *e;
	int rv;

	if (argc < 3)
		goto cmd_battery_vendor_param_usage;

	if (!strcasecmp(argv[1], "get"))
		p.mode = BATTERY_VENDOR_PARAM_MODE_GET;
	else if (!strcasecmp(argv[1], "set"))
		p.mode = BATTERY_VENDOR_PARAM_MODE_SET;
	else
		goto cmd_battery_vendor_param_usage;

	p.param = strtol(argv[2], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Invalid param.\n");
		goto cmd_battery_vendor_param_usage;
	}

	if (p.mode == BATTERY_VENDOR_PARAM_MODE_SET) {
		if (argc != 4) {
			fprintf(stderr, "Missing value.\n");
			goto cmd_battery_vendor_param_usage;
		}

		p.value = strtol(argv[3], &e, 0);
		if (e && *e) {
			fprintf(stderr, "Invalid value.\n");
			goto cmd_battery_vendor_param_usage;
		}
	}

	rv = ec_command(EC_CMD_BATTERY_VENDOR_PARAM, 0, &p, sizeof(p),
			&r, sizeof(r));

	if (rv < 0)
		return rv;

	printf("0x%08x\n", r.value);

	return 0;

cmd_battery_vendor_param_usage:
	fprintf(stderr,
		"Usage:\t %s get <param>\n"
		"\t %s set <param> <value>\n",
		argv[0], argv[0]);
	return -1;
}

int cmd_board_version(int argc, char *argv[])
{
	struct ec_response_board_version response;
	int rv;

	rv = ec_command(EC_CMD_GET_BOARD_VERSION, 0, NULL, 0, &response,
			sizeof(response));
	if (rv < 0)
		return rv;

	printf("%d\n", response.board_version);
	return rv;
}

int cmd_chipinfo(int argc, char *argv[])
{
	struct ec_response_get_chip_info info;
	int rv;

	printf("Chip info:\n");

	rv = ec_command(EC_CMD_GET_CHIP_INFO, 0, NULL, 0, &info, sizeof(info));
	if (rv < 0)
		return rv;
	printf("  vendor:    %s\n", info.vendor);
	printf("  name:      %s\n", info.name);
	printf("  revision:  %s\n", info.revision);

	return 0;
}

int cmd_proto_info(int argc, char *argv[])
{
	struct ec_response_get_protocol_info info;
	int rv;
	int i;

	printf("Protocol info:\n");

	rv = ec_command(EC_CMD_GET_PROTOCOL_INFO, 0, NULL, 0,
			&info, sizeof(info));
	if (rv < 0) {
		fprintf(stderr, "Protocol info unavailable.  EC probably only "
				"supports protocol version 2.\n");
		return rv;
	}

	printf("  protocol versions:");
	for (i = 0; i < 32; i++) {
		if (info.protocol_versions & (1 << i))
			printf(" %d", i);
	}
	printf("\n");

	printf("  max request:  %4d bytes\n", info.max_request_packet_size);
	printf("  max response: %4d bytes\n", info.max_response_packet_size);
	printf("  flags: 0x%08x\n", info.flags);
	if (info.flags & EC_PROTOCOL_INFO_IN_PROGRESS_SUPPORTED)
		printf("    EC_RES_IN_PROGRESS supported\n");
	return 0;
}

static int ec_hash_help(const char *cmd)
{
	printf("Usage:\n");
	printf("  %s                        - get last hash\n", cmd);
	printf("  %s abort                  - abort hashing\n", cmd);
	printf("  %s start [<offset> <size> [<nonce>]] - start hashing\n", cmd);
	printf("  %s recalc [<offset> <size> [<nonce>]] - sync rehash\n", cmd);
	printf("\n"
	       "If <offset> is RO or RW, offset and size are computed\n"
	       "automatically for the EC-RO or EC-RW firmware image.\n");

	return 0;
}


static int ec_hash_print(const struct ec_response_vboot_hash *r)
{
	int i;

	if (r->status == EC_VBOOT_HASH_STATUS_BUSY) {
		printf("status:  busy\n");
		return 0;
	} else if (r->status == EC_VBOOT_HASH_STATUS_NONE) {
		printf("status:  unavailable\n");
		return 0;
	} else if (r->status != EC_VBOOT_HASH_STATUS_DONE) {
		printf("status:  %d\n", r->status);
		return 0;
	}

	printf("status:  done\n");
	if (r->hash_type == EC_VBOOT_HASH_TYPE_SHA256)
		printf("type:    SHA-256\n");
	else
		printf("type:    %d\n", r->hash_type);

	printf("offset:  0x%08x\n", r->offset);
	printf("size:    0x%08x\n", r->size);

	printf("hash:    ");
	for (i = 0; i < r->digest_size; i++)
		printf("%02x", r->hash_digest[i]);
	printf("\n");
	return 0;
}


int cmd_ec_hash(int argc, char *argv[])
{
	struct ec_params_vboot_hash p;
	struct ec_response_vboot_hash r;
	char *e;
	int rv;

	if (argc < 2) {
		/* Get hash status */
		p.cmd = EC_VBOOT_HASH_GET;
		rv = ec_command(EC_CMD_VBOOT_HASH, 0,
				&p, sizeof(p), &r, sizeof(r));
		if (rv < 0)
			return rv;

		return ec_hash_print(&r);
	}

	if (argc == 2 && !strcasecmp(argv[1], "abort")) {
		/* Abort hash calculation */
		p.cmd = EC_VBOOT_HASH_ABORT;
		rv = ec_command(EC_CMD_VBOOT_HASH, 0,
				&p, sizeof(p), &r, sizeof(r));
		return (rv < 0 ? rv : 0);
	}

	/* The only other commands are start and recalc */
	if (!strcasecmp(argv[1], "start"))
		p.cmd = EC_VBOOT_HASH_START;
	else if (!strcasecmp(argv[1], "recalc"))
		p.cmd = EC_VBOOT_HASH_RECALC;
	else
		return ec_hash_help(argv[0]);

	p.hash_type = EC_VBOOT_HASH_TYPE_SHA256;

	if (argc < 3) {
		fprintf(stderr, "Must specify offset\n");
		return -1;
	}

	if (!strcasecmp(argv[2], "ro")) {
		p.offset = EC_VBOOT_HASH_OFFSET_RO;
		p.size = 0;
		printf("Hashing EC-RO...\n");
	} else if (!strcasecmp(argv[2], "rw")) {
		p.offset = EC_VBOOT_HASH_OFFSET_RW;
		p.size = 0;
		printf("Hashing EC-RW...\n");
	} else if (argc < 4) {
		fprintf(stderr, "Must specify size\n");
		return -1;
	} else {
		p.offset = strtol(argv[2], &e, 0);
		if (e && *e) {
			fprintf(stderr, "Bad offset.\n");
			return -1;
		}
		p.size = strtol(argv[3], &e, 0);
		if (e && *e) {
			fprintf(stderr, "Bad size.\n");
			return -1;
		}
		printf("Hashing %d bytes at offset %d...\n", p.size, p.offset);
	}

	if (argc == 5) {
		/*
		 * Technically nonce can be any binary data up to 64 bytes,
		 * but this command only supports a 32-bit value.
		 */
		uint32_t nonce = strtol(argv[4], &e, 0);
		if (e && *e) {
			fprintf(stderr, "Bad nonce integer.\n");
			return -1;
		}
		memcpy(p.nonce_data, &nonce, sizeof(nonce));
		p.nonce_size = sizeof(nonce);
	} else
		p.nonce_size = 0;

	rv = ec_command(EC_CMD_VBOOT_HASH, 0, &p, sizeof(p), &r, sizeof(r));
	if (rv < 0)
		return rv;

	/* Start command doesn't wait for hashing to finish */
	if (p.cmd == EC_VBOOT_HASH_START)
		return 0;

	/* Recalc command does wait around, so a result is ready now */
	return ec_hash_print(&r);
}


int cmd_rtc_get(int argc, char *argv[])
{
	struct ec_response_rtc r;
	int rv;

	rv = ec_command(EC_CMD_RTC_GET_VALUE, 0, NULL, 0, &r, sizeof(r));
	if (rv < 0)
		return rv;

	printf("Current time: 0x%08x (%d)\n", r.time, r.time);
	return 0;
}


int cmd_rtc_set(int argc, char *argv[])
{
	struct ec_params_rtc p;
	char *e;
	int rv;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <time>\n", argv[0]);
		return -1;
	}
	p.time = strtol(argv[1], &e, 0);
	if (e && *e) {
		fprintf(stderr, "Bad time.\n");
		return -1;
	}

	rv = ec_command(EC_CMD_RTC_SET_VALUE, 0, &p, sizeof(p), NULL, 0);
	if (rv < 0)
		return rv;

	printf("Time set.\n");
	return 0;
}

int cmd_console(int argc, char *argv[])
{
	char *out = (char *)ec_inbuf;
	int rv;

	/* Snapshot the EC console */
	rv = ec_command(EC_CMD_CONSOLE_SNAPSHOT, 0, NULL, 0, NULL, 0);
	if (rv < 0)
		return rv;

	/* Loop and read from the snapshot until it's done */
	while (1) {
		rv = ec_command(EC_CMD_CONSOLE_READ, 0,
				NULL, 0, ec_inbuf, ec_max_insize);
		if (rv < 0)
			return rv;

		/* Empty response means done */
		if (!rv || !*out)
			break;

		/* Make sure output is null-terminated, then dump it */
		out[ec_max_insize - 1] = '\0';
		fputs(out, stdout);
	}
	printf("\n");
	return 0;
}

/* Flood port 80 with byte writes */
int cmd_port_80_flood(int argc, char *argv[])
{
	int i;

	for (i = 0; i < 256; i++)
		outb(i, 0x80);
	return 0;
}

struct param_info {
	const char *name;	/* name of this parameter */
	const char *help;	/* help message */
	int size;		/* size in bytes */
	int offset;		/* offset within structure */
};

#define FIELD(fname, field, help_str) \
	{ \
		.name = fname, \
		.help = help_str, \
		.size = sizeof(((struct ec_mkbp_config *)NULL)->field), \
		.offset = __builtin_offsetof(struct ec_mkbp_config, field), \
	}

static const struct param_info keyconfig_params[] = {
	FIELD("scan_period", scan_period_us, "period between scans"),
	FIELD("poll_timeout", poll_timeout_us,
	      "revert to irq mode after no activity for this long"),
	FIELD("min_post_scan_delay", min_post_scan_delay_us,
	      "minimum post-scan delay before starting a new scan"),
	FIELD("output_settle", output_settle_us,
	      "delay to wait for output to settle"),
	FIELD("debounce_down", debounce_down_us,
	      "time for debounce on key down"),
	FIELD("debounce_up", debounce_up_us, "time for debounce on key up"),
	FIELD("fifo_max_depth", fifo_max_depth,
	      "maximum depth to allow for fifo (0 = disable)"),
	FIELD("flags", flags, "0 to disable scanning, 1 to enable"),
};

static const struct param_info *find_field(const struct param_info *params,
		int count, const char *name, unsigned int *nump)
{
	const struct param_info *param;
	int i;

	for (i = 0, param = params; i < count; i++, param++) {
		if (0 == strcmp(param->name, name)) {
			if (nump)
				*nump = i;
			return param;
		}
	}

	fprintf(stderr, "Unknown parameter '%s'\n", name);
	return NULL;
}

static int get_value(const struct param_info *param, const char *config)
{
	const char *field;

	field = config + param->offset;
	switch (param->size) {
	case 1:
		return *(uint8_t *)field;
	case 2:
		return *(uint16_t *)field;
	case 4:
		return *(uint32_t *)field;
	default:
		fprintf(stderr, "Internal error: unknown size %d\n",
			param->size);
	}

	return -1;
}

static int show_fields(struct ec_mkbp_config *config, int argc, char *argv[])
{
	const struct param_info *param;
	uint32_t mask;
	int i;

	if (!argc) {
		mask = -1U;	/* show all fields */
	} else {
		mask = 0;
		while (argc > 0) {
			unsigned int num;

			param = find_field(keyconfig_params,
					   ARRAY_SIZE(keyconfig_params),
					   argv[0], &num);
			if (!param)
				return -1;
			mask |= 1 << num;
			argc--;
			argv++;
		}
	}

	param = keyconfig_params;
	for (i = 0; i < ARRAY_SIZE(keyconfig_params); i++, param++) {
		if (mask & (1 << i)) {
			fprintf(stderr, "%-12s   %u\n", param->name,
				get_value(param, (char *)config));
		}
	}

	return 0;
}

static int cmd_keyconfig(int argc, char *argv[])
{
	struct ec_params_mkbp_set_config req;
	int cmd;
	int rv;

	if (argc < 2) {
		const struct param_info *param;
		int i;

		fprintf(stderr, "Usage: %s get [<param>] - print params\n"
			"\t%s set [<param>> <value>]\n"
			"   Available params are: (all time values are in us)",
			argv[0], argv[0]);

		param = keyconfig_params;
		for (i = 0; i < ARRAY_SIZE(keyconfig_params); i++, param++) {
			fprintf(stderr, "%-12s   %s\n", param->name,
				param->name);
		}
		return -1;
	}

	/* Get the command */
	if (0 == strcmp(argv[1], "get")) {
		cmd = EC_CMD_MKBP_GET_CONFIG;
	} else if (0 == strcmp(argv[1], "set")) {
		cmd = EC_CMD_MKBP_SET_CONFIG;
	} else {
		fprintf(stderr, "Invalid command '%s\n", argv[1]);
		return -1;
	}

	switch (cmd) {
	case EC_CMD_MKBP_GET_CONFIG:
		/* Read the existing config */
		rv = ec_command(cmd, 0, NULL, 0, &req, sizeof(req));
		if (rv < 0)
			return rv;
		show_fields(&req.config, argc - 2, argv + 2);
		break;
	}

	return 0;
}

int cmd_tmp006cal(int argc, char *argv[])
{
	struct ec_params_tmp006_set_calibration p;
	char *e;
	int idx;
	int rv;

	if (argc < 2) {
		fprintf(stderr, "Must specify tmp006 index.\n");
		return -1;
	}

	idx = strtol(argv[1], &e, 0);
	if ((e && *e) || idx < 0 || idx > 255) {
		fprintf(stderr, "Bad index.\n");
		return -1;
	}

	if (argc == 2) {
		struct ec_params_tmp006_get_calibration pg;
		struct ec_response_tmp006_get_calibration r;

		pg.index = idx;

		rv = ec_command(EC_CMD_TMP006_GET_CALIBRATION, 0,
				&pg, sizeof(pg), &r, sizeof(r));
		if (rv < 0)
			return rv;

		printf("S0: %e\n", r.s0);
		printf("b0: %e\n", r.b0);
		printf("b1: %e\n", r.b1);
		printf("b2: %e\n", r.b2);
		return EC_SUCCESS;
	}

	if (argc != 6) {
		fprintf(stderr, "Must specify all calibration params.\n");
		return -1;
	}

	memset(&p, 0, sizeof(p));
	p.index = idx;

	p.s0 = strtod(argv[2], &e);
	if (e && *e) {
		fprintf(stderr, "Bad S0.\n");
		return -1;
	}

	p.b0 = strtod(argv[3], &e);
	if (e && *e) {
		fprintf(stderr, "Bad b0.\n");
		return -1;
	}

	p.b1 = strtod(argv[4], &e);
	if (e && *e) {
		fprintf(stderr, "Bad b1.\n");
		return -1;
	}

	p.b2 = strtod(argv[5], &e);
	if (e && *e) {
		fprintf(stderr, "Bad b2.\n");
		return -1;
	}

	return ec_command(EC_CMD_TMP006_SET_CALIBRATION, 0,
			  &p, sizeof(p), NULL, 0);
}

int cmd_tmp006raw(int argc, char *argv[])
{
	struct ec_params_tmp006_get_raw p;
	struct ec_response_tmp006_get_raw r;
	char *e;
	int idx;
	int rv;

	if (argc != 2) {
		fprintf(stderr, "Must specify tmp006 index.\n");
		return -1;
	}

	idx = strtol(argv[1], &e, 0);
	if ((e && *e) || idx < 0 || idx > 255) {
		fprintf(stderr, "Bad index.\n");
		return -1;
	}

	p.index = idx;

	rv = ec_command(EC_CMD_TMP006_GET_RAW, 0, &p, sizeof(p), &r, sizeof(r));
	if (rv < 0)
		return rv;

	printf("T: %d.%02d K\n", r.t / 100, r.t % 100);
	printf("V: %d nV\n", r.v);
	return EC_SUCCESS;
}

static int cmd_hang_detect(int argc, char *argv[])
{
	struct ec_params_hang_detect req;
	char *e;

	memset(&req, 0, sizeof(req));

	if (argc == 2 && !strcasecmp(argv[1], "stop")) {
		req.flags = EC_HANG_STOP_NOW;
		return ec_command(EC_CMD_HANG_DETECT, 0, &req, sizeof(req),
				  NULL, 0);
	}

	if (argc == 2 && !strcasecmp(argv[1], "start")) {
		req.flags = EC_HANG_START_NOW;
		return ec_command(EC_CMD_HANG_DETECT, 0, &req, sizeof(req),
				  NULL, 0);
	}

	if (argc == 4) {
		req.flags = strtol(argv[1], &e, 0);
		if (e && *e) {
			fprintf(stderr, "Bad flags.\n");
			return -1;
		}

		req.host_event_timeout_msec = strtol(argv[2], &e, 0);
		if (e && *e) {
			fprintf(stderr, "Bad event timeout.\n");
			return -1;
		}

		req.warm_reboot_timeout_msec = strtol(argv[3], &e, 0);
		if (e && *e) {
			fprintf(stderr, "Bad reboot timeout.\n");
			return -1;
		}

		printf("hang flags=0x%x\n"
		       "event_timeout=%d ms\n"
		       "reboot_timeout=%d ms\n",
		       req.flags, req.host_event_timeout_msec,
		       req.warm_reboot_timeout_msec);

		return ec_command(EC_CMD_HANG_DETECT, 0, &req, sizeof(req),
				  NULL, 0);
	}

	fprintf(stderr,
		"Must specify start/stop or <flags> <event_ms> <reboot_ms>\n");
	return -1;
}

enum port_80_event {
	PORT_80_EVENT_RESUME = 0x1001,  /* S3->S0 transition */
	PORT_80_EVENT_RESET = 0x1002,   /* RESET transition */
};

int cmd_port80_read(int argc, char *argv[])
{
	struct ec_params_port80_read p;
	int cmdver = 1, rv;
	int i, head, tail;
	uint16_t *history;
	uint32_t writes, history_size;
	struct ec_response_port80_read rsp;
	int printed = 0;

	if (!ec_cmd_version_supported(EC_CMD_PORT80_READ, cmdver)) {
		/* fall back to last boot */
		struct ec_response_port80_last_boot r;
		rv = ec_command(EC_CMD_PORT80_LAST_BOOT, 0,
				NULL, 0, &r, sizeof(r));
		fprintf(stderr, "Last boot %2x\n", r.code);
		printf("done.\n");
		return 0;
	}


	/* read writes and history_size */
	p.subcmd = EC_PORT80_GET_INFO;
	rv = ec_command(EC_CMD_PORT80_READ, cmdver,
			&p, sizeof(p), &rsp, sizeof(rsp));
	if (rv < 0) {
		fprintf(stderr, "Read error at writes\n");
		return rv;
	}
	writes = rsp.get_info.writes;
	history_size = rsp.get_info.history_size;

	history = malloc(history_size*sizeof(uint16_t));
	if (!history) {
		fprintf(stderr, "Unable to allocate buffer.\n");
		return -1;
	}
	/* As the history buffer is quite large, we read data in chunks, with
	    size in bytes of EC_PORT80_SIZE_MAX in each chunk.
	    Incrementing offset until all history buffer has been read. To
	    simplify the design, chose HISTORY_LEN is always multiple of
	    EC_PORT80_SIZE_MAX.

	    offset: entry offset from the beginning of history buffer.
	    num_entries: number of entries requested.
	*/
	p.subcmd = EC_PORT80_READ_BUFFER;
	for (i = 0; i < history_size; i += EC_PORT80_SIZE_MAX) {
		p.read_buffer.offset = i;
		p.read_buffer.num_entries = EC_PORT80_SIZE_MAX;
		rv = ec_command(EC_CMD_PORT80_READ, cmdver,
				&p, sizeof(p), &rsp, sizeof(rsp));
		if (rv < 0) {
			fprintf(stderr, "Read error at offset %d\n", i);
			free(history);
			return rv;
		}
		memcpy((void *)(history + i), rsp.data.codes,
			EC_PORT80_SIZE_MAX*sizeof(uint16_t));
	}

	head = writes;
	if (head > history_size)
		tail = head - history_size;
	else
		tail = 0;

	fprintf(stderr, "Port 80 writes");
	for (i = tail; i < head; i++) {
		int e = history[i % history_size];
		switch (e) {
		case PORT_80_EVENT_RESUME:
			fprintf(stderr, "\n(S3->S0)");
			printed = 0;
			break;
		case PORT_80_EVENT_RESET:
			fprintf(stderr, "\n(RESET)");
			printed = 0;
			break;
		default:
			if (!(printed++ % 20))
				fprintf(stderr, "\n ");
			fprintf(stderr, " %02x", e);
		}
	}
	fprintf(stderr, " <--new\n");

	free(history);
	printf("done.\n");
	return 0;
}

struct command {
	const char *name;
	int (*handler)(int argc, char *argv[]);
};

/* NULL-terminated list of commands */
const struct command commands[] = {
	{"extpwrcurrentlimit", cmd_ext_power_current_limit},
	{"autofanctrl", cmd_thermal_auto_fan_ctrl},
	{"backlight", cmd_lcd_backlight},
	{"battery", cmd_battery},
	{"batterycutoff", cmd_battery_cut_off},
	{"batteryparam", cmd_battery_vendor_param},
	{"boardversion", cmd_board_version},
	{"chargecurrentlimit", cmd_charge_current_limit},
	{"chargecontrol", cmd_charge_control},
	{"chargestate", cmd_charge_state},
	{"chipinfo", cmd_chipinfo},
	{"cmdversions", cmd_cmdversions},
	{"console", cmd_console},
	{"echash", cmd_ec_hash},
	{"eventclear", cmd_host_event_clear},
	{"eventclearb", cmd_host_event_clear_b},
	{"eventget", cmd_host_event_get_raw},
	{"eventgetb", cmd_host_event_get_b},
	{"eventgetscimask", cmd_host_event_get_sci_mask},
	{"eventgetsmimask", cmd_host_event_get_smi_mask},
	{"eventgetwakemask", cmd_host_event_get_wake_mask},
	{"eventsetscimask", cmd_host_event_set_sci_mask},
	{"eventsetsmimask", cmd_host_event_set_smi_mask},
	{"eventsetwakemask", cmd_host_event_set_wake_mask},
	{"fanduty", cmd_fanduty},
	{"flasherase", cmd_flash_erase},
	{"flashprotect", cmd_flash_protect},
	{"flashread", cmd_flash_read},
	{"flashwrite", cmd_flash_write},
	{"flashinfo", cmd_flash_info},
	{"gpioget", cmd_gpio_get},
	{"gpioset", cmd_gpio_set},
	{"hangdetect", cmd_hang_detect},
	{"hello", cmd_hello},
	{"kbpress", cmd_kbpress},
	{"i2cread", cmd_i2c_read},
	{"i2cwrite", cmd_i2c_write},
	{"i2cxfer", cmd_i2c_xfer},
	{"led", cmd_led},
	{"lightbar", cmd_lightbar},
	{"keyconfig", cmd_keyconfig},
	{"keyscan", cmd_keyscan},
	{"motionsense", cmd_motionsense},
	{"panicinfo", cmd_panic_info},
	{"pause_in_s5", cmd_s5},
	{"port80read", cmd_port80_read},
	{"powerinfo", cmd_power_info},
	{"protoinfo", cmd_proto_info},
	{"pstoreinfo", cmd_pstore_info},
	{"pstoreread", cmd_pstore_read},
	{"pstorewrite", cmd_pstore_write},
	{"pwmgetfanrpm", cmd_pwm_get_fan_rpm},
	{"pwmgetkblight", cmd_pwm_get_keyboard_backlight},
	{"pwmsetfanrpm", cmd_pwm_set_fan_rpm},
	{"pwmsetkblight", cmd_pwm_set_keyboard_backlight},
	{"readtest", cmd_read_test},
	{"reboot_ec", cmd_reboot_ec},
	{"rtcget", cmd_rtc_get},
	{"rtcset", cmd_rtc_set},
	{"sertest", cmd_serial_test},
	{"port80flood", cmd_port_80_flood},
	{"switches", cmd_switches},
	{"temps", cmd_temperature},
	{"tempsinfo", cmd_temp_sensor_info},
	{"test", cmd_test},
	{"thermalget", cmd_thermal_get_threshold},
	{"thermalset", cmd_thermal_set_threshold},
	{"tmp006cal", cmd_tmp006cal},
	{"tmp006raw", cmd_tmp006raw},
	{"usbchargemode", cmd_usb_charge_set_mode},
	{"usbmux", cmd_usb_mux},
	{"usbpd", cmd_usb_pd},
	{"version", cmd_version},
	{"wireless", cmd_wireless},
	{NULL, NULL}
};

int main(int argc, char *argv[])
{
	const struct command *cmd;
	int dev = 0;
	int interfaces = COMM_ALL;
	int rv = 1;
	int parse_error = 0;
	char *e;
	int i;

	BUILD_ASSERT(ARRAY_SIZE(lb_command_paramcount) == LIGHTBAR_NUM_CMDS);

	while ((i = getopt_long(argc, argv, "?", long_opts, NULL)) != -1) {
		switch (i) {
		case '?':
			/* Unhandled option */
			parse_error = 1;
			break;

		case OPT_DEV:
			dev = strtoul(optarg, &e, 0);
			if (!*optarg || (e && *e)) {
				fprintf(stderr, "Invalid --dev\n");
				parse_error = 1;
			}
			break;

		case OPT_INTERFACE:
			if (!strcasecmp(optarg, "dev")) {
				interfaces = COMM_DEV;
			} else if (!strcasecmp(optarg, "lpc")) {
				interfaces = COMM_LPC;
			} else if (!strcasecmp(optarg, "i2c")) {
				interfaces = COMM_I2C;
			} else {
				fprintf(stderr, "Invalid --interface\n");
				parse_error = 1;
			}
			break;
		}
	}

	/* Must specify a command */
	if (!parse_error && optind == argc)
		parse_error = 1;

	/* 'ectool help' prints help with commands */
	if (!parse_error && !strcasecmp(argv[optind], "help")) {
		print_help(argv[0], 1);
		exit(1);
	}

	/* Handle sub-devices command offset */
	if (dev > 0 && dev < 4) {
		set_command_offset(EC_CMD_PASSTHRU_OFFSET(dev));
	} else if (dev != 0) {
		fprintf(stderr, "Bad device number %d\n", dev);
		parse_error = 1;
	}

	if (parse_error) {
		print_help(argv[0], 0);
		exit(1);
	}

	if (acquire_gec_lock(GEC_LOCK_TIMEOUT_SECS) < 0) {
		fprintf(stderr, "Could not acquire GEC lock.\n");
		exit(1);
	}

	if (comm_init(interfaces)) {
		fprintf(stderr, "Couldn't find EC\n");
		goto out;
	}

	/* Handle commands */
	for (cmd = commands; cmd->name; cmd++) {
		if (!strcasecmp(argv[optind], cmd->name)) {
			rv = cmd->handler(argc - optind, argv + optind);
			goto out;
		}
	}

	/* If we're still here, command was unknown */
	fprintf(stderr, "Unknown command '%s'\n\n", argv[optind]);
	print_help(argv[0], 0);

out:
	release_gec_lock();
	return !!rv;
}
