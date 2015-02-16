/*
 * U-boot - main board file
 *
 * Copyright (c) 2008-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 *
 * ADSP-BF70x fix by DAB-Embedded, 2014 (www.dab-embedded.com)
 *
 */

#include <common.h>
#include <asm/blackfin.h>
#include <asm/io.h>
#include <i2c.h>
#include "soft_switch.h"

struct switch_config {
	uchar dir0; /* IODIRA */
	uchar dir1; /* IODIRB */
	uchar value0; /* OLATA */
	uchar value1; /* OLATB */
};

static struct switch_config switch_config_array[NUM_SWITCH] = {
	{
		.dir0 = 0x0, /* all output */
		.dir1 = 0x0, /* all output */
		.value0 = SW0_RF_SPI2_SEL1_nEN | SW0_RF_SPI2_SEL2_nEN,
		.value1 = SW0_CAN0_EN | SW0_CAN1_EN,
	},
	{
		.dir0 = 0x0, /* all output */
		.dir1 = 0x0, /* all output */
		.value0 = 0x0,
		.value1 = 0x0,
	},
};

static int setup_soft_switch(int addr, struct switch_config *config)
{
	int ret = 0;
	
	ret = i2c_write(addr, OLATA, 1, &config->value0, 1);
	if (ret)
		return ret;
	ret = i2c_write(addr, OLATB, 1, &config->value1, 1);
	if (ret)
		return ret;

	ret = i2c_write(addr, IODIRA, 1, &config->dir0, 1);
	if (ret)
		return ret;
	return i2c_write(addr, IODIRB, 1, &config->dir1, 1);
}

int config_switch_bit(int addr, int port, int bit, int dir, uchar value)
{
	int ret, data_reg, dir_reg;
	uchar tmp;

	if (port == IO_PORT_A) {
		data_reg = OLATA;
		dir_reg = IODIRA;
	} else {
		data_reg = OLATB;
		dir_reg = IODIRB;
	}

	if (dir == IO_PORT_INPUT) {
		ret = i2c_read(addr, dir_reg, 1, &tmp, 1);
		if (ret)
			return ret;
		tmp |= bit;
		return i2c_write(addr, dir_reg, 1, &tmp, 1);
	} else {
		ret = i2c_read(addr, data_reg, 1, &tmp, 1);
		if (ret)
			return ret;
		if (value)
			tmp |= bit;
		else
			tmp &= ~bit;
		ret = i2c_write(addr, data_reg, 1, &tmp, 1);
		if (ret)
			return ret;
		ret = i2c_read(addr, dir_reg, 1, &tmp, 1);
		if (ret)
			return ret;
		tmp &= ~bit;
		return i2c_write(addr, dir_reg, 1, &tmp, 1);
	}
}

int setup_board_switches(void)
{
	int ret;
	int i;

	for (i = 0; i < NUM_SWITCH; i++) {
		ret = setup_soft_switch(SW0_I2C_ADDR + i,
				&switch_config_array[i]);
		if (ret)
			return ret;
	}
	return 0;
}
