/*
 * Analog Devices SPI3 controller driver
 *
 * Copyright (c) 2014 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ADSP-BF70x fix by DAB-Embedded, 2014 (www.dab-embedded.com)
 *
 */

#include <common.h>
#include <malloc.h>
#include <spi.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/clock.h>
#include "adi_spi3.h"

#define to_adi_spi_slave(s) container_of(s, struct adi_spi_slave, slave)

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	if (is_gpio_cs(cs))
		return gpio_is_valid(gpio_cs(cs));
	else
		return adi_spi_cs_valid(bus, cs);
}

void spi_cs_activate(struct spi_slave *slave)
{
	struct adi_spi_slave *sdev = to_adi_spi_slave(slave);

	if (is_gpio_cs(slave->cs)) {
		unsigned int cs = gpio_cs(slave->cs);
		gpio_set_value(cs, sdev->cs_pol);
	} else {
		u32 ssel;
		ssel = readl(&sdev->regs->ssel);
		ssel |= 1 << slave->cs;
		if (sdev->cs_pol)
			ssel |= (1 << 8) << slave->cs;
		else
			ssel &= ~((1 << 8) << slave->cs);
		writel(ssel, &sdev->regs->ssel);
	}
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	struct adi_spi_slave *sdev = to_adi_spi_slave(slave);

	if (is_gpio_cs(slave->cs)) {
		unsigned int cs = gpio_cs(slave->cs);
		gpio_set_value(cs, !sdev->cs_pol);
		gpio_set_value(cs, 1);
	} else {
		u32 ssel;
		ssel = readl(&sdev->regs->ssel);
		if (sdev->cs_pol)
			ssel &= ~((1 << 8) << slave->cs);
		else
			ssel |= (1 << 8) << slave->cs;
		/* deassert cs */
		writel(ssel, &sdev->regs->ssel);
		/* disable cs */
		ssel &= ~(1 << slave->cs);
		writel(ssel, &sdev->regs->ssel);
	}
}

void spi_init()
{
}

void spi_set_speed(struct spi_slave *slave, uint hz)
{
	struct adi_spi_slave *sdev = to_adi_spi_slave(slave);
	u32 clock;

	clock = get_spi_clk() / hz;
	if (clock)
		clock--;
	sdev->clock = clock;
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	struct adi_spi_slave *sdev;

	if (!spi_cs_is_valid(bus, cs))
		return NULL;

	if (max_hz > get_spi_clk())
		return NULL;

	sdev = adi_spi_setup(bus, cs);
	if (!sdev)
		return NULL;

	sdev->control = SPI_CTL_EN | SPI_CTL_MSTR;
	if (mode & SPI_CPHA)
		sdev->control |= SPI_CTL_CPHA;
	if (mode & SPI_CPOL)
		sdev->control |= SPI_CTL_CPOL;
	if (mode & SPI_LSB_FIRST)
		sdev->control |= SPI_CTL_LSBF;
	sdev->control &= ~SPI_CTL_ASSEL;
	sdev->cs_pol = mode & SPI_CS_HIGH ? 1 : 0;
	spi_set_speed(&sdev->slave, max_hz);

	return &sdev->slave;
}

void spi_free_slave(struct spi_slave *slave)
{
	struct adi_spi_slave *sdev = to_adi_spi_slave(slave);
	free(sdev);
}

int spi_claim_bus(struct spi_slave *slave)
{
	struct adi_spi_slave *sdev = to_adi_spi_slave(slave);

	debug("%s: bus:%i cs:%i\n", __func__, slave->bus, slave->cs);

	if (is_gpio_cs(slave->cs)) {
		unsigned int cs = gpio_cs(slave->cs);
		gpio_request(cs, "adi-spi3");
		gpio_direction_output(cs, !sdev->cs_pol);
		sdev->pins[0] = P_DONTCARE;
	}
	peripheral_request_list(sdev->pins, "adi-spi3");

	writel(sdev->control, &sdev->regs->control);
	writel(sdev->clock, &sdev->regs->clock);
	writel(0x0, &sdev->regs->delay);
	writel(SPI_RXCTL_REN, &sdev->regs->rx_control);
	writel(SPI_TXCTL_TEN | SPI_TXCTL_TTI, &sdev->regs->tx_control);

	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
	struct adi_spi_slave *sdev = to_adi_spi_slave(slave);

	debug("%s: bus:%i cs:%i\n", __func__, slave->bus, slave->cs);

	peripheral_free_list(sdev->pins);
	if (is_gpio_cs(slave->cs))
		gpio_free(gpio_cs(slave->cs));

	writel(0x0, &sdev->regs->rx_control);
	writel(0x0, &sdev->regs->tx_control);
	writel(0x0, &sdev->regs->control);
}

#ifndef CONFIG_SPI_IDLE_VAL
# define CONFIG_SPI_IDLE_VAL 0xff
#endif

static int spi_pio_xfer(struct adi_spi_slave *sdev, const u8 *tx, u8 *rx,
			uint bytes)
{
	/* discard invalid rx data and empty rfifo */
	while (!(readl(&sdev->regs->status) & SPI_STAT_RFE))
		readl(&sdev->regs->rfifo);

	while (bytes--) {
		u8 value = (tx ? *tx++ : CONFIG_SPI_IDLE_VAL);
		debug("%s: tx:%x ", __func__, value);
		writel(value, &sdev->regs->tfifo);
		while (readl(&sdev->regs->status) & SPI_STAT_RFE)
			if (ctrlc())
				return -1;
		value = readl(&sdev->regs->rfifo);
		if (rx)
			*rx++ = value;
		debug("rx:%x\n", value);
	}

	return 0;
}

int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
		void *din, unsigned long flags)
{
	struct adi_spi_slave *sdev = to_adi_spi_slave(slave);
	const u8 *tx = dout;
	u8 *rx = din;
	uint bytes = bitlen / 8;
	int ret = 0;

	debug("%s: bus:%i cs:%i bitlen:%i bytes:%i flags:%lx\n", __func__,
		slave->bus, slave->cs, bitlen, bytes, flags);

	if (bitlen == 0)
		goto done;

	/* we can only do 8 bit transfers */
	if (bitlen % 8) {
		flags |= SPI_XFER_END;
		goto done;
	}

	if (flags & SPI_XFER_BEGIN)
		spi_cs_activate(slave);

	ret = spi_pio_xfer(sdev, tx, rx, bytes);

 done:
	if (flags & SPI_XFER_END)
		spi_cs_deactivate(slave);

	return ret;
}
