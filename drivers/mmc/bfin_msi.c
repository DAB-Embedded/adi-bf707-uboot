/*
 * Driver for Blackfin on-chip MSI controller
 *
 * Copyright (c) 2014 DAB-Embedded.
 *
 * Licensed under the GPL-2 or later.
 */

#include <common.h>
#include <malloc.h>
#include <part.h>
#include <mmc.h>

#include <asm/io.h>
#include <asm/errno.h>
#include <asm/byteorder.h>
#include <asm/blackfin.h>
#include <asm/portmux.h>
#include <asm/mach-common/bits/dma.h>

#include <dwmmc.h>
#include <asm-generic/errno.h>
#include <asm/clock.h>

# define MSI_PORTMUX_PINS \
	{ P_MSI_CLK, P_MSI_CMD, P_MSI_DATA0, P_MSI_DATA1, P_MSI_DATA2, P_MSI_DATA3, P_MSI_DATA4, P_MSI_DATA5, P_MSI_DATA6, P_MSI_DATA7, P_MSI_CD, 0 }

#define PAGE_SIZE 4096

#define MSI_DEBUG_REGS 1

#ifdef MSI_DEBUG_REGS
char ISTAT_bit_desc[16][50] = 
{
  "Card Detect",
  "Response Error",
  "Command Done",
  "Data Transfer Over",
  "Transmit FIFO Data Request",
  "Receive FIFO Data Request",
  "Response CRC Error",
  "Data CRC Error",
  "Response Timeout",
  "Data Read Timeout",
  "Host Timeout",
  "FIFO underrun/overrun error",
  "Hardware Locked Write Error",
  "Start Bit Error Busy Complete Interrupt",
  "Auto command done",
  "End-bit Error",
};

static void dwmci_display_dump_regs(void)
{
	unsigned long val32;
	int           i;

	// Show ISTAT description
	val32 = bfin_read32(MSI0_ISTAT);
	
	if (val32) {

		printf("MSI0 ISTAT reports:\n");

		for (i = 0; i < 16; i++)
			if (val32 & (1 << i))
				printf("    ! %s\n", ISTAT_bit_desc[i]);
		printf("\n");
	}

}
#endif

static int dwmci_wait_reset(u32 value)
{
	unsigned long timeout = 1000;
	u32 ctrl;

	bfin_write_MSI0_CTL(value);
	SSYNC();

	while (timeout--) {
		SSYNC();
		ctrl = bfin_read_MSI0_CTL();
		if (!(ctrl & DWMCI_RESET_ALL))
			return 1;
	}
	return 0;
}

static void dwmci_set_idma_desc(struct dwmci_idmac *idmac,
		u32 desc0, u32 desc1, u32 desc2)
{
	struct dwmci_idmac *desc = idmac;

	desc->flags = desc0;
	desc->cnt = desc1;
	desc->addr = desc2;
	desc->next_addr = (unsigned int)desc + sizeof(struct dwmci_idmac);
}

static void dwmci_prepare_data(struct mmc_data *data)
{
	unsigned long ctrl;
	unsigned int i = 0, flags, cnt, blk_cnt;
	ulong data_start, data_end, start_addr;
	ALLOC_CACHE_ALIGN_BUFFER(struct dwmci_idmac, cur_idmac, data->blocks);

	if (data->flags == MMC_DATA_READ)
		invalidate_dcache_range((unsigned long)data->dest,
				(unsigned long)(data->dest + (data->blocks * data->blocksize)));
	else
		invalidate_dcache_range((unsigned long)data->src,
				(unsigned long)(data->src + (data->blocks * data->blocksize)));

	SSYNC();

	blk_cnt = data->blocks;

	dwmci_wait_reset(DWMCI_CTRL_FIFO_RESET);
	SSYNC();

	data_start = (ulong)cur_idmac;
	bfin_write_MSI0_DBADDR((unsigned int)cur_idmac);
	SSYNC();

	if (data->flags == MMC_DATA_READ)
		start_addr = (unsigned int)data->dest;
	else
		start_addr = (unsigned int)data->src;

	do {
		flags = DWMCI_IDMAC_OWN | DWMCI_IDMAC_CH ;
		flags |= (i == 0) ? DWMCI_IDMAC_FS : 0;
		if (blk_cnt <= 8) {
			flags |= DWMCI_IDMAC_LD;
			cnt = data->blocksize * blk_cnt;
		} else
			cnt = data->blocksize * 8;

		dwmci_set_idma_desc(cur_idmac, flags, cnt,
				start_addr + (i * PAGE_SIZE));
		SSYNC();

		if(blk_cnt < 8)
			break;
		blk_cnt -= 8;
		cur_idmac++;
		i++;
	} while(1);

	data_end = (ulong)cur_idmac;
	invalidate_dcache_range((unsigned long)data_start, (unsigned long)(data_end + ARCH_DMA_MINALIGN));
	SSYNC();

	ctrl = bfin_read_MSI0_CTL();
	ctrl |= DWMCI_IDMAC_EN | DWMCI_DMA_EN;
	bfin_write_MSI0_CTL(ctrl);
	SSYNC();

	ctrl = bfin_read_MSI0_BUSMODE();
	ctrl |= DWMCI_BMOD_IDMAC_FB | DWMCI_BMOD_IDMAC_EN;
	bfin_write_MSI0_BUSMODE(ctrl);
	SSYNC();

	bfin_write_MSI0_BLKSIZ(data->blocksize);
	bfin_write_MSI0_BYTCNT(data->blocksize * data->blocks);
	SSYNC();
}

static int dwmci_set_transfer_mode(struct mmc_data *data)
{
	unsigned long mode;

	mode = DWMCI_CMD_DATA_EXP;
	if (data->flags & MMC_DATA_WRITE)
		mode |= DWMCI_CMD_RW;

	return mode;
}

static int dwmci_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
		struct mmc_data *data)
{
	int flags = 0, i;
	unsigned int timeout = 100000;
	u32 retry = 10000;
	u32 mask, ctrl;
	ulong start = get_timer(0);

	while (bfin_read_MSI0_STAT() & DWMCI_BUSY) {
		SSYNC();
		if (get_timer(start) > timeout) {
			printf("Timeout on data busy\n");
			return TIMEOUT;
		}
	}

	bfin_write_MSI0_ISTAT(DWMCI_INTMSK_ALL);
	SSYNC();

	if (data)
		dwmci_prepare_data(data);

	bfin_write_MSI0_CMDARG(cmd->cmdarg);
	SSYNC();

	if (data)
		flags = dwmci_set_transfer_mode(data);

	if ((cmd->resp_type & MMC_RSP_136) && (cmd->resp_type & MMC_RSP_BUSY))
		return -1;

	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
		flags |= DWMCI_CMD_ABORT_STOP;
	else
		flags |= DWMCI_CMD_PRV_DAT_WAIT;

	if (cmd->resp_type & MMC_RSP_PRESENT) {
		flags |= DWMCI_CMD_RESP_EXP;
		if (cmd->resp_type & MMC_RSP_136)
			flags |= DWMCI_CMD_RESP_LENGTH;
	}

	if (cmd->resp_type & MMC_RSP_CRC)
		flags |= DWMCI_CMD_CHECK_CRC;

	flags |= (cmd->cmdidx | DWMCI_CMD_START | DWMCI_CMD_USE_HOLD_REG);

	debug("Sending CMD%d with ARG%x\n",cmd->cmdidx, cmd->cmdarg);

	bfin_write_MSI0_CMD(flags);
	SSYNC();

	for (i = 0; i < retry; i++) {
		mask = bfin_read_MSI0_ISTAT();
		SSYNC();
		if (mask & DWMCI_INTMSK_CDONE) {
			if (!data)
				bfin_write_MSI0_ISTAT(mask);
			break;
		}
	}
	SSYNC();

	if (i == retry)
		return TIMEOUT;

	if (mask & DWMCI_INTMSK_RTO) {
		debug("Response Timeout..\n");
		return TIMEOUT;
	} else if (mask & DWMCI_INTMSK_RE) {
		debug("Response Error..\n");
		return -1;
	}


	if (cmd->resp_type & MMC_RSP_PRESENT) {
		if (cmd->resp_type & MMC_RSP_136) {
			cmd->response[0] = bfin_read_MSI0_RESP3();
			cmd->response[1] = bfin_read_MSI0_RESP2();
			cmd->response[2] = bfin_read_MSI0_RESP1();
			cmd->response[3] = bfin_read_MSI0_RESP0();
		} else {
			cmd->response[0] = bfin_read_MSI0_RESP0();
		}
	}

	if (data) {
		do {
			SSYNC();
			mask = bfin_read_MSI0_ISTAT();
			if (mask & (DWMCI_DATA_ERR | DWMCI_DATA_TOUT)) {
				debug("DATA ERROR, ISTAT=0x%x!\n", mask);
#ifdef MSI_DEBUG_REGS
				dwmci_display_dump_regs();
#endif
				return -1;
			}
			udelay(2);
		} while (!(mask & DWMCI_INTMSK_DTO));

		bfin_write_MSI0_ISTAT(mask);
		SSYNC();

		ctrl = bfin_read_MSI0_CTL();
		ctrl &= ~(DWMCI_DMA_EN);
		bfin_write_MSI0_CTL(ctrl);
		SSYNC();
	}

	udelay(100);

	return 0;
}

static int dwmci_setup_bus(u32 freq)
{
	u32 div, status;
	int timeout = 10000;
	unsigned long sclk;

	if (freq == 0)
		return 0;

	sclk = get_sclk();
	div = DIV_ROUND_UP(sclk, 2 * freq);

	bfin_write_MSI0_CLKEN(0);
	SSYNC();

	bfin_write_MSI0_CLKDIV(div);
	bfin_write_MSI0_CMD(DWMCI_CMD_PRV_DAT_WAIT |
			DWMCI_CMD_UPD_CLK | DWMCI_CMD_START);
	SSYNC();

	do {
		status = bfin_read_MSI0_CMD();
		if (timeout-- < 0) {
			printf("TIMEOUT error!!\n");
			return -ETIMEDOUT;
		}
	} while (status & DWMCI_CMD_START);

	bfin_write_MSI0_CLKEN(DWMCI_CLKEN_ENABLE |
			DWMCI_CLKEN_LOW_PWR);

	bfin_write_MSI0_CMD(DWMCI_CMD_PRV_DAT_WAIT |
			DWMCI_CMD_UPD_CLK | DWMCI_CMD_START);
	SSYNC();

	timeout = 10000;
	do {
		SSYNC();
		status = bfin_read_MSI0_CMD();
		if (timeout-- < 0) {
			printf("TIMEOUT error!!\n");
			return -ETIMEDOUT;
		}
	} while (status & DWMCI_CMD_START);

	return 0;
}

static void dwmci_set_ios(struct mmc *mmc)
{
	u32 ctype;

	debug("Buswidth = %d, clock: %d\n",mmc->bus_width, mmc->clock);

	dwmci_setup_bus(mmc->clock);
	switch (mmc->bus_width) {
	case 8:
		ctype = DWMCI_CTYPE_8BIT;
		break;
	case 4:
		ctype = DWMCI_CTYPE_4BIT;
		break;
	default:
		ctype = DWMCI_CTYPE_1BIT;
		break;
	}

	bfin_write_MSI0_CTYPE(ctype);
	SSYNC();
}

static int dwmci_init(struct mmc *mmc)
{
	u32 fifo_size;
	u32 fifoth_val;

#if ANOMALY_19000012

	bfin_write32(PORTC_FER_CLR,  0x3DD0);
	bfin_write32(PORTC_DIR_SET,  0x3DD0);
	bfin_write32(PORTC_DATA_SET, 0x3DD0);
	bfin_write32(PORTC_INEN_SET, 0x3DD0);

#endif	
	bfin_write_MSI0_PWREN(1);
	SSYNC();

	if (!dwmci_wait_reset(DWMCI_RESET_ALL)) {
		debug("%s[%d] Fail-reset!!\n",__func__,__LINE__);
		return -1;
	}

	/* Enumerate at 400KHz */
	dwmci_setup_bus(get_sclk() >> 9);

	bfin_write_MSI0_ISTAT(0xFFFFFFFF);
	bfin_write_MSI0_IMSK(0);
	SSYNC();

	bfin_write_MSI0_TMOUT(0xFFFFFFFF);

	bfin_write_MSI0_IDINTEN(0);
	bfin_write_MSI0_BUSMODE(1);
	SSYNC();
	
	udelay(2);
	
	bfin_write_MSI0_DEBNCE(0x0000FFFF);
	SSYNC();

	fifo_size = bfin_read_MSI0_FIFOTH();
	fifo_size = ((fifo_size & RX_WMARK_MASK) >> RX_WMARK_SHIFT) + 1;
	fifoth_val = MSIZE(0x2) | RX_WMARK(fifo_size / 2 - 1) |
			TX_WMARK(fifo_size / 2);

	bfin_write_MSI0_FIFOTH(fifoth_val);

	bfin_write_MSI0_CLKEN(0);

	return 0;
}

static int dwmci_getcd(struct mmc *mmc)
{
	return bfin_read_MSI0_CDETECT() ? 0 : 1;
}


static const struct mmc_ops bfin_msi_ops = {
	.send_cmd	= dwmci_send_cmd,
	.set_ios	= dwmci_set_ios,
	.init		= dwmci_init,
	.getcd		= dwmci_getcd,
};

static struct mmc_config bfin_msi_cfg = {
	.name		= "Blackfin MSI",
	.ops		= &bfin_msi_ops,
	.host_caps	= MMC_MODE_4BIT | MMC_MODE_8BIT | MMC_MODE_HS_52MHz | MMC_MODE_HS | MMC_MODE_HC,
	.voltages	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.b_max		= CONFIG_SYS_MMC_MAX_BLK_COUNT,
};

int bfin_dwmsi_init(bd_t *bis)
{
	struct mmc *mmc;
	const unsigned short pins[] = MSI_PORTMUX_PINS;

	/* Initialize pins */
	if ( peripheral_request_list(pins, "bfin_msi") < 0)
		return -1;

	bfin_msi_cfg.f_max = get_sclk();
	bfin_msi_cfg.f_min = bfin_msi_cfg.f_max >> 9;

	mmc = mmc_create(&bfin_msi_cfg, NULL);
	if (mmc == NULL)
		return -1;

	return 0;
}
