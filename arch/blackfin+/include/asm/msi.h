/*
 * msi.h, export bfin_mmc_init
 *
 * ADSP-BF70x MSI header by DAB-Embedded, 2014 (www.dab-embedded.com)
 *
 */

#ifndef __ASM_MSI_H__
#define __ASM_MSI_H__

#include <mmc.h>
#include <asm/u-boot.h>

int bfin_dwmsi_init(bd_t *bis);

#endif
