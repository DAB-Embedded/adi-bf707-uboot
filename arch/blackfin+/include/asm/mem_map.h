/*
 * Common Blackfin memory map
 *
 * Copyright 2004-2009 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#ifndef __BFIN_MEM_MAP_H__
#define __BFIN_MEM_MAP_H__

/* Every Blackfin so far has MMRs like this */
#ifndef COREMMR_BASE
# define COREMMR_BASE 0x1FC00000
#endif
#ifndef SYSMMR_BASE
# define SYSMMR_BASE  0x20000000
#endif
#ifndef SYSMMR_BASE_END
# define SYSMMR_BASE_END  0x20300000
#endif

/* Every Blackfin so far has on-chip Scratch Pad SRAM like this */
#ifndef L1_SRAM_SCRATCH
# define L1_SRAM_SCRATCH      0x11B00000
# define L1_SRAM_SCRATCH_SIZE 0x1000
# define L1_SRAM_SCRATCH_END  (L1_SRAM_SCRATCH + L1_SRAM_SCRATCH_SIZE)
#endif

#endif
