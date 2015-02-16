/*
 * Code for early processor initialization
 *
 * Copyright (c) 2004-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 *
 * ADSP-BF70x fix by DAB-Embedded, 2014 (www.dab-embedded.com)
 *
 */

#ifndef __BFIN_INITCODE_H__
#define __BFIN_INITCODE_H__

#include <asm/mach-common/bits/bootrom.h>

#ifndef BFIN_IN_INITCODE
# define serial_putc(c)
#endif

__attribute__((always_inline)) static inline void
program_async_controller(ADI_BOOT_DATA *bs)
{

	serial_putc('d');
}

#endif
