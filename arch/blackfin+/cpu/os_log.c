/*
 * functions for handling OS log buffer
 *
 * Copyright (c) 2009 Analog Devices Inc.
 *
 * Licensed under the Clear BSD.
 *
 * ADSP-BF70x fix by DAB-Embedded, 2014 (www.dab-embedded.com)
 *
 */

#include <common.h>

#define OS_LOG_MAGIC       0xDEADBEEF
#define OS_LOG_MAGIC_ADDR  ((unsigned long *)CONFIG_SYS_SDRAM_BASE + 0x4f0)
#define OS_LOG_PTR_ADDR    ((char **)CONFIG_SYS_SDRAM_BASE + 0x4f4)

int bfin_os_log_check(void)
{
	if (*OS_LOG_MAGIC_ADDR != OS_LOG_MAGIC)
		return 0;
	*OS_LOG_MAGIC_ADDR = 0;
	return 1;
}

void bfin_os_log_dump(void)
{
	char *log = *OS_LOG_PTR_ADDR;
	while (*log) {
		puts(log);
		log += strlen(log) + 1;
	}
}
