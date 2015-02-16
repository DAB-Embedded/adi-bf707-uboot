/*
 * U-boot - Configuration file for BF707 EZ-Kit board
 */

#ifndef __CONFIG_BF707_EZKIT_H__
#define __CONFIG_BF707_EZKIT_H__

#include <asm/config-pre.h>


/*
 * Processor Settings
 */
#define CONFIG_BFIN_CPU             bf707

#define SPI_FLASH_LOAD		    0

#if (SPI_FLASH_LOAD == 1)
#define CONFIG_BFIN_BOOT_MODE       BFIN_BOOT_SPI_MASTER
#else
#define CONFIG_BFIN_BOOT_MODE       BFIN_BOOT_UART
#endif

#define CONFIG_SYS_SDRAM_BASE       0x80000000
#define CONFIG_LOADADDR             0x81000000
#define CONFIG_SYS_NO_FLASH
#define CONFIG_BAUDRATE             115200


/*
 * Clock Settings
 *	CCLK = (CLKIN * VCO_MULT) / CCLK_DIV     = 400MHz
 *	SCLK = (CLKIN * VCO_MULT) / SYSCLK_DIV   = 200MHz
 *	SCLK0 = SCLK / SCLK0_DIV                 = 100MHz
 *	SCLK1 = SCLK / SCLK1_DIV                 = 200MHz
 */
/* CONFIG_CLKIN_HZ is any value in Hz					*/
#define CONFIG_CLKIN_HZ			(25000000)
#define CONFIG_USB_BLACKFIN_CLKIN 24

/* CLKIN_HALF controls the DF bit in PLL_CTL      0 = CLKIN		*/
/*                                                1 = CLKIN / 2		*/
#define CONFIG_CLKIN_HALF		(0)

/* VCO_MULT controls the MSEL (multiplier) bits in PLL_CTL		*/
/* Values can range from 0-127 (where 0 means 128)			*/
#define CONFIG_VCO_MULT			(16)

/* CCLK_DIV controls the core clock divider				*/
/* Values can range from 0-31 (where 0 means 32)			*/
#define CONFIG_CCLK_DIV			(1)
/* SCLK_DIV controls the system clock divider				*/
/* Values can range from 0-31 (where 0 means 32)			*/
#define CONFIG_SCLK_DIV			(2)
/* Values can range from 0-7 (where 0 means 8)				*/
#define CONFIG_SCLK0_DIV		(2)
#define CONFIG_SCLK1_DIV		(1)
/* DCLK_DIV controls the DDR clock divider				*/
/* Values can range from 0-31 (where 0 means 32)			*/
#define CONFIG_DCLK_DIV			(2)
/* OCLK_DIV controls the output clock divider				*/
/* Values can range from 0-127 (where 0 means 128)			*/
#define CONFIG_OCLK_DIV			(4)

/*
 * Memory Settings
 */
#define CONFIG_MEM_SIZE		128

#define CONFIG_SYS_MONITOR_LEN	(768 * 1024)
#define CONFIG_SYS_MALLOC_LEN	(512 * 1024)

#define CONFIG_HW_WATCHDOG

/*
 * Network Settings
 */
#define CONFIG_HOSTNAME		"bf707-ezkit"
#define CONFIG_CMD_NET

/* i2c Settings */
#define CONFIG_ADI_I2C
#define CONFIG_HARD_I2C

/*
 * Flash Settings
 */
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_JFFS2

/*
 * SPI Settings
 */
#define CONFIG_ADI_SPI3
#define CONFIG_BFIN_SPI6XX
#define CONFIG_ENV_SPI_MAX_HZ	25000000
#define CONFIG_SF_DEFAULT_SPEED	25000000
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_ALL
#define CONFIG_DEFAULT_SPI_BUS  2
#define CONFIG_SF_DEFAULT_BUS   2
#define CONFIG_ENV_SPI_BUS      2
#define CONFIG_DEFAULT_SPI_CS   1
#define CONFIG_SF_DEFAULT_CS    1
#define CONFIG_ENV_SPI_CS       1

#define CONFIG_USB
#define CONFIG_MUSB_HCD
#define CONFIG_USB_BLACKFIN
#define CONFIG_USB_STORAGE
#define CONFIG_MUSB_TIMEOUT 100000
#define MUSB_HW_VERSION2

/*
 * Env Storage Settings
 */
#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_OFFSET       0x10000
#define CONFIG_ENV_SIZE         0x2000
#define CONFIG_ENV_SECT_SIZE    0x10000
#define CONFIG_ENV_IS_EMBEDDED_IN_LDR

#define FLASHBOOT_ENV_SETTINGS "ramboot=fatload mmc 0 ${loadaddr} uImage; bootm\0"

/*
 * MSI Settings
 */
#define CONFIG_GENERIC_MMC
#define CONFIG_MMC
#define CONFIG_BFIN_MSI

/*
 * Misc Settings
 */
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_UART_CONSOLE	0

#define CONFIG_CMD_MEMORY
#define CONFIG_CMD_SOFTSWITCH

#define CONFIG_MISC_INIT_R
#define CONFIG_BFIN_SOFT_SWITCH

#define CONFIG_ADI_GPIO2

#define CONFIG_SYS_MEMTEST_END (CONFIG_STACKBASE - 20*1024*1024 + 4)

#if 0
#define CONFIG_UART_MEM 1024
#undef CONFIG_UART_CONSOLE
#undef CONFIG_JTAG_CONSOLE
#undef CONFIG_UART_CONSOLE_IS_JTAG
#endif

#define CONFIG_BOARD_SIZE_LIMIT $$(( 512 * 1024 ))

/*
 * Pull in common ADI header for remaining command/environment setup
 */
#include <configs/bfin_adi_common.h>
#endif
