/*
 * U-boot - main board file
 *
 * ADSP-BF70x by DAB-Embedded, 2014 (www.dab-embedded.com)
 */

#include <common.h>
#include <netdev.h>
#include <asm/blackfin.h>
#include <asm/io.h>
#include <asm/msi.h>
#include <asm/portmux.h>
#include "soft_switch.h"

int checkboard(void)
{
	printf("Board: ADI BF707 EZ-Kit board\n");
	printf("       Support: http://dab-embedded.com/\n");
	return 0;
}

int board_early_init_f(void)
{
	
	return 0;
}

#ifdef CONFIG_DESIGNWARE_ETH
int board_eth_init(bd_t *bis)
{
	int ret = 1;



	return ret;
}
#endif

#ifdef CONFIG_BFIN_MSI
int board_mmc_init(bd_t *bis)
{
	return bfin_dwmsi_init(bis);
}
#endif

/* miscellaneous platform dependent initialisations */
int misc_init_r(void)
{
	printf("other init\n");
	return setup_board_switches();
}
