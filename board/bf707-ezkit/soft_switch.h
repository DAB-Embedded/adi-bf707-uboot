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

#ifndef __BOARD_SOFT_SWITCH_H__
#define __BOARD_SOFT_SWITCH_H__

#include <asm/soft_switch.h>

/* switch 0 port A */
#define SW0_SPF_CS_nEN         0x01
#define SW0_SPF_D2_nEN         0x02
#define SW0_SPF_D3_nEN         0x04
#define SW0_RF_SPI2_SEL1_nEN   0x08
#define SW0_SD_CS_nEN          0x10
#define SW0_SD_WP_nEN          0x20
#define SW0_RF_SPI2_SEL2_nEN   0x80

/* switch 0 port B */
#define SW0_CAN0_EN            0x01
#define SW0_CAN0_nSTB          0x02
#define SW0_CAN1_EN            0x04
#define SW0_CAN1_nSTB          0x08
#define SW0_CAN0_ERR_nEN       0x40
#define SW0_CAN1_ERR_nEN       0x80

/* switch 1 port A */
#define SW1_CAN0_TX_nEN        0x01
#define SW1_CAN1_TX_nEN        0x02
#define SW1_CAN0_RX_nEN        0x04
#define SW1_CAN1_RX_nEN        0x08
#define SW1_UART0_nEN          0x10
#define SW1_UART0RTS_nEN       0x20
#define SW1_UART0CTS_nEN       0x40

/* switch 1 port B */
#define SW1_LED1_GPIO1_nEN     0x04
#define SW1_LED2_GPIO2_nEN     0x08
#define SW1_LED3_GPIO3_nEN     0x10
#define SW1_PB0_nEN            0x20
#define SW1_PB1_nEN            0x40

#define NUM_SWITCH      2
#define IODIRA          0x0
#define IODIRB          0x1
#define OLATA           0x14
#define OLATB           0x15

#define SW0_I2C_ADDR    0x21
#define SW1_I2C_ADDR    0x22

int setup_board_switches(void);

#endif /* __BOARD_SOFT_SWITCH_H__ */
