/*
 * Copyright 2008-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later
 */

#ifndef _MACH_PORTMUX_H_
#define _MACH_PORTMUX_H_

#define MAX_RESOURCES	MAX_BLACKFIN_GPIOS

/* PPI Port Mux */
#define P_PPI0_D0	(P_DEFINED | P_IDENT(GPIO_PF0) | P_FUNCT(1))
#define P_PPI0_D1	(P_DEFINED | P_IDENT(GPIO_PF1) | P_FUNCT(1))
#define P_PPI0_D2	(P_DEFINED | P_IDENT(GPIO_PF2) | P_FUNCT(1))
#define P_PPI0_D3	(P_DEFINED | P_IDENT(GPIO_PF3) | P_FUNCT(1))
#define P_PPI0_D4	(P_DEFINED | P_IDENT(GPIO_PF4) | P_FUNCT(1))
#define P_PPI0_D5	(P_DEFINED | P_IDENT(GPIO_PF5) | P_FUNCT(1))
#define P_PPI0_D6	(P_DEFINED | P_IDENT(GPIO_PF6) | P_FUNCT(1))
#define P_PPI0_D7	(P_DEFINED | P_IDENT(GPIO_PF7) | P_FUNCT(1))
#define P_PPI0_D8	(P_DEFINED | P_IDENT(GPIO_PF8) | P_FUNCT(1))
#define P_PPI0_D9	(P_DEFINED | P_IDENT(GPIO_PF9) | P_FUNCT(1))
#define P_PPI0_D10	(P_DEFINED | P_IDENT(GPIO_PF10) | P_FUNCT(1))
#define P_PPI0_D11	(P_DEFINED | P_IDENT(GPIO_PF11) | P_FUNCT(1))
#define P_PPI0_D12	(P_DEFINED | P_IDENT(GPIO_PF12) | P_FUNCT(1))
#define P_PPI0_D13	(P_DEFINED | P_IDENT(GPIO_PF13) | P_FUNCT(1))
#define P_PPI0_D14	(P_DEFINED | P_IDENT(GPIO_PF14) | P_FUNCT(1))
#define P_PPI0_D15	(P_DEFINED | P_IDENT(GPIO_PF15) | P_FUNCT(1))
#define P_PPI0_D16	(P_DEFINED | P_IDENT(GPIO_PE3) | P_FUNCT(1))
#define P_PPI0_D17	(P_DEFINED | P_IDENT(GPIO_PE4) | P_FUNCT(1))
#define P_PPI0_D18	(P_DEFINED | P_IDENT(GPIO_PE0) | P_FUNCT(1))
#define P_PPI0_D19	(P_DEFINED | P_IDENT(GPIO_PE1) | P_FUNCT(1))
#define P_PPI0_D20	(P_DEFINED | P_IDENT(GPIO_PD12) | P_FUNCT(1))
#define P_PPI0_D21	(P_DEFINED | P_IDENT(GPIO_PD15) | P_FUNCT(1))
#define P_PPI0_D22	(P_DEFINED | P_IDENT(GPIO_PE2) | P_FUNCT(1))
#define P_PPI0_D23	(P_DEFINED | P_IDENT(GPIO_PE5) | P_FUNCT(1))
#define P_PPI0_CLK	(P_DEFINED | P_IDENT(GPIO_PE9) | P_FUNCT(1))
#define P_PPI0_FS1	(P_DEFINED | P_IDENT(GPIO_PE8) | P_FUNCT(1))
#define P_PPI0_FS2	(P_DEFINED | P_IDENT(GPIO_PE7) | P_FUNCT(1))
#define P_PPI0_FS3	(P_DEFINED | P_IDENT(GPIO_PE6) | P_FUNCT(1))

#define P_PPI1_D0	(P_DEFINED | P_IDENT(GPIO_PC0) | P_FUNCT(1))
#define P_PPI1_D1	(P_DEFINED | P_IDENT(GPIO_PC1) | P_FUNCT(1))
#define P_PPI1_D2	(P_DEFINED | P_IDENT(GPIO_PC2) | P_FUNCT(1))
#define P_PPI1_D3	(P_DEFINED | P_IDENT(GPIO_PC3) | P_FUNCT(1))
#define P_PPI1_D4	(P_DEFINED | P_IDENT(GPIO_PC4) | P_FUNCT(1))
#define P_PPI1_D5	(P_DEFINED | P_IDENT(GPIO_PC5) | P_FUNCT(1))
#define P_PPI1_D6	(P_DEFINED | P_IDENT(GPIO_PC6) | P_FUNCT(1))
#define P_PPI1_D7	(P_DEFINED | P_IDENT(GPIO_PC7) | P_FUNCT(1))
#define P_PPI1_D8	(P_DEFINED | P_IDENT(GPIO_PC8) | P_FUNCT(1))
#define P_PPI1_D9	(P_DEFINED | P_IDENT(GPIO_PC9) | P_FUNCT(1))
#define P_PPI1_D10	(P_DEFINED | P_IDENT(GPIO_PC10) | P_FUNCT(1))
#define P_PPI1_D11	(P_DEFINED | P_IDENT(GPIO_PC11) | P_FUNCT(1))
#define P_PPI1_D12	(P_DEFINED | P_IDENT(GPIO_PC12) | P_FUNCT(1))
#define P_PPI1_D13	(P_DEFINED | P_IDENT(GPIO_PC13) | P_FUNCT(1))
#define P_PPI1_D14	(P_DEFINED | P_IDENT(GPIO_PC14) | P_FUNCT(1))
#define P_PPI1_D15	(P_DEFINED | P_IDENT(GPIO_PC15) | P_FUNCT(1))
#define P_PPI1_D16	(P_DEFINED | P_IDENT(GPIO_PD0) | P_FUNCT(1))
#define P_PPI1_D17	(P_DEFINED | P_IDENT(GPIO_PD1) | P_FUNCT(1))
#define P_PPI1_CLK	(P_DEFINED | P_IDENT(GPIO_PB14) | P_FUNCT(1))
#define P_PPI1_FS1	(P_DEFINED | P_IDENT(GPIO_PB13) | P_FUNCT(1))
#define P_PPI1_FS2	(P_DEFINED | P_IDENT(GPIO_PD6) | P_FUNCT(1))
#define P_PPI1_FS3	(P_DEFINED | P_IDENT(GPIO_PB15) | P_FUNCT(1))

#define P_PPI2_D0	(P_DEFINED | P_IDENT(GPIO_PA0) | P_FUNCT(1))
#define P_PPI2_D1	(P_DEFINED | P_IDENT(GPIO_PA1) | P_FUNCT(1))
#define P_PPI2_D2	(P_DEFINED | P_IDENT(GPIO_PA2) | P_FUNCT(1))
#define P_PPI2_D3	(P_DEFINED | P_IDENT(GPIO_PA3) | P_FUNCT(1))
#define P_PPI2_D4	(P_DEFINED | P_IDENT(GPIO_PA4) | P_FUNCT(1))
#define P_PPI2_D5	(P_DEFINED | P_IDENT(GPIO_PA5) | P_FUNCT(1))
#define P_PPI2_D6	(P_DEFINED | P_IDENT(GPIO_PA6) | P_FUNCT(1))
#define P_PPI2_D7	(P_DEFINED | P_IDENT(GPIO_PA7) | P_FUNCT(1))
#define P_PPI2_D8	(P_DEFINED | P_IDENT(GPIO_PA8) | P_FUNCT(1))
#define P_PPI2_D9	(P_DEFINED | P_IDENT(GPIO_PA9) | P_FUNCT(1))
#define P_PPI2_D10	(P_DEFINED | P_IDENT(GPIO_PA10) | P_FUNCT(1))
#define P_PPI2_D11	(P_DEFINED | P_IDENT(GPIO_PA11) | P_FUNCT(1))
#define P_PPI2_D12	(P_DEFINED | P_IDENT(GPIO_PA12) | P_FUNCT(1))
#define P_PPI2_D13	(P_DEFINED | P_IDENT(GPIO_PA13) | P_FUNCT(1))
#define P_PPI2_D14	(P_DEFINED | P_IDENT(GPIO_PA14) | P_FUNCT(1))
#define P_PPI2_D15	(P_DEFINED | P_IDENT(GPIO_PA15) | P_FUNCT(1))
#define P_PPI2_D16	(P_DEFINED | P_IDENT(GPIO_PB7) | P_FUNCT(1))
#define P_PPI2_D17	(P_DEFINED | P_IDENT(GPIO_PB8) | P_FUNCT(1))
#define P_PPI2_CLK	(P_DEFINED | P_IDENT(GPIO_PB0) | P_FUNCT(1))
#define P_PPI2_FS1	(P_DEFINED | P_IDENT(GPIO_PB1) | P_FUNCT(1))
#define P_PPI2_FS2	(P_DEFINED | P_IDENT(GPIO_PB2) | P_FUNCT(1))
#define P_PPI2_FS3	(P_DEFINED | P_IDENT(GPIO_PB3) | P_FUNCT(1))

/* SPI Port Mux */
#define P_SPI0_SS	(P_DEFINED | P_IDENT(GPIO_PA5) | P_FUNCT(3))
#define P_SPI0_SCK	(P_DEFINED | P_IDENT(GPIO_PB0) | P_FUNCT(2))
#define P_SPI0_MISO	(P_DEFINED | P_IDENT(GPIO_PB1) | P_FUNCT(2))
#define P_SPI0_MOSI	(P_DEFINED | P_IDENT(GPIO_PB2) | P_FUNCT(2))
#define P_SPI0_RDY	(P_DEFINED | P_IDENT(GPIO_PA6) | P_FUNCT(2))
#define P_SPI0_D2	(P_DEFINED | P_IDENT(GPIO_PB3) | P_FUNCT(2))
#define P_SPI0_D3	(P_DEFINED | P_IDENT(GPIO_PB7) | P_FUNCT(2))

#define P_SPI0_SSEL1	(P_DEFINED | P_IDENT(GPIO_PA5) | P_FUNCT(1))
#define P_SPI0_SSEL2	(P_DEFINED | P_IDENT(GPIO_PA6) | P_FUNCT(1))
#define P_SPI0_SSEL3	(P_DEFINED | P_IDENT(GPIO_PC11) | P_FUNCT(2))
#define P_SPI0_SSEL4	(P_DEFINED | P_IDENT(GPIO_PB4) | P_FUNCT(2))
#define P_SPI0_SSEL5	(P_DEFINED | P_IDENT(GPIO_PB5) | P_FUNCT(2))
#define P_SPI0_SSEL6	(P_DEFINED | P_IDENT(GPIO_PB6) | P_FUNCT(2))

#define P_SPI1_SS	(P_DEFINED | P_IDENT(GPIO_PA4) | P_FUNCT(3))
#define P_SPI1_SCK	(P_DEFINED | P_IDENT(GPIO_PA0) | P_FUNCT(0))
#define P_SPI1_MISO	(P_DEFINED | P_IDENT(GPIO_PA1) | P_FUNCT(0))
#define P_SPI1_MOSI	(P_DEFINED | P_IDENT(GPIO_PA2) | P_FUNCT(0))
#define P_SPI1_RDY	(P_DEFINED | P_IDENT(GPIO_PA3) | P_FUNCT(1))

#define P_SPI1_SSEL1	(P_DEFINED | P_IDENT(GPIO_PA4) | P_FUNCT(0))
#define P_SPI1_SSEL2	(P_DEFINED | P_IDENT(GPIO_PA3) | P_FUNCT(0))
#define P_SPI1_SSEL3	(P_DEFINED | P_IDENT(GPIO_PC10) | P_FUNCT(2))
#define P_SPI1_SSEL4	(P_DEFINED | P_IDENT(GPIO_PA14) | P_FUNCT(1))

#define P_SPI2_SS	(P_DEFINED | P_IDENT(GPIO_PB15) | P_FUNCT(3))
#define P_SPI2_SCK	(P_DEFINED | P_IDENT(GPIO_PB10) | P_FUNCT(0))
#define P_SPI2_MISO	(P_DEFINED | P_IDENT(GPIO_PB11) | P_FUNCT(0))
#define P_SPI2_MOSI	(P_DEFINED | P_IDENT(GPIO_PB12) | P_FUNCT(0))
#define P_SPI2_RDY	(P_DEFINED | P_IDENT(GPIO_PA4) | P_FUNCT(2))
#define P_SPI2_D2	(P_DEFINED | P_IDENT(GPIO_PB13) | P_FUNCT(0))
#define P_SPI2_D3	(P_DEFINED | P_IDENT(GPIO_PB14) | P_FUNCT(0))

#define P_SPI2_SSEL1	(P_DEFINED | P_IDENT(GPIO_PB15) | P_FUNCT(0))
#define P_SPI2_SSEL2	(P_DEFINED | P_IDENT(GPIO_PB15) | P_FUNCT(0))
#define P_SPI2_SSEL3	(P_DEFINED | P_IDENT(GPIO_PB15) | P_FUNCT(0))
#define P_SPI2_SSEL4	(P_DEFINED | P_IDENT(GPIO_PB15) | P_FUNCT(0))
#define P_SPI2_SSEL5	(P_DEFINED | P_IDENT(GPIO_PB15) | P_FUNCT(0))
#define P_SPI2_SSEL6	(P_DEFINED | P_IDENT(GPIO_PB15) | P_FUNCT(0))

#define GPIO_DEFAULT_BOOT_SPI_CS
#define P_DEFAULT_BOOT_SPI_CS

/* UART Port Mux */
#define P_UART0_TX	(P_DEFINED | P_IDENT(GPIO_PB8) | P_FUNCT(0))
#define P_UART0_RX	(P_DEFINED | P_IDENT(GPIO_PB9) | P_FUNCT(0))
#define P_UART0_RTS	(P_DEFINED | P_IDENT(GPIO_PC2) | P_FUNCT(0))
#define P_UART0_CTS	(P_DEFINED | P_IDENT(GPIO_PC3) | P_FUNCT(0))

#define P_UART1_TX	(P_DEFINED | P_IDENT(GPIO_PC0) | P_FUNCT(0))
#define P_UART1_RX	(P_DEFINED | P_IDENT(GPIO_PC1) | P_FUNCT(0))
#define P_UART1_RTS	(P_DEFINED | P_IDENT(GPIO_PB13) | P_FUNCT(1))
#define P_UART1_CTS	(P_DEFINED | P_IDENT(GPIO_PB14) | P_FUNCT(1))

/* Timer */
#define P_TMRCLK	(P_DEFINED | P_IDENT(GPIO_PG13) | P_FUNCT(3))
#define P_TMR0		(P_DEFINED | P_IDENT(GPIO_PE14) | P_FUNCT(2))
#define P_TMR1		(P_DEFINED | P_IDENT(GPIO_PG4) | P_FUNCT(1))
#define P_TMR2		(P_DEFINED | P_IDENT(GPIO_PG1) | P_FUNCT(1))
#define P_TMR3		(P_DEFINED | P_IDENT(GPIO_PG8) | P_FUNCT(1))
#define P_TMR4		(P_DEFINED | P_IDENT(GPIO_PG9) | P_FUNCT(1))
#define P_TMR5		(P_DEFINED | P_IDENT(GPIO_PG7) | P_FUNCT(1))
#define P_TMR6		(P_DEFINED | P_IDENT(GPIO_PG11) | P_FUNCT(1))
#define P_TMR7		(P_DEFINED | P_IDENT(GPIO_PG12) | P_FUNCT(1))

/* MSI */
#define P_MSI_DATA0	(P_DEFINED | P_IDENT(GPIO_PC8) | P_FUNCT(2))
#define P_MSI_DATA1	(P_DEFINED | P_IDENT(GPIO_PC4) | P_FUNCT(2))
#define P_MSI_DATA2	(P_DEFINED | P_IDENT(GPIO_PC7) | P_FUNCT(2))
#define P_MSI_DATA3	(P_DEFINED | P_IDENT(GPIO_PC6) | P_FUNCT(2))
#define P_MSI_DATA4	(P_DEFINED | P_IDENT(GPIO_PC10) | P_FUNCT(1))
#define P_MSI_DATA5	(P_DEFINED | P_IDENT(GPIO_PC11) | P_FUNCT(1))
#define P_MSI_DATA6	(P_DEFINED | P_IDENT(GPIO_PC12) | P_FUNCT(1))
#define P_MSI_DATA7	(P_DEFINED | P_IDENT(GPIO_PC13) | P_FUNCT(1))
#define P_MSI_CMD	(P_DEFINED | P_IDENT(GPIO_PC5) | P_FUNCT(2))
#define P_MSI_CLK	(P_DEFINED | P_IDENT(GPIO_PC9) | P_FUNCT(2))
#define P_MSI_CD	(P_DEFINED | P_IDENT(GPIO_PA8) | P_FUNCT(1))

/* PTP */
#define P_PTP0_PPS	(P_DEFINED | P_IDENT(GPIO_PB15) | P_FUNCT(0))
#define P_PTP0_CLKIN	(P_DEFINED | P_IDENT(GPIO_PC13) | P_FUNCT(2))
#define P_PTP0_AUXIN	(P_DEFINED | P_IDENT(GPIO_PC11) | P_FUNCT(2))

#define P_PTP1_PPS	(P_DEFINED | P_IDENT(GPIO_PC9) | P_FUNCT(0))
#define P_PTP1_CLKIN	(P_DEFINED | P_IDENT(GPIO_PC13) | P_FUNCT(2))
#define P_PTP1_AUXIN	(P_DEFINED | P_IDENT(GPIO_PC11) | P_FUNCT(2))

/* SMC Port Mux */
#define P_A3		(P_DEFINED | P_IDENT(GPIO_PA0) | P_FUNCT(0))
#define P_A4		(P_DEFINED | P_IDENT(GPIO_PA1) | P_FUNCT(0))
#define P_A5		(P_DEFINED | P_IDENT(GPIO_PA2) | P_FUNCT(0))
#define P_A6		(P_DEFINED | P_IDENT(GPIO_PA3) | P_FUNCT(0))
#define P_A7		(P_DEFINED | P_IDENT(GPIO_PA4) | P_FUNCT(0))
#define P_A8		(P_DEFINED | P_IDENT(GPIO_PA5) | P_FUNCT(0))
#define P_A9		(P_DEFINED | P_IDENT(GPIO_PA6) | P_FUNCT(0))
#define P_A10		(P_DEFINED | P_IDENT(GPIO_PA7) | P_FUNCT(0))
#define P_A11		(P_DEFINED | P_IDENT(GPIO_PA8) | P_FUNCT(1))
#define P_A12		(P_DEFINED | P_IDENT(GPIO_PA9) | P_FUNCT(0))
#define P_A13		(P_DEFINED | P_IDENT(GPIO_PB2) | P_FUNCT(0))
#define P_A14		(P_DEFINED | P_IDENT(GPIO_PA10) | P_FUNCT(0))
#define P_A15		(P_DEFINED | P_IDENT(GPIO_PA11) | P_FUNCT(0))
#define P_A16		(P_DEFINED | P_IDENT(GPIO_PB3) | P_FUNCT(0))
#define P_A17		(P_DEFINED | P_IDENT(GPIO_PA12) | P_FUNCT(0))
#define P_A18		(P_DEFINED | P_IDENT(GPIO_PA13) | P_FUNCT(0))
#define P_A19		(P_DEFINED | P_IDENT(GPIO_PA14) | P_FUNCT(0))
#define P_A20		(P_DEFINED | P_IDENT(GPIO_PA15) | P_FUNCT(0))
#define P_A21		(P_DEFINED | P_IDENT(GPIO_PB6) | P_FUNCT(0))
#define P_A22		(P_DEFINED | P_IDENT(GPIO_PB7) | P_FUNCT(0))
#define P_A23		(P_DEFINED | P_IDENT(GPIO_PB8) | P_FUNCT(0))
#define P_A24		(P_DEFINED | P_IDENT(GPIO_PB10) | P_FUNCT(0))
#define P_A25		(P_DEFINED | P_IDENT(GPIO_PB11) | P_FUNCT(0))
#define P_NORCK         (P_DEFINED | P_IDENT(GPIO_PB0) | P_FUNCT(0))

#define P_AMS1		(P_DEFINED | P_IDENT(GPIO_PB1) | P_FUNCT(0))
#define P_AMS2		(P_DEFINED | P_IDENT(GPIO_PB4) | P_FUNCT(0))
#define P_AMS3		(P_DEFINED | P_IDENT(GPIO_PB5) | P_FUNCT(0))

#define P_ABE0		(P_DEFINED | P_IDENT(GPIO_PB4) | P_FUNCT(1))
#define P_ABE1		(P_DEFINED | P_IDENT(GPIO_PB5) | P_FUNCT(1))

/* CAN */
#define P_CAN0_TX	(P_DEFINED | P_IDENT(GPIO_PG1) | P_FUNCT(2))
#define P_CAN0_RX	(P_DEFINED | P_IDENT(GPIO_PG4) | P_FUNCT(2))

#endif				/* _MACH_PORTMUX_H_ */
