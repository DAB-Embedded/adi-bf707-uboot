/*
 * Copyright (C) 2008 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 *
 * ADSP-BF70x fix by DAB-Embedded, 2014 (www.dab-embedded.com)
 *
 */

#ifndef _MACH_GPIO_H_
#define _MACH_GPIO_H_

#define MAX_BLACKFIN_GPIOS 47

#define GPIO_PA0	0
#define GPIO_PA1	1
#define GPIO_PA2	2
#define GPIO_PA3	3
#define GPIO_PA4	4
#define GPIO_PA5	5
#define GPIO_PA6	6
#define GPIO_PA7	7
#define GPIO_PA8	8
#define GPIO_PA9	9
#define GPIO_PA10	10
#define GPIO_PA11	11
#define GPIO_PA12	12
#define GPIO_PA13	13
#define GPIO_PA14	14
#define GPIO_PA15	15
#define GPIO_PB0	16
#define GPIO_PB1	17
#define GPIO_PB2	18
#define GPIO_PB3	19
#define GPIO_PB4	20
#define GPIO_PB5	21
#define GPIO_PB6	22
#define GPIO_PB7	23
#define GPIO_PB8	24
#define GPIO_PB9	25
#define GPIO_PB10	26
#define GPIO_PB11	27
#define GPIO_PB12	28
#define GPIO_PB13	29
#define GPIO_PB14	30
#define GPIO_PB15	31
#define GPIO_PC0	32
#define GPIO_PC1	33
#define GPIO_PC2	34
#define GPIO_PC3	35
#define GPIO_PC4	36
#define GPIO_PC5	37
#define GPIO_PC6	38
#define GPIO_PC7	39
#define GPIO_PC8	40
#define GPIO_PC9	41
#define GPIO_PC10	42
#define GPIO_PC11	43
#define GPIO_PC12	44
#define GPIO_PC13	45
#define GPIO_PC14	46


#ifndef __ASSEMBLY__

struct gpio_port_t {
	unsigned long port_fer;
	unsigned long port_fer_set;
	unsigned long port_fer_clear;
	unsigned long data;
	unsigned long data_set;
	unsigned long data_clear;
	unsigned long dir;
	unsigned long dir_set;
	unsigned long dir_clear;
	unsigned long inen;
	unsigned long inen_set;
	unsigned long inen_clear;
	unsigned long port_mux;
	unsigned long toggle;
	unsigned long polar;
	unsigned long polar_set;
	unsigned long polar_clear;
	unsigned long lock;
	unsigned long spare;
	unsigned long revid;
};

#endif

#endif /* _MACH_GPIO_H_ */
