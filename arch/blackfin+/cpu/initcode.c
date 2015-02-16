/*
 * initcode.c - Initialize the processor.  This is usually entails things
 * like external memory, voltage regulators, etc...  Note that this file
 * cannot make any function calls as it may be executed all by itself by
 * the Blackfin's bootrom in LDR format.
 *
 * Copyright (c) 2004-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 *
 * ADSP-BF70x fix by DAB-Embedded, 2014 (www.dab-embedded.com)
 *
 */

#define BFIN_IN_INITCODE

#include <config.h>
#include <asm/blackfin.h>
#include <asm/mach-common/bits/watchdog.h>
#include <asm/mach-common/bits/bootrom.h>
#include <asm/mach-common/bits/core.h>
#include <asm/serial.h>


#include <asm/mach-common/bits/cgu.h>

#define CONFIG_BFIN_GET_DCLK_M \
	((CONFIG_CLKIN_HZ*CONFIG_VCO_MULT)/(CONFIG_DCLK_DIV*1000000))

/* DMC control bits */
#define SRREQ			0x8

/* DMC status bits */
#define IDLE                    0x1
#define MEMINITDONE             0x4
#define SRACK                   0x8
#define PDACK                   0x10
#define DPDACK                  0x20
#define DLLCALDONE              0x2000
#define PENDREF                 0xF0000
#define PHYRDPHASE              0xF00000
#define PHYRDPHASE_OFFSET       20

/* DMC DLL control bits */
#define DLLCALRDCNT             0xFF
#define DATACYC_OFFSET          8

/* DMC PHY & PAD values */
#define BITM_DMC_PAD_CTL0_PUCALEN            (0x20000000)
#define BITM_DMC_PAD_CTL0_PDCALEN            (0x40000000)
#define BITM_DMC_PAD_CTL0_RTTCALEN           (0x80000000)

#define ENUM_DMC_PHY_CTL4_DDR2               (0x00000001)
#define DMC0_PADCTL0_VALUE                   (BITM_DMC_PAD_CTL0_PUCALEN | BITM_DMC_PAD_CTL0_PDCALEN | BITM_DMC_PAD_CTL0_RTTCALEN)
#define DMC0_PADCTL2_VALUE                   (0x0078283C)
#define BITM_DMC_CAL_PADCTL0_CALSTRT         (0x10000000)
#define BITM_DMC_STAT_INITDONE               (0x00000004) 



__attribute__((always_inline))
static inline void led_out(int led)
{
	/* LED On */
	bfin_write32(PORTA_DIR_SET, 0x3);
	bfin_write32(PORTB_DIR_SET, 0x2);

	bfin_write32(PORTA_DATA_CLR, 0x3);
	bfin_write32(PORTB_DATA_CLR, 0x2);

	bfin_write32(PORTA_DATA_SET, (led & 0x3));
	bfin_write32(PORTB_DATA_SET, (led & 0x4)>>1);
}

__attribute__((always_inline))
static inline void serial_init(void)
{
#if defined(__ADSPBF70x__)
# ifdef BFIN_BOOT_UART_USE_RTS
#  define BFIN_UART_USE_RTS 1
# else
#  define BFIN_UART_USE_RTS 0
# endif
	if (BFIN_UART_USE_RTS && CONFIG_BFIN_BOOT_MODE == BFIN_BOOT_UART) {
		uint32_t uart_base = UART_BASE;
		size_t i;

		/* force RTS rather than relying on auto RTS */
#if BFIN_UART_HW_VER < 4
		bfin_write16(&pUART->mcr, bfin_read16(&pUART->mcr) | FCPOL);
#else
		bfin_write32(&pUART->control, bfin_read32(&pUART->control) |
				FCPOL);
#endif

		/* Wait for the line to clear up.  We cannot rely on UART
		 * registers as none of them reflect the status of the RSR.
		 * Instead, we'll sleep for ~10 bit times at 9600 baud.
		 * We can precalc things here by assuming boot values for
		 * PLL rather than loading registers and calculating.
		 *	baud    = SCLK / (16 ^ (1 - EDBO) * Divisor)
		 *	EDB0    = 0
		 *	Divisor = (SCLK / baud) / 16
		 *	SCLK    = baud * 16 * Divisor
		 *	SCLK    = (0x14 * CONFIG_CLKIN_HZ) / 5
		 *	CCLK    = (16 * Divisor * 5) * (9600 / 10)
		 * In reality, this will probably be just about 1 second delay,
		 * so assuming 9600 baud is OK (both as a very low and too high
		 * speed as this will buffer things enough).
		 */
#define _NUMBITS (10)                                   /* how many bits to delay */
#define _LOWBAUD (9600)                                 /* low baud rate */
#define _SCLK    ((0x10 * CONFIG_CLKIN_HZ) / 2)         /* SCLK based on PLL */
#define _DIVISOR ((_SCLK / _LOWBAUD) / 16)              /* UART DLL/DLH */
#define _NUMINS  (3)                                    /* how many instructions in loop */
#define _CCLK    (((16 * _DIVISOR * 5) * (_LOWBAUD / _NUMBITS)) / _NUMINS)
		i = _CCLK;
		while (i--)
			asm volatile("" : : : "memory");
	}
#endif

#if CONFIG_BFIN_BOOT_MODE != BFIN_BOOT_BYPASS
	if (BFIN_DEBUG_EARLY_SERIAL) {
		serial_early_init(UART_BASE);
		serial_early_set_baud(UART_BASE, CONFIG_BAUDRATE);
	}
#endif
}

__attribute__((always_inline))
static inline void serial_deinit(void)
{
#if defined(__ADSPBF70x__)
	uint32_t uart_base = UART_BASE;

	if (BFIN_UART_USE_RTS && CONFIG_BFIN_BOOT_MODE == BFIN_BOOT_UART) {
		/* clear forced RTS rather than relying on auto RTS */
#if BFIN_UART_HW_VER < 4
		bfin_write16(&pUART->mcr, bfin_read16(&pUART->mcr) & ~FCPOL);
#else
		bfin_write32(&pUART->control, bfin_read32(&pUART->control) &
				~FCPOL);
#endif
	}
#endif
}

__attribute__((always_inline))
static inline void serial_putc(char c)
{
	uint32_t uart_base = UART_BASE;

	if (!BFIN_DEBUG_EARLY_SERIAL)
		return;

	if (c == '\n')
		serial_putc('\r');

	bfin_write(&pUART->thr, c);

	while (!(_lsr_read(pUART) & TEMT))
		continue;
}

#include "initcode.h"

__attribute__((always_inline)) static inline void
program_nmi_handler(void)
{
	u32 tmp1, tmp2;

	/* Older bootroms don't create a dummy NMI handler,
	 * so make one ourselves ASAP in case it fires.
	 */
	if (CONFIG_BFIN_BOOT_MODE != BFIN_BOOT_BYPASS && !ANOMALY_05000219)
		return;

	asm volatile (
		"%0 = RETS;" /* Save current RETS */
		"CALL 1f;"   /* Figure out current PC */
		"RTN;"       /* The simple NMI handler */
		"1:"
		"%1 = RETS;" /* Load addr of NMI handler */
		"RETS = %0;" /* Restore RETS */
		"[%2] = %1;" /* Write NMI handler */
		: "=d"(tmp1), "=d"(tmp2)
		: "ab"(EVT2)
	);
}

/* Max SCLK can be 133MHz ... dividing that by (2*4) gives
 * us a freq of 16MHz for SPI which should generally be
 * slow enough for the slow reads the bootrom uses.
 */
#if !defined(CONFIG_SPI_FLASH_SLOW_READ) && \
    ((defined(__ADSPBF52x__) && __SILICON_REVISION__ >= 2) || \
     (defined(__ADSPBF54x__) && __SILICON_REVISION__ >= 1))
# define BOOTROM_SUPPORTS_SPI_FAST_READ 1
#else
# define BOOTROM_SUPPORTS_SPI_FAST_READ 0
#endif
#ifndef CONFIG_SPI_BAUD_INITBLOCK
# define CONFIG_SPI_BAUD_INITBLOCK (BOOTROM_SUPPORTS_SPI_FAST_READ ? 2 : 4)
#endif
#ifdef SPI0_BAUD
# define bfin_write_SPI_BAUD bfin_write_SPI0_BAUD
#endif

#ifndef CONFIG_CGU_CTL_VAL
# define CONFIG_CGU_CTL_VAL ((CONFIG_VCO_MULT << 8) | CONFIG_CLKIN_HALF)
#endif

#ifndef CONFIG_CGU_DIV_VAL
# define CONFIG_CGU_DIV_VAL \
	((CONFIG_CCLK_DIV   << CSEL_P)   | \
	 (CONFIG_SCLK0_DIV  << S0SEL_P)  | \
	 (CONFIG_SCLK_DIV << SYSSEL_P) | \
	 (CONFIG_SCLK1_DIV  << S1SEL_P)  | \
	 (CONFIG_DCLK_DIV   << DSEL_P)   | \
	 (CONFIG_OCLK_DIV   << OSEL_P))
#endif

__attribute__((always_inline)) static inline void
program_early_devices(ADI_BOOT_DATA *bs, uint *sdivB, uint *divB, uint *vcoB)
{
	serial_putc('a');

	/* Save the clock pieces that are used in baud rate calculation */
	if (BFIN_DEBUG_EARLY_SERIAL || CONFIG_BFIN_BOOT_MODE == BFIN_BOOT_UART) {
		serial_putc('b');

		*sdivB = bfin_read_CGU_DIV();
		*sdivB = ((*sdivB >> 8) & 0x1f) * ((*sdivB >> 5) & 0x7);
		*vcoB = (bfin_read_CGU_CTL() >> 8) & 0x7f;

		*divB = serial_early_get_div();
		serial_putc('c');
	}

	serial_putc('d');

#ifdef CONFIG_HW_WATCHDOG
# ifndef CONFIG_HW_WATCHDOG_TIMEOUT_INITCODE
#  define CONFIG_HW_WATCHDOG_TIMEOUT_INITCODE 20000
# endif
	/* Program the watchdog with an initial timeout of ~20 seconds.
	 * Hopefully that should be long enough to load the u-boot LDR
	 * (from wherever) and then the common u-boot code can take over.
	 * In bypass mode, the start.S would have already set a much lower
	 * timeout, so don't clobber that.
	 */
	if (CONFIG_BFIN_BOOT_MODE != BFIN_BOOT_BYPASS) {
		serial_putc('e');

		/* Reset system event controller */
		bfin_write_SEC_GCTL(0x2);
		bfin_write_SEC_CCTL(0x2);
		SSYNC();

		/* Enable fault event output and system reset action in fault
		 * controller. Route watchdog timeout event to fault interface.
		 */
		bfin_write_SEC_FCTL(0x51);
		/* Enable watchdog interrupt source */
		bfin_write_SEC_SCTL(3, bfin_read_SEC_SCTL(3) | 0x6);
		SSYNC();

		/* Enable system event controller */
		bfin_write_SEC_GCTL(0x1);
		bfin_write_SEC_CCTL(0x1);
		SSYNC();

		bfin_write_WDOG_CTL(WDDIS);
		SSYNC();
		bfin_write_WDOG_CNT(MSEC_TO_SCLK(CONFIG_HW_WATCHDOG_TIMEOUT_INITCODE));
#if CONFIG_BFIN_BOOT_MODE != BFIN_BOOT_UART
		bfin_write_WDOG_CTL(WDEN);
#endif
		serial_putc('f');
	}
#endif

	serial_putc('g');

	/* Blackfin bootroms use the SPI slow read opcode instead of the SPI
	 * fast read, so we need to slow down the SPI clock a lot more during
	 * boot.  Once we switch over to u-boot's SPI flash driver, we'll
	 * increase the speed appropriately.
	 */
#ifdef SPI_BAUD
	if (CONFIG_BFIN_BOOT_MODE == BFIN_BOOT_SPI_MASTER) {
		serial_putc('h');
		if (BOOTROM_SUPPORTS_SPI_FAST_READ && CONFIG_SPI_BAUD_INITBLOCK < 4)
			bs->dFlags |= BFLAG_FASTREAD;
		bfin_write_SPI_BAUD(CONFIG_SPI_BAUD_INITBLOCK);
		serial_putc('i');
	}
#endif

	serial_putc('j');
}

__attribute__((always_inline)) static inline bool
maybe_self_refresh(ADI_BOOT_DATA *bs)
{
	serial_putc('a');

	if (!CONFIG_MEM_SIZE)
		return false;


	/* resume from hibernate, return false let ddr initialize */
	if ((bfin_read32(DPM0_STAT) & 0xF0) == 0x50) {
		serial_putc('b');
		return false;
	}

	serial_putc('c');

	return false;
}

__attribute__((always_inline)) static inline u16
program_clocks(ADI_BOOT_DATA *bs, bool put_into_srfs)
{
	serial_putc('a');

	if (bfin_read_DMC0_STAT() & MEMINITDONE) {
		bfin_write_DMC0_CTL(bfin_read_DMC0_CTL() | SRREQ);
		__builtin_bfin_ssync();
		while (!(bfin_read_DMC0_STAT() & SRACK))
			continue;
	}

	/* Don't set the same value of MSEL and DF to CGU_CTL */
	if ((bfin_read_CGU_CTL() & (MSEL_MASK | DF_MASK))
			!= CONFIG_CGU_CTL_VAL) {
		bfin_write_CGU_DIV(CONFIG_CGU_DIV_VAL);
		bfin_write_CGU_CTL(CONFIG_CGU_CTL_VAL);
		while ((bfin_read_CGU_STAT() & (CLKSALGN | PLLBP)) ||
				!(bfin_read_CGU_STAT() & PLLLK))
			continue;
	}

	bfin_write_CGU_DIV(CONFIG_CGU_DIV_VAL | UPDT);
	while (bfin_read_CGU_STAT() & CLKSALGN)
		continue;

	if (bfin_read_DMC0_STAT() & MEMINITDONE) {
		bfin_write_DMC0_CTL(bfin_read_DMC0_CTL() & ~SRREQ);
		__builtin_bfin_ssync();
		while (bfin_read_DMC0_STAT() & SRACK)
			continue;
	}

	serial_putc('o');

	return 0;
}

__attribute__((always_inline)) static inline void
update_serial_clocks(ADI_BOOT_DATA *bs, uint sdivB, uint divB, uint vcoB)
{
	/* Since we've changed the SCLK above, we may need to update
	 * the UART divisors (UART baud rates are based on SCLK).
	 * Do the division by hand as there are no native instructions
	 * for dividing which means we'd generate a libgcc reference.
	 */
	unsigned int sdivR, vcoR;
	unsigned int dividend;
	unsigned int divisor;
	unsigned int quotient;

	serial_putc('a');

	if (BFIN_DEBUG_EARLY_SERIAL ||
		CONFIG_BFIN_BOOT_MODE == BFIN_BOOT_UART) {

	sdivR = bfin_read_CGU_DIV();
	sdivR = ((sdivR >> 8) & 0x1f) * ((sdivR >> 5) & 0x7);
	vcoR = (bfin_read_CGU_CTL() >> 8) & 0x7f;

	dividend = sdivB * divB * vcoR;
	divisor = vcoB * sdivR;
	quotient = early_division(dividend, divisor);
	serial_early_put_div(quotient - ANOMALY_05000230);
	}

	serial_putc('c');
}

__attribute__((always_inline)) static inline void
program_memory_controller(ADI_BOOT_DATA *bs, bool put_into_srfs)
{

	int i=0 ;

	bfin_write_DMC0_PHY_CTL4(ENUM_DMC_PHY_CTL4_DDR2);

	/* Program the PAD RTT and driver impedance values required here */
	bfin_write_DMC0_CAL_PADCTL0(DMC0_PADCTL0_VALUE);
	bfin_write_DMC0_CAL_PADCTL2(DMC0_PADCTL2_VALUE);

	/* Start calibration */
	bfin_write_DMC0_CAL_PADCTL0( bfin_read_DMC0_CAL_PADCTL0() | BITM_DMC_CAL_PADCTL0_CALSTRT );
	__builtin_bfin_ssync();

	/* Wait for PAD calibration to complete - 300 DCLK cycle. */
	for(i=0; i<(300); i++)
	{
		asm("nop;");
	}

	/* Wait for DLL lock. Wait for atleast 4500 DCLK cycles. */
	for(i=0; i<(4500); i++)
	{
		asm("nop;");
	}	

	/* Configure the DMC registers */
	bfin_write_DMC0_CFG( 0x00000522 );
	bfin_write_DMC0_TR0( 0x20b08323 );
	bfin_write_DMC0_TR1( 0x20270618 );
	bfin_write_DMC0_TR2( 0x00323209 );
	bfin_write_DMC0_MR(  0x00000432 );
	bfin_write_DMC0_EMR1( 0x00000000 );
	bfin_write_DMC0_EMR2( 0x00000000 );

	/* Configure the DMC_CTL register at the end including the INIT bit */
	bfin_write_DMC0_CTL( 0x00002404 );
	__builtin_bfin_ssync();

	/* Wait till INITDONE is not set */
	while((bfin_read_DMC0_STAT() & BITM_DMC_STAT_INITDONE)==0);

	/* Program the DLLCTL register now */
	bfin_write_DMC0_DLLCTL( 0x0000054b );
	__builtin_bfin_ssync();

	serial_putc('e');

}

__attribute__((always_inline)) static inline void
check_hibernation(ADI_BOOT_DATA *bs, u16 vr_ctl, bool put_into_srfs)
{
	serial_putc('a');

	if (!CONFIG_MEM_SIZE)
		return;

	serial_putc('b');

	if (bfin_read32(DPM0_RESTORE0) != 0) {
		uint32_t reg = bfin_read_DMC0_CTL();
		reg &= ~0x8;
		bfin_write_DMC0_CTL(reg);

		while ((bfin_read_DMC0_STAT() & 0x8))
			continue;
		while (!(bfin_read_DMC0_STAT() & 0x1))
			continue;

		serial_putc('z');
		volatile uint32_t *hibernate_magic = (volatile uint32_t *)bfin_read32(DPM0_RESTORE4);
		__builtin_bfin_ssync(); /* make sure memory controller is done */
		if (hibernate_magic[0] == 0xDEADBEEF) {
			serial_putc('c');

			__builtin_bfin_ssync();

			bfin_write_EVT15(hibernate_magic[1]);
			bfin_write_IMASK(EVT_IVG15);
			__asm__ __volatile__ (
				/* load reti early to avoid anomaly 281 */
				"reti = %2;"
				/* clear hibernate magic */
				"[%0] = %1;"
				/* load stack pointer */
				"SP = [%0 + 8];"
				/* lower ourselves from reset ivg to ivg15 */
				"raise 15;"
				"nop;nop;nop;"
				"rti;"
				:
				: "p"(hibernate_magic),
				"d"(0x2000 /* jump.s 0 */),
				"d"(0x11a00000)
			);
		}


	}

	serial_putc('e');
}

BOOTROM_CALLED_FUNC_ATTR
void initcode(ADI_BOOT_DATA *bs)
{
	ADI_BOOT_DATA bootstruct_scratch;

	/* Setup NMI handler before anything else */
	program_nmi_handler();

	serial_init();

	serial_putc('A');

	/* If the bootstruct is NULL, then it's because we're loading
	 * dynamically and not via LDR (bootrom).  So set the struct to
	 * some scratch space.
	 */
	if (!bs)
		bs = &bootstruct_scratch;

	serial_putc('B');
	bool put_into_srfs = maybe_self_refresh(bs);

	serial_putc('C');
	uint sdivB, divB, vcoB;
	program_early_devices(bs, &sdivB, &divB, &vcoB);

	serial_putc('D');
	u16 vr_ctl = program_clocks(bs, put_into_srfs);

	serial_putc('E');
	update_serial_clocks(bs, sdivB, divB, vcoB);

	serial_putc('F');
	program_memory_controller(bs, put_into_srfs);

	serial_putc('G');
	check_hibernation(bs, vr_ctl, put_into_srfs);

	serial_putc('H');
	program_async_controller(bs);

#ifdef CONFIG_BFIN_BOOTROM_USES_EVT1
	serial_putc('I');
	/* Tell the bootrom where our entry point is so that it knows
	 * where to jump to when finishing processing the LDR.  This
	 * allows us to avoid small jump blocks in the LDR, and also
	 * works around anomaly 05000389 (init address in external
	 * memory causes bootrom to trigger external addressing IVHW).
	 */
	if (CONFIG_BFIN_BOOT_MODE != BFIN_BOOT_BYPASS)
		bfin_write_EVT1(CONFIG_SYS_MONITOR_BASE);
#endif

	serial_putc('>');
	serial_putc('\n');

	serial_deinit();
}
