/*
 * FirstTry.C
 *
 *  Created on: Jan 16, 2017
 *      Author: jaa
 */
/*
 * This program serves multiple purposes. This is a learning project
 * for STM32F103C8T6 'Blue Pill', but it also documents the way
 * different stuff is set up. That's why it doesn't use any CMSIS-stuff.
 */

#include <stdint.h>

/* debug flags */
//#define DEBUG_SYSTICK
//#define DEBUG_TIM1_CLK
//#define DEBUG_USART1_ECHO
//#define DEBUG_RTC_CLK
//#define DEBUG_RTC_INIT
#define APPLICATION
#define STANDBY_ENABLED
#define RTC_WAKEUP
//#define WKUP_WAKEUP
#define VOLTAGE_ALARM
//#define ADC_USED
#define TEMP_SENSOR

/* use xtal or internal clock for RTC */
#define RTC_USE_XTAL

//#define USART1_BAUD_115200
#define USART1_BAUD_19200
//#define USART1_BAUD_9600

// use TXE as transmit interrupt instead of TC
#define DEBUG_UART1_TXE

// Voltage alarm threshold 2,5V
#define LOW_VOLTAGE_THR (3 << 5)

// 32767 -> 1 Hz
//#define RTC_PRESCALER_XTAL 0x00007fff
// 32767 -> 1 Hz (+ correction -0.48% -> 32610)
#define RTC_PRESCALER_XTAL 0x00007F62

/* FCK_CNT = fCK_PSC / (PSC[15:0] + 1).
   72 MHz / 0x1C1F = 10 000 Hz = 0.1 ms per tick */
#define TIM1_PRESCALER 0x1C1F

#define PERIPH_BASE           0x40000000

/* Buses */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)
#define PRIVPERIPH_BASE       0xe0000000

/* Peripherals */
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define BKP_BASE              (APB1PERIPH_BASE + 0x6C00)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)
#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)
#define PA_BASE               (APB2PERIPH_BASE + 0x0800)
#define PB_BASE               (APB2PERIPH_BASE + 0x0c00)
#define PC_BASE               (APB2PERIPH_BASE + 0x1000)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2400)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2800)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
#define USART1_BASE           (APB2PERIPH_BASE + 0x3800)
#define RCC_BASE              (AHBPERIPH_BASE + 0x1000)
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x2000)
#define CRC_BASE              (AHBPERIPH_BASE + 0x3000)

#define SCB_BASE              (PRIVPERIPH_BASE + 0xed00)
#define SYSTIC_BASE           (PRIVPERIPH_BASE + 0xe010)
#define NVIC_BASE             (PRIVPERIPH_BASE + 0xe100)

/* Registers */
#define RCC_CR 0
#define RCC_CFGR 1
#define RCC_CIR 2
#define RCC_APB2RSTR 3
#define RCC_APB1RSTR 4
#define RCC_AHBENR 5
#define RCC_APB2ENR 6
#define RCC_APB1ENR 7
#define RCC_BDCR 8
#define RCC_CSR 9
#define GPIO_CRL 0
#define GPIO_CRH 1
#define GPIO_IDR 2
#define GPIO_ODR 3
#define GPIO_BSRR 4
#define GPIO_BRR 5
#define GPIO_LCKR 6
#define PC_13
#define FLASH_ACR 0
#define PWR_CR 0
#define PWR_CSR 1
#define RTC_CRH 0
#define RTC_CRL 1
#define RTC_PRLH 2
#define RTC_PRLL 3
#define RTC_DIVH 4
#define RTC_DIVL 5
#define RTC_CNTH 6
#define RTC_CNTL 7
#define RTC_ALRH 8
#define RTC_ALRL 9
#define BKP_DATA1 1 /* 1 - 42 */
#define BKP_DATA2 2 /* 1 - 42 */
#define USART_SR 0
#define USART_DR 1
#define USART_BRR 2
#define USART_CR1 3
#define USART_CR2 4
#define USART_CR3 5
#define USART_GTPR 6
#define TIM1_CR1 0
#define TIM1_CR2 1
#define TIM1_SMCR 2
#define TIM1_DIER 3
#define TIM1_SR 4
#define TIM1_EGR 5
#define TIM1_CNT 9
#define TIM1_PSC 10
#define TIM1_ARR 11
#define EXTI_IMR 0
#define EXTI_EMR 1
#define EXTI_RTSR 2
#define EXTI_FTSR 3
#define EXTI_SWIER 4
#define EXTI_PR 5
#define ADC_SR 0
#define ADC_CR1 1
#define ADC_CR2 2
#define ADC_SMPR1 3
#define ADC_SMPR2 4
#define ADC_JOFR1 5
#define ADC_JOFR2 6
#define ADC_JOFR3 7
#define ADC_JOFR4 8
#define ADC_HTR 9
#define ADC_LTR 10
#define ADC_SQR1 11
#define ADC_SQR2 12
#define ADC_SQR3 13
#define ADC_JSQR 14
#define ADC_JDR1 15
#define ADC_JDR2 16
#define ADC_JDR3 17
#define ADC_JDR4 18
#define ADC_DR 19

#define SYST_CSR 0
#define SYST_RVR 1
#define SYST_CVR 2
#define SYST_CAL 3
/* ISER 0: IRQs 0 - 31, ISER 1: IRQs 32 - 63, ISER 2: IRQs 64 - 67 */
/* 1 bit per IRQ, IRQ0 is the lsb of ISER 0 */
#define NVIC_ISER_0 0
#define NVIC_ISER_1 1
#define NVIC_ISER_2 2
#define NVIC_ICER_0 32
#define NVIC_ICER_1 33
#define NVIC_ICER_2 34
#define NVIC_ISPR_0 64
#define NVIC_ISPR_1 65
#define NVIC_ISPR_2 66
#define NVIC_ICPR_0 96
#define NVIC_ICPR_1 97
#define NVIC_ICPR_2 98
#define NVIC_IABR_0 128
#define NVIC_IABR_1 129
#define NVIC_IABR_2 130
#define NVIC_IPR_0 192
#define NVIC_STIR 896
#define SCB_CPUID 0
#define SCB_ICSR 1
#define SCB_VTOR 2
#define SCB_AIRCR 3
#define SCB_SCR 4
#define SCB_CCR 5

/* When 72 MHz clock */
#define TICS_PER_MS 0x1193F
#define MS_PER_HOUR 3600000
#define USART1_RXLEN 64
#define USART1_TXLEN 64

volatile uint32_t wup_flags;
volatile uint32_t bkp_data;
volatile uint32_t ticks;
volatile uint8_t power_low;

volatile uint32_t rx_head, rx_tail;
volatile uint32_t tx_head, tx_tail;
volatile uint8_t rx_buff[USART1_RXLEN];
volatile uint8_t tx_buff[USART1_TXLEN];
volatile uint32_t rx_dropped;
volatile uint32_t tx_dropped;
volatile uint32_t ore;
volatile uint32_t ne;
volatile uint32_t fe;

volatile uint8_t rtc_sec;
volatile uint8_t rtc_min;
volatile uint8_t rtc_hour;
volatile uint8_t rtc_day;

volatile uint32_t adc_values[3];

volatile uint32_t dbg1, dbg2, dbg3;

/* Code */

void clkint(void)
{
	volatile uint32_t *syst;
	syst = (uint32_t *) SYSTIC_BASE;

	/* The flag resets when read? */
	if (syst[SYST_CSR] & 0x00010000) /* COUNTFLAG */
	{
		ticks++;
		if (ticks >= MS_PER_HOUR) /* hour */
		{
			ticks -= MS_PER_HOUR;
		}
	}
}

void RCC_reset(void)
{
	volatile uint32_t *rcc;
	rcc = (uint32_t *) RCC_BASE;

	/* Set HSION bit - ensure HSI is running */
	rcc[RCC_CR] |= (uint32_t)0x00000001;

	/* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
	/* clear pre-scalers etc., don't touch HSE-parameters,
	   select HSI (not PLL) */
	/* Note: both HSE and HSI are 8MHz */
	rcc[RCC_CFGR] &= (uint32_t)0xF8FF0000;

	/* Reset HSEON, CSSON and PLLON bits */
	/* That is: turn off HSE, clock security and PLL */
	rcc[RCC_CR] &= (uint32_t)0xFEF6FFFF;

	/* Reset HSEBYP bit */
	rcc[RCC_CR] &= (uint32_t)0xFFFBFFFF;

	/* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE bits */
	/* Clear USB and HSE pre-scalers, switch PLL in to HSI,
	   clear PLL multiplier */
	rcc[RCC_CFGR] &= (uint32_t)0xFF80FFFF;

	/* Disable all interrupts and clear pending bits  */
	rcc[RCC_CIR] = 0x009F0000; // CIR
}

void RCC_setup_clocks(void)
{
	int tmp;

	volatile uint32_t *rcc, *flash, *pwr, *syst;
	rcc = (uint32_t *) RCC_BASE;
	flash = (uint32_t *) FLASH_R_BASE;
	pwr = (uint32_t *) PWR_BASE;
	syst = (uint32_t *) SYSTIC_BASE;

	/* Turn HSE ON */
	rcc[RCC_CR] |= (uint32_t) 0x00010000;

	/* Wait for HSE to stabilize (poll ready-flag) */
	tmp = 1000;
	while (!(rcc[RCC_CR] & (uint32_t) 0x00020000))
	{
		tmp--;
		if (tmp == 0)
		{
			goto pedro;
		}
	}

    /* Enable Flash Prefetch Buffer - must be done while sysclk < 24 MHz and
    AHB has no prescaler (sysclk = hclk) - Must be ON if there is a prescaler */
    /* Note: FLASH and SRAM are enabled by default */
	flash[FLASH_ACR] |= ((uint32_t)0x00000010); // enable prefetch buffer
    /* Flash 2 wait state */
	flash[FLASH_ACR] = (flash[FLASH_ACR] & ((uint32_t)0x00000038))
		| ((uint32_t)0x00000002);

    /* HPRE = 0 -> HCLK = SYSCLK */
    rcc[RCC_CFGR] &= (uint32_t) 0xffffff0f;

    /* PPRE2 = 0 -> PCLK2 = HCLK (APB2, high-speed) */
    rcc[RCC_CFGR] &= (uint32_t) 0xffffc7ff;

    /* PPRE1 = 4 -> PCLK1 = HCLK/2 (APB1, low-speed) */
    rcc[RCC_CFGR] = (rcc[RCC_CFGR] & (uint32_t) 0xfffff8ff)
		| ((uint32_t) 0x00000400);

	/* USBPRE = 0 -> USB pre-scaler = 1.5 */
	rcc[RCC_CFGR] &= (uint32_t) 0xffbfffff;

	/* ADC pre-scaler = 2 -> ADC clock = PCLK2/6 (must not exceed 14 MHz) */
	/* Gives 12 MHz when PCLK2 = HCLK = 72 MHz */
	rcc[RCC_CFGR] = (rcc[RCC_CFGR] & (uint32_t) 0xffff3fff)
		| ((uint32_t) 0x00008000);

	/* APB1-peripherals */
	/* Enable PWR, BKP and SPI2, disable other peripherals */
	rcc[RCC_APB1ENR] = (rcc[RCC_APB1ENR] & (uint32_t) 0xc5013600)
		| ((uint32_t) 0x18004000);

	/* APB2-peripherals */
	/* Enable ADC1, IOPC, disable other peripherals */
	rcc[RCC_APB2ENR] = (rcc[RCC_APB2ENR] & (uint32_t) 0xffc70002)
		| ((uint32_t) 0x00000210);

	/* Enable Backup domain access */
	pwr[PWR_CR] =(pwr[PWR_CR] & (uint32_t) 0xfffffe00)
		| ((uint32_t) 0x00000100); 	/* Set DBP-bit */

    /* PLLCLK = 8MHz * 9 = 72 MHz -> PLL mult = 7 */
    /* PLL source = HSE ->  PLLSRC = 1 */
	rcc[RCC_CFGR] = (rcc[RCC_CFGR] & (uint32_t) 0xffc2ffff)
		| ((uint32_t) 0x001d0000);

    /* Enable PLL */
    rcc[RCC_CR] |= 0x01000000;

    /* Wait until PLL is ready */
	tmp = 1000;
	while (!(rcc[RCC_CR] & (uint32_t) 0x02000000))
	{
		tmp--;
		if (tmp == 0)
		{
			goto pedro;
		}
	}

    /* Select PLL as system clock source -> SW = 2 */
	rcc[RCC_CFGR] = (rcc[RCC_CFGR] & (uint32_t) 0xfffffffc)
		| ((uint32_t) 0x00000002);

    /* Wait until PLL is used as system clock source -> SWS = 2 */
	tmp = 1000;
    while ((rcc[RCC_CFGR] & (uint32_t) 0x0000000c) != (uint32_t) 0x00000008)
	{
		tmp--;
		if (tmp == 0)
		{
			goto pedro;
		}
	}

	/* Init SYSTICK as millisecond counter */
	syst[SYST_RVR] = TICS_PER_MS;
#if 1
	/* Processor clock select + interrupt + enable */
	syst[SYST_CSR] = (syst[SYST_CSR] & (uint32_t) 0xfffefff8)
		| ((uint32_t) 0x00000007);
#else
	/* Processor clock select + enable  - just for debug */
	syst[SYST_CSR] = (syst[SYST_CSR] & (uint32_t) 0xfffefff8)
		| ((uint32_t) 0x00000005);
#endif

	ticks = 0;

	return;

/* "My name is Pedro and I kill for money, but you are my friend
   I kill you for nothing." */
pedro:
	tmp = 0;
	while(1)
	{
		tmp++; /* place for breakpoint */
		if (tmp > 10000) tmp = 0;
	}
}

/* ***** RTC ***** */
void inir_rtc(void)
{
	int tmp;
	volatile uint32_t *rcc, *pwr, *bkp, *rtc, *scb, *nvic;
	rcc = (uint32_t *) RCC_BASE;
	pwr = (uint32_t *) PWR_BASE;
	bkp = (uint32_t *) BKP_BASE;
	rtc = (uint32_t *) RTC_BASE;
	scb = (uint32_t *) SCB_BASE;
	nvic = (uint32_t *) NVIC_BASE;

	/* if bkp domain is not reset */
	bkp_data = bkp[BKP_DATA1] & 0x0000ffff;
	if ((bkp[BKP_DATA1] & 0x0000ffff) == 0x0000a5a5)
	{
#ifdef STANDBY_ENABLED
		wup_flags = pwr[PWR_CSR]; // store globally
		if (wup_flags & 0x00000003) // wakeup?
		{
			rtc[RTC_CRH] &= ~0x00000002; // disable alarm interrupt
			nvic[NVIC_ICER_1] |= (1 << 9); // disable interrupt from NVIC
			nvic[NVIC_ICPR_1] |= (1 << 9); // clear pending from NVIC

			/* clear wake up flags: SBF and WUP */
			pwr[PWR_CR] &= ~0x0000000c;

			/* SCB_SCR should be zero after reset */
			/* no need to clear it */
			// scb[SCB_SCR] = 0x00000000;

			/* SEVONPEND = 0, SLEEPONEXIT = 0, DEEPSLEEP = 0 */
			scb[SCB_SCR] &= 0x00000000;

			// TODO: just a place
			/* clear OWF, ALRF, SECF */
			rtc[RTC_CRL] &= 0xfffffff8;

			/* wait for synchronization (RSF)*/
			rtc[RTC_CRL] &=  ~0x00000008; /* mark not synced */
			while (!(rtc[RTC_CRL] & 0x00000008)); /* wait for sync */

#if 0
			/* if some actions are needed depending on
			   whether waken up by RTC alarm or WKUP-pin */
			if (wup_flags & 0x00000002) // SBF
			{
				pwr[PWR_CR] &= ~0x00000008; // CSBF
			}

			if (wup_flags & 0x00000001) // WKUP
			{
				pwr[PWR_CR] &= ~0x00000004; // CWUP
			}
#endif
		}
#endif
		return;
	}

    /* ***** Only after bkp domain reset(?) ***** */

    /* Write '1' to BDRST-bit in RCC_BDCR to reset Backup domain */
    rcc[RCC_BDCR] |= ((uint32_t) 0x00010000);

	/* Enable Backup domain access after bkp-reset
		for access to most RCC_BDCR-bits */
	pwr[PWR_CR] =(pwr[PWR_CR] & (uint32_t) 0xfffffe00)
		| ((uint32_t) 0x00000100); 	/* Set DBP-bit */

	/* Enable PWR, BKP and SPI2, disable other peripherals */
	rcc[RCC_APB1ENR] = (rcc[RCC_APB1ENR] & (uint32_t) 0xc5013600)
		| ((uint32_t) 0x18004000);

#ifdef RTC_USE_XTAL
	/* Set LSE ON */
    rcc[RCC_BDCR] = (rcc[RCC_BDCR] & (uint32_t) 0xfffe7ff8)
		| ((uint32_t) 0x00000001);

    /* Wait until RTC external clock is stabilized -> LSERDY = 1 */
    tmp = 5000000;
    while ((rcc[RCC_BDCR] & (uint32_t) 0x00000002) != (uint32_t) 0x00000002)
    {
    	tmp--;
    	if (!tmp)
    		while(1);
    }

    /* RTC clock source selection + RTC clock enable */
    rcc[RCC_BDCR] = (rcc[RCC_BDCR] & (uint32_t) 0xfffe7cf8)
		| ((uint32_t) 0x00008101);

    /* wait for synchronization (RSF) */
    rtc[RTC_CRL] &= (uint32_t) 0xfffffff7; /* mark not synced */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000008));
#else
	/* Set LSE OFF */
    rcc[RCC_BDCR] = (rcc[RCC_BDCR] & (uint32_t) 0xfffe7cf8);

    /* Set LSI ON */
    rcc[RCC_CSR] |= 0x00000001; /* LSION */
    /* Wait until RTC internal clock is stabilized -> LSIRDY = 1 */

    tmp = 5000000;
    while ((rcc[RCC_CSR] & (uint32_t) 0x00000002) != (uint32_t) 0x00000002)
    {
    	tmp--;
    	if (!tmp)
    		while(1);
    }

    /* RTC clock source selection + RTC clock enable */
    rcc[RCC_BDCR] = (rcc[RCC_BDCR] & (uint32_t) 0xfffe7cf8)
		| ((uint32_t) 0x00008200);
	/* wait for synchronization (RSF) (can last a minute) */
    rtc[RTC_CRL] &= (uint32_t) 0xfffffff7; /* mark not synced */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000008));
#endif

	/* wait for last write to finish (RTOFF) */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000020));

	rtc[RTC_CRL] = 0x00000010; /* enter CNF */
    rtc[RTC_CNTH] = 0x00000000; /* Count high */
    rtc[RTC_CNTL] = 0x00000000; /* Count low */
#ifdef RTC_USE_XTAL
    rtc[RTC_PRLH] = 0x00000000; /* Prescaler hi */
    rtc[RTC_PRLL] = RTC_PRESCALER_XTAL; /* Prescaler lo */
#else
    /* Internal RC - 40 kHz typical (30 - 60) */
    rtc[RTC_PRLH] = 0x00000000; /* Prescaler hi */
    rtc[RTC_PRLL] = 0x00009C3F; /* Prescaler lo = 40 000 -> 1 Hz*/
#endif
    rtc[RTC_CRL] = 0x00000000; /* exit CNF */
	/* wait for last write to finish (RTOFF) */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000020));
	/* wait for synchronization (RSF) (can last a minute) */
    rtc[RTC_CRL] &= (uint32_t) 0xfffffff7; /* mark not synced */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000008));
	/* Mark RTC initialized */
	bkp[BKP_DATA1] = (bkp[BKP_DATA1] & 0xffff0000) | 0x0000a5a5;
	/* Mark no power low */
	bkp[BKP_DATA2] = (bkp[BKP_DATA2] & 0xffff0000) | 0x00000000;
}

/* read RTC seconds and set the global variables */
void rtc_get_time(void)
{
	uint32_t rtc_time;
	volatile uint32_t *rtc;
	rtc = (uint32_t *) RTC_BASE;

	rtc_time = (rtc[RTC_CNTH] << 16) & 0xffff0000;
	rtc_time |= (rtc[RTC_CNTL] & 0x0000ffff);

	rtc_sec = rtc_time % 60;
	rtc_time /= 60;
	rtc_min = rtc_time % 60;
	rtc_time /= 60;
	rtc_hour = rtc_time % 24;
	rtc_day = rtc_time / 24;
}

#ifdef STANDBY_ENABLED
void go_standby(void)
{
	volatile uint32_t *rcc, *pwr, *bkp, *rtc, *scb, *syst, *nvic;
	// rcc = (uint32_t *) RCC_BASE;
	pwr = (uint32_t *) PWR_BASE;
	bkp = (uint32_t *) BKP_BASE;
	rtc = (uint32_t *) RTC_BASE;
	scb = (uint32_t *) SCB_BASE;
	syst = (uint32_t *) SYSTIC_BASE;
	nvic = (uint32_t *) NVIC_BASE;

	/* TODO: save data over standy */
	// bkp[BKP_DATA2] = ... // store data

	/* stop systick interrupts */
	syst[SYST_CSR] &= ~0x00000002;

	/* clear RSF, OWF, ALRF, SECF */
	rtc[RTC_CRL] &= 0xfffffff0;
	/* wait until RTC second event flag */
	while (rtc[RTC_CRL] & 0x00000001);
	/* clear RSF, OWF, ALRF, SECF */
	rtc[RTC_CRL] &= 0xfffffff0;

#ifdef RTC_WAKEUP
	/* wait for last write to finish (RTOFF) */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000020));
	rtc[RTC_CRL] = 0x00000010; /* enter CNF */
	/* sleeptime */
	rtc[RTC_ALRH] = rtc[RTC_CNTH]; // high 16 bits
	rtc[RTC_ALRL] = rtc[RTC_CNTL] + 0x0000001e; // 30 sec, low 16 bits
	rtc[RTC_CRL] = 0x00000000; /* exit CNF */

	/* wait for synchronization (RSF) (can last a minute) */
    rtc[RTC_CRL] &= (uint32_t) 0xfffffff7; /* mark not synced */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000008));

	/* wait for last write to finish (RTOFF) */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000020));

	//rtc[RTC_CRH] |= 0x00000002; // enable alarm interrupt - must NOT do
#endif

#ifdef WKUP_WAKEUP
	/* enable wakeup by WKUP-pin */
	pwr[PWR_CSR] |= 0x00000100; // enable WKUP-pin
#endif

	/* clear all pending interrupts */
	nvic[NVIC_ICPR_0] = 0xffffffff;
	nvic[NVIC_ICPR_1] = 0xffffffff;
	nvic[NVIC_ICPR_2] = 0xffffffff;

	/* SEVONPEND = 0, SLEEPONEXIT = 0, DEEPSLEEP = 1 */
	scb[SCB_SCR] = 0x00000004;
	/* clear CSBF and CWUF, PDDS = standby */
	pwr[PWR_CR] = (pwr[PWR_CR] & 0xfffffff0) | 0x0000000e;

#if 1
	// dsb is probably needed before wfi
	asm volatile (
		"dsb\n\t"
		"wfi\n\t"
	);
#else
	// check that timer alarm works before trying actual sleep
	while (!(rtc[RTC_CRL] & 0x00000002)); // wait for RTC alarm
	/* start systick interrupts */
	syst[SYST_CSR] |= 0x00000002;
#endif
}
#endif

void init_usart1(void)
{
	/* exception vector: 0x000000d4 */
	/* USART1 IRQ = IRQ37 */
	volatile uint32_t *rcc, *gpioa, *usart1, *nvic;
	rcc = (uint32_t *) RCC_BASE;
	gpioa = (uint32_t *) PA_BASE;
	usart1 = (uint32_t *) USART1_BASE;
	nvic = (uint32_t *) NVIC_BASE;

	/* init buffers */
	rx_head = rx_tail = 0;
	tx_head = tx_tail = 0;

	rx_dropped = 0;
	tx_dropped = 0;
	ore = 0; // overruns
	ne = 0; // noise errors
	fe = 0; // framing errors

	/* Pins A9 = Tx, A10 = Rx */
	if (!(rcc[RCC_APB2ENR] & 0x00000004)) /* if port A doesn't have clock */
	{
		rcc[RCC_APB2ENR] |= 0x00000004;
	}
	/* clear port A bits 10 and 9 - just in case... */
	gpioa[GPIO_BRR] = 0x00000600;

	/* Pin 10 as input/float, pin 9 as alternate push-pull, 50MHz */
    gpioa[GPIO_CRH] = (gpioa[GPIO_CRH] & (uint32_t) 0xfffff00f)
		| ((uint32_t) 0x000004b0);

	/* USART1 ON */
	rcc[RCC_APB2ENR] = (rcc[RCC_APB2ENR] & (uint32_t) 0xffffbfff)
		| ((uint32_t) 0x00004000);

	/* 1 stopbit */
	usart1[USART_CR2] = 0x00000000;
	/* enable, 8 bits, no ints, tx ON, rx ON */
	usart1[USART_CR1] = 0x0000200C;
	/* no handshakes, DMA disabled */
   	usart1[USART_CR3] = 0x00000000;
	/* just to be on the safe side */
   	usart1[USART_GTPR] = 0x00000001;

#ifdef USART1_BAUD_9600
	/* 9600 baud: BRR = 468/12 */
	usart1[USART_BRR] = 0x1d4c; /* 9600 */
#endif
#ifdef USART1_BAUD_19200
	/* 19200 baud: BRR = 0xea/6 */
	usart1[USART_BRR] = 0xea6; /* 19200 */
#endif
#ifdef USART1_BAUD_115200
	/* 115200 baud: BRR = 39/1 */
	usart1[USART_BRR] = 0x271; /* 115200 */
#endif
	/* Enable USART1 global interrupt */
	/* IRQ 37: ISER 1, 6th lowest bit */
	nvic[NVIC_ISER_1] |= (1 << 5);
	/* RXNEIE ON */
	usart1[USART_CR1] |= 0x00000020;
}

/* Send data */
void putch(int ch)
{
	uint32_t newtail;
	volatile uint32_t *usart1;
	usart1 = (uint32_t *) USART1_BASE;
	newtail = (tx_tail + 1) % USART1_TXLEN;
	if (newtail == tx_head)
	{
		tx_dropped++;
		return; // full -> drop
	}
	tx_buff[tx_tail] = (uint8_t)ch;
	tx_tail = newtail;

#ifdef DEBUG_UART1_TXE
	/* if transmitting (TXEIE) is off */
	if (!(usart1[USART_CR1] & 0x00000080))
	{
		/* enable TXEIE */
		/* let tx interrupt start the transmitting */
		usart1[USART_CR1] |= 0x00000080;
	}
#else
	/* if transmitting (TCIE) is off */
	if (!(usart1[USART_CR1] & 0x00000040))
	{
		/* enable TCIE */
		/* let tx interrupt start the transmitting */
		usart1[USART_CR1] |= 0x00000040;
	}
#endif
}

/* Reveive data */
int getch(void)
{
	int ch;
	uint32_t newhead;
	if (rx_head == rx_tail) return -1; // empty
	ch = (int)rx_buff[rx_head];
	newhead = (rx_head + 1) % USART1_RXLEN;
	rx_head = newhead;
	return ch;
}

void put_rxch(int ch)
{
	uint32_t newtail;
	newtail = (rx_tail + 1) % USART1_RXLEN;
	if (newtail == rx_head)
	{
		rx_dropped++;
		return; // full -> drop
	}
	rx_buff[rx_tail] = (uint8_t)ch;
	rx_tail = newtail;
}

int get_txch(void)
{
	int ch;
	uint32_t newhead;
	if (tx_head == tx_tail) return -1; // empty
	ch = (int)tx_buff[tx_head];
	newhead = (tx_head + 1) % USART1_TXLEN;
	tx_head = newhead;
	return ch;
}

/* Note: turning TE on and off causes disturbancies with
   both transmission and receiving */
void usart1_irq()
{
	int ch;
	volatile uint32_t status;
	volatile uint32_t *usart1 = (uint32_t *) USART1_BASE;

	status = usart1[USART_SR];
	if (status & 8) ore++;
	if (status & 4) ne++;
	if (status & 2) fe++;

	usart1[USART_SR] &= 0xfffffff0; // clear error flags

	/* if rx irq */
	//if (usart1[USART_SR] & 0x00000020) // RXNE
	if (status & 0x00000020) // RXNE
	{
		/* receive char */
		ch = (int)(usart1[USART_DR] & 0xff);
		put_rxch(ch);
	}

	/* if tx irq */
#ifdef DEBUG_UART1_TXE
	if (status & 0x00000080) // TXE
	//if (usart1[USART_SR] & 0x00000080) // TXE
#else
	if (status & 0x00000040) // TC
	//if (usart1[USART_SR] & 0x00000040) // TC
#endif
	{
		ch = get_txch();
		if (ch == -1) // empty
		{
#ifdef DEBUG_UART1_TXE
			/* stop transmitting: disable TXEIE */
			usart1[USART_CR1] &= ~0x00000080;
#else
			/* stop transmitting: disable TCIE */
			usart1[USART_CR1] &= ~0x00000040;
#endif
		}
		else // character(s) to send
		{
			/* send char */
			usart1[USART_DR] = (uint32_t)(ch & 0xff);
		}
	}
}

void init_led(void)
{
	volatile uint32_t *rcc, *gpioc;
	rcc = (uint32_t *) RCC_BASE;
	gpioc = (uint32_t *) PC_BASE;
	/* LED on PC13 - active low */
	/* Does port C have clock? */
	if (!(rcc[RCC_APB2ENR] & 0x00000010)) /* if port c doesn't have clock */
	{
		rcc[RCC_APB2ENR] |= 0x00000010;
	}
	/* Pin 13 as open-drain output, max. 2MHz */
    gpioc[GPIO_CRH] = (gpioc[GPIO_CRH] & (uint32_t) 0xff0fffff)
		| ((uint32_t) 0x00600000);
	gpioc[GPIO_BSRR] = 0x00002000; /* set bit 13  - OFF */
}

/* for checking the APB2 clock */
void init_timer1(void)
{
	volatile uint32_t *tim1, *rcc;
	tim1 = (uint32_t *) TIM1_BASE;
	rcc = (uint32_t *) RCC_BASE;

	/* enable timer1 */
	rcc[RCC_APB2ENR] |= ((uint32_t) 0x00000800);

	/* FCK_CNT = fCK_PSC / (PSC[15:0] + 1).
	   TIM1_PRESCALER = 0x1C1F */
	/* Update_event = TIM_CLK/((PSC + 1)*(ARR + 1)*(RCR + 1)) */
	/* Time base configuration */
	tim1[TIM1_PSC] = TIM1_PRESCALER;
	tim1[TIM1_ARR] = 0x2710; /* 10 000 = 1 s */
	tim1[TIM1_CNT] = 0x2710; /* initialize */

	tim1[TIM1_CR1] = (tim1[TIM1_CR1] & 0xfffffc00)
		| 0x0080; // check ARPE, URS and UDIS; CEN is needed
	tim1[TIM1_CR2] &= 0xffff8002; // check MMS
	tim1[TIM1_SMCR]	&= 0xffff0008;
	tim1[TIM1_DIER] &= 0xffff8000;
#if 1
	tim1[TIM1_EGR] = (tim1[TIM1_EGR] & 0xffffff00)
		| 0x0001; // UG?
#else
	tim1[TIM1_EGR] &= 0xffffff00;
#endif

	tim1[TIM1_CR1] |= 0x00000001; /* CEN - counter enable */
}

void init_pwrmon(void)
{
	uint32_t tmp;
	volatile uint32_t *exti, *nvic, *pwr;
	exti = (uint32_t *) EXTI_BASE;
	nvic = (uint32_t *) NVIC_BASE;
	pwr = (uint32_t *) PWR_BASE;

	/* set threshold (PLS) */
	tmp = (pwr[PWR_CR] & 0xffffff0f) | LOW_VOLTAGE_THR;
	tmp |= 0x00000010; // enable (PVDE)
	pwr[PWR_CR] = tmp;
	/* PVD rising edge causes interrupt */
	exti[EXTI_RTSR] |= 0x00010000; // PVD output
	/* enable EXTI 16 interrupt */
	exti[EXTI_IMR] |= 0x00010000; // PVD output
	/* IRQ 1 = EXTI 16 = PVD */
	nvic[NVIC_ISER_0] |= 0x00000002; // enable IRQ 1
}

void pwr_irq(void)
{
	power_low = 1;
}

#ifdef ADC_USED
#define USE_ADC
#else
#ifdef TEMP_SENSOR
#define USE_ADC
#endif
#endif

#ifdef USE_ADC
/* PB1 = ADC1_IN9, PB0 = ADC1_IN8 */
void init_adc(void)
{
	int wloops;
	volatile uint32_t *adc1, *rcc, *gpiob;
	adc1 = (uint32_t *) ADC1_BASE;
	rcc = (uint32_t *) RCC_BASE;
	gpiob = (uint32_t *) PB_BASE;

	/* PB1 and PB0 for ADC */
	/* Does port B have clock? */
	if (!(rcc[RCC_APB2ENR] & 0x00000008)) /* if port B doesn't have clock */
	{
		rcc[RCC_APB2ENR] |= 0x00000008;
	}
	/* Pins 1 and 0 as analog inputs */
    gpiob[GPIO_CRL] &= (uint32_t) 0xffffff00;
	gpiob[GPIO_BSRR] = 0x00000003; /* set bits 0 and 1 - OFF */

	/* DISCEN? */
	// adc1[ADC_CR1] = (adc1[ADC_CR1] & 0xff300000) | 0x00000800;
	adc1[ADC_CR1] = (adc1[ADC_CR1] & 0xff300000) | 0x00000000;
	adc1[ADC_SR] &= 0xffffffe0; // clear flags including EOC
	/* TEMP, SWSTART, ADON (wake up)*/
	adc1[ADC_CR2] = (adc1[ADC_CR2] & 0xff0106f0) | 0x008e0001;
	/* wait at least 2 ADC cycles (72MHz/12MHz = 6) */
	wloops = 12;
	while (wloops--);
	/* IN16 = temp, IN9 and IN8 = external sensors */
	/* conversion time ch16&17: 17.1us ADC clk = 12 MHz => 205.2 cycles */
	adc1[ADC_SMPR1] = (adc1[ADC_SMPR1] & 0xff000000) | 0x00fc0000;
	/* the external sensors - use max time for them too */
	adc1[ADC_SMPR2] = (adc1[ADC_SMPR2] & 0xc0000000) | 0x3f000000;
	/* calibrate */
	adc1[ADC_CR2] = (adc1[ADC_CR2] & 0xfffffff1) | 0x008e000c;
	while (adc1[ADC_CR2] & 0x00000004); // wait for calibration
	adc1[ADC_SR] &= 0xffffffe0; // clear flags

	/* Only one channel at a time */
	adc1[ADC_SQR1] = (adc1[ADC_SQR1] & 0xff000000) | 0x00100000;
	adc1[ADC_SQR2] = (adc1[ADC_SQR2] & 0xc0000000) | 0x00000000;

	/* ADC in power down mode by resetting the ADON bit */
	// adc1[ADC_CR2] |= ~0x00000001; // ADON - start
}

void read_adc(void)
{
	int temp;
	volatile uint32_t *adc1;
	adc1 = (uint32_t *) ADC1_BASE;

#ifdef TEMP_SENSOR
	/* select TEMP - ADC1_IN16 */
	adc1[ADC_SQR3] = (adc1[ADC_SQR3] & 0xc0000000) | 0x0000010;
	adc1[ADC_CR2] |= 0x00000001; // ADON - start conversion
	// adc1[ADC_SR] &= ~0x00000002 // clear EOC
	while (!(adc1[ADC_SR] & 0x00000002)); // wait for conversion
	/* read data - clear dual mode part (also resets EOC) */
	adc_values[0] = adc1[ADC_DR] & 0x00000fff; // 12-bit ADC
	// Temperature (in °C) = {(V25 - VSENSE) / Avg_Slope} + 25
	// V25 = 1430mV typ., Avg_Slope = 4.3 mV/C typ., Vref+ = VDD
	// VDD = 3300mV = 0x0fff => VSENSE = sense*3300/0x0fff
	// (Vintref was 1.16V, so guess that V25 is also min = 1340mV)
	// temp = ((1430 - VSENSE) / 4.3) + 25
	// calculate in 0.1mV units
	dbg1 = adc_values[0];
	temp = (adc_values[0] * 33000) / 0x0fff;
	dbg2 = temp;
	adc_values[0] = ((13400 - temp) / 43) + 25;
#if 1
	/* Vintref - for debug */
	adc1[ADC_SQR3] = (adc1[ADC_SQR3] & 0xc0000000) | 0x0000011;
	adc1[ADC_CR2] |= 0x00000001; // ADON - start conversion
	// adc1[ADC_SR] &= ~0x00000002 // clear EOC
	while (!(adc1[ADC_SR] & 0x00000002)); // wait for conversion
	/* read data - clear dual mode part (also resets EOC) */
	dbg3 = adc1[ADC_DR] & 0x00000fff; // 12-bit ADC
	dbg3 = dbg3 * 3300 / 0xfff; // Vintref in mV
#endif
#endif

#ifdef ADC_USED
	/* select ADC1_IN9 */
	adc1[ADC_SQR3] = (adc1[ADC_SQR3] & 0xc0000000) | 0x0000009;
	adc1[ADC_CR2] |= 0x00000001;
	// adc1[ADC_SR] &= ~0x00000002
	while (!(adc1[ADC_SR] & 0x00000002));
	adc_values[1] = adc1[ADC_DR] & 0x0000ffff;

	/* select ADC1_IN8 */
	adc1[ADC_SQR3] = (adc1[ADC_SQR3] & 0xc0000000) | 0x0000008;
	adc1[ADC_CR2] |= 0x00000001;
	// adc1[ADC_SR] &= ~0x00000002
	while (!(adc1[ADC_SR] & 0x00000002));
	adc_values[2] = adc1[ADC_DR] & 0x0000ffff;
#endif
}
#endif

void init(void)
{
	power_low = 0;

	/* RCC system reset */
	RCC_reset();

	/* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
	RCC_setup_clocks();

	/* TODO: figure this out - if needed */
	/* Enable Clock Security System(CSS): this will generate an NMI exception
	   when HSE clock fails */
	// RCC_ClockSecuritySystemCmd(ENABLE);

    inir_rtc();

    init_usart1();

#ifdef VOLTAGE_ALARM
    init_pwrmon();
#endif

#ifdef USE_ADC
    init_adc();
#endif

    /* for debugging usart1 baud problem */
#ifdef DEBUG_TIM1_CLK
    init_timer1();
#endif
}

uint32_t time_diff(uint32_t now, uint32_t earlier)
{
	int tmp = now - earlier;
	if (tmp < 0)
	{
		tmp = MS_PER_HOUR - (earlier - now); /* modulo hour */
	}
	return (uint32_t) tmp;
}

char nibble_to_hex(char n)
{
	if (n < 10) return (n + '0');
	else return ((n-10) + 'A');
}

void pr_dec (uint32_t value)
{
	int i;
	char num[11];
	char digit;
	num[10] = '\0';
	for (i=0; i<10; i++)
	{
		digit = (char)(value % 10);
		value /= 10;
		num[9-i] = nibble_to_hex(digit);
	}
	i = 0;

	while ((num[i] == '0') && (i < 9)) i++; // skip leading zeroes
	while (num[i])
	{
		putch((int)num[i++]);
	}
}

void pr_word(uint32_t value)
{
	int i;
	char nibble;
	putch((int)'0');
	putch((int)'x');
	for (i=0; i<8; i++)
	{
		nibble = (value >> ((7-i)*4)) & 0xf;
		putch((int)nibble_to_hex(nibble));
	}
}

void pr_text(char *ptr)
{
	while(*ptr) putch((int)(*(ptr++)));
}

void main(void)
{
	int ch;
	char chr;
	int flag, i;
	uint32_t last, rtc_time, rtc_old, chime, systicks, rounds;
	volatile uint32_t *gpioc, *syst, *rtc, *tim1, *rcc, *bkp;
	gpioc = (uint32_t *) PC_BASE;
	syst = (uint32_t *) SYSTIC_BASE;
	rtc = (uint32_t *) RTC_BASE;
	tim1 = (uint32_t *) TIM1_BASE;
	rcc = (uint32_t *) RCC_BASE;
	bkp = (uint32_t *) BKP_BASE;

	init();
	init_led();
	flag = 0; // led is OFF

	/* debug variables */
	last = ticks;
	chime = ticks;
	rounds = 0;
	i = 0;

#if 0
    /* check bus prescalers */
	pr_text("\r\n");
	pr_text("RCC_CFGR: ");
    pr_word((uint32_t)rcc[RCC_CFGR]);
	pr_text("\r\n");
#endif

#ifdef DEBUG_SYSTICK
	/* check the systick */
	systicks = ticks;
	pr_text("\r\nMinute\r\n*\r\n");
	while (time_diff(ticks, systicks) < 60000); /* a minute */
	pr_text("#\r\n");
#endif

#ifdef DEBUG_TIM1_CLK
	// TIM1_CNT, TIM1_SR
	systicks = ticks;
	i = 0;
	do
	{
		while (!(tim1[TIM1_SR] & 1)); // wait for UIF
		tim1[TIM1_SR] &= ~1; // clear UIF
		i++;

	} while (time_diff(ticks, systicks) < 60000); /* a minute */
	pr_text("\r\n");
	pr_text("Tim1 UIs: ");
	pr_word((uint32_t) i);
	pr_text("   (");
	pr_dec ((uint32_t) i);
	pr_text(" dec)\r\n\n");
#endif

#ifdef DEBUG_RTC_INIT
	pr_text("BKP data: ");
	pr_word(bkp_data);
	pr_text("\r\n");
#endif

	last = ticks;
	chime = ticks;
	rtc_old = 0;

#if 0
	rtc_old = (rtc[RTC_CNTH] << 16) & 0xffff0000;
	rtc_old |= (rtc[RTC_CNTL] & 0x0000ffff);
	pr_text("\r\nRTC old: ");
	pr_word(rtc_old);
	for(i=0; i<60; i++) // 10 minutes
	{
		/* for 60 seconds (about) */
		while (time_diff(ticks, chime) < 60000);
		chime = ticks;

	}
	rtc_time = (rtc[RTC_CNTH] << 16) & 0xffff0000;
	rtc_time |= (rtc[RTC_CNTL] & 0x0000ffff);
	pr_text("\r\nRTC new: ");
	pr_word(rtc_time);
	pr_text("\r\nRTC diff: ");
	pr_word(rtc_time - rtc_old);
	pr_text("\r\n");

	last = ticks;
	chime = ticks;
	rounds = 0;
	rtc_old = 0;
#endif

#if 0
	rtc_time = (rtc[RTC_CNTH] << 16) & 0xffff0000;
	rtc_time |= (rtc[RTC_CNTL] & 0x0000ffff);
	rtc_old = rtc_time;
	while (rtc_time - rtc_old < (10*60)) // 10 minutes
	// while (rtc_time - rtc_old < (60*60)); // hour
	{
		rtc_time = (rtc[RTC_CNTH] << 16) & 0xffff0000;
		rtc_time |= (rtc[RTC_CNTL] & 0x0000ffff);
	}
	pr_text("\r\nRTC: ");
	pr_word(rtc_time);
	pr_text("\r\nRTC diff: ");
	pr_word(rtc_time - rtc_old);
	pr_text("\r\n");
	rtc_old = 0;
#endif

	while (1)
	{
		if (power_low)
		{
			if ((bkp[BKP_DATA2] & 0x0000ffff) == 0)
			{
				bkp[BKP_DATA2] = 1;
				pr_text("\r\nPOWER LOW!\r\n");
			}
			power_low = 0;
		}
#ifdef DEBUG_USART1_ECHO
		/* USART echo */
		ch = getch();
		while (ch != -1)
		{
			putch(ch);
			i++;
			ch = getch();
		}
#endif

#ifdef APPLICATION
		ch = getch();
		if (ch  != -1)
		{
			chr = (char) ch;
			switch (ch)
			{
			case 'c': /* get time */
				{
					rtc_time = (rtc[RTC_CNTH] << 16) & 0xffff0000;
					rtc_time |= (rtc[RTC_CNTL] & 0x0000ffff);
					rtc_get_time();
					pr_text("\r\nRTC: ");
					pr_word(rtc_time);
					pr_text("\r\n");
					pr_dec((uint32_t) rtc_day);
					pr_text(" days, ");
					pr_dec((uint32_t) rtc_hour);
					pr_text(":");
					pr_dec((uint32_t) rtc_min);
					pr_text(":");
					pr_dec((uint32_t) rtc_sec);
					pr_text("\r\n");
					break;
				}

#ifdef STANDBY_ENABLED
			case 's':
				{
					go_standby();
					break;
				}
#endif

			default:
				{
					break; // some compilers want this
				}
			} /* end switch */
		}
#endif

#if 1
		/* blink once per second */
		if (time_diff(ticks, last) >= 500)
		{
			if (flag)
			{
				gpioc[GPIO_BSRR] = 0x00002000; /* set bit 13 - LED OFF */
				flag = 0;
				/* TODO: bookmark */
#ifdef USE_ADC
				read_adc();
#endif

#ifdef TEMP_SENSOR
				pr_text("TEMP: ");
				pr_dec(adc_values[0]);
				pr_text("\r\n");
#if 0
				pr_text("dbg1, Vsense, Vintref: ");
				pr_word(dbg1);
				pr_text(", ");
				pr_dec(dbg2);
				pr_text(", ");
				pr_dec(dbg3);
				pr_text("\r\n");
#endif
#endif

#ifdef ADC_USED
				pr_text("PB1, PB0: ");
				pr_word(adc_values[1]);
				pr_text(", ");
				pr_word(adc_values[2]);
				pr_text("\r\n");
#endif
			}
			else
			{
				gpioc[GPIO_BSRR] = 0x20000000; /* reset bit 13 - LED ON */
				flag = 1;
			}
			last = ticks;
		}
#endif

#ifdef DEBUG_RTC_CLK
		/* every 10 seconds (about) */
		if (time_diff(ticks, chime) >= 10000)
		{
			/* print RTC seconds */
			rtc_time = (rtc[RTC_CNTH] << 16) & 0xffff0000;
			rtc_time |= (rtc[RTC_CNTL] & 0x0000ffff);
			pr_text("\r\nRTC: ");
			pr_word(rtc_time);
			pr_text("\r\nRTC diff: ");
			pr_word(rtc_time - rtc_old);
			pr_text("\r\n");
			rtc_old = rtc_time;
			chime = ticks;
		}
#endif
		/* wait 1 second */
		//while (time_diff(ticks, last) < 1000);
		//last = ticks;

		if (rounds > 60)
		{
			rounds = 0; // place for breakpoint
		}
		rounds++;
	}
}


