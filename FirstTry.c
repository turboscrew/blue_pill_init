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
// for debugging PA1 interrupts by polling
//#define DEBUG_PA1
#define DEBUG_RTC_INIT
#define APPLICATION
#define STANDBY_ENABLED
#define RTC_WAKEUP
#define WKUP_WAKEUP
#define VOLTAGE_ALARM
// ADC not tested - not even tried
//#define ADC_USED
//#define TEMP_SENSOR
#define DHT_SENSOR
// SPI not tested - not even tried
#define SPI_USED

/* use xtal or internal clock for RTC */
#define RTC_USE_XTAL

//#define USART1_BAUD_115200
#define USART1_BAUD_19200
//#define USART1_BAUD_9600

// 72 MHz / 8 = 16MHz
// RFM69: max 10 MHz
#define SPI1_BAUD 2

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
/* FCK_CNT = fCK_PSC / (PSC[15:0] + 1).
   72 -> 1us per tick = 1 MHz */
/* The timer clock frequencies are automatically fixed by hardware. There are two cases:
1. if the APB prescaler is 1, the timer clock frequencies are set to the same frequency as
that of the APB domain to which the timers are connected.
2. otherwise, they are set to twice (×2) the frequency of the APB domain to which the
timers are connected. RM0008, 7.2 Clocks */
#define TIM2_PRESCALER 71


#define PERIPH_BASE           0x40000000

/* Buses */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)
#define PRIVPERIPH_BASE       0xe0000000

/* Peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
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
#define AFIO_EVCR 0
#define AFIO_MAPR 1
#define AFIO_EXTICR1 2
#define AFIO_EXTICR2 3
#define AFIO_EXTICR3 4
#define AFIO_EXTICR4 5
/* reg 6 doesn't exist */
#define AFIO_MAPR2 7
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
#define TIM_CR1 0
#define TIM_CR2 1
#define TIM_SMCR 2
#define TIM_DIER 3
#define TIM_SR 4
#define TIM_EGR 5
#define TIM_CCMR1 6
#define TIM_CCMR2 7
#define TIM_CCER 8
#define TIM_CNT 9
#define TIM_PSC 10
#define TIM_ARR 11
#define TIM_RCR 12
#define TIM_CCR1 13
#define TIM_CCR2 14
#define TIM_CCR3 15
#define TIM_CCR4 16
#define TIM_BDTR 17
#define TIM_DCR 18
#define TIM_DMAR 19
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
#define SPI_CR1 0
#define SPI_CR2 1
#define SPI_SR 2
#define SPI_DR 3
#define SPI_CRCPR 4
#define SPI_RXCRCR 5
#define SPI_TXCRCR 6
#define SPI_I2SCFGR 7
#define SPI_I2SPR 8

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

#define USART1_RXLEN 128
#define USART1_TXLEN 128
#define SPI1_BLEN 64

// from loader script
extern char __sys_stack;
extern char __usr_stack;
extern char __usr_stksz;
extern char __sys_stksz;

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

volatile uint8_t spi1_buff[SPI1_BLEN];

volatile uint8_t rtc_sec;
volatile uint8_t rtc_min;
volatile uint8_t rtc_hour;
volatile uint8_t rtc_day;

volatile uint32_t adc_values[3];

volatile uint8_t dht_ready;
volatile uint8_t dht_bits_left;
volatile uint8_t dht_ack_wait;
volatile uint32_t ts_rising;
volatile uint8_t dht_buff[9];
volatile uint32_t dht_dbg_ints;
volatile uint32_t dht_dbg_any, dht_dbg_any2;

volatile uint32_t dbg1, dbg2, dbg3, dbg4, dbg5, dbg6;

#ifdef DEBUG_PA1
volatile uint8_t dbg_pa1_flag;
volatile uint32_t dbg_tms_buff[256];
volatile uint32_t dbg_edge_buff[256];
volatile uint32_t dbg_tm_idx;
#endif

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

uint32_t time_diff(uint32_t now, uint32_t earlier)
{
	int tmp = now - earlier;
	if (tmp < 0)
	{
		tmp = MS_PER_HOUR - (earlier - now); /* modulo hour */
	}
	return (uint32_t) tmp;
}

void wait_ms(uint32_t wtime)
{
	uint32_t last = ticks;
	while (time_diff(ticks, last) < wtime);
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
void init_rtc(void)
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
	if (((bkp[BKP_DATA1] & 0x0000ffff) == 0x0000a5a5)
		|| ((bkp[BKP_DATA1] & 0x0000ffff) == 0x00005a5a))
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

	/* Enable PWR, BKP and SPI2, disable other peripherals */
	/* BKP must be enabled, otherwise RCC_BDCR-bits don't get written */
	rcc[RCC_APB1ENR] = (rcc[RCC_APB1ENR] & (uint32_t) 0xc5013600)
		| ((uint32_t) 0x18004000);

	/* Enable Backup domain access after bkp-reset
		for access to most RCC_BDCR-bits */
	pwr[PWR_CR] =(pwr[PWR_CR] & (uint32_t) 0xfffffe00)
		| ((uint32_t) 0x00000100); 	/* Set DBP-bit */

#ifdef RTC_USE_XTAL
	/* Set LSE ON and remove BDRST*/
    //rcc[RCC_BDCR] = (rcc[RCC_BDCR] & (uint32_t) 0xfffe7ff8)
	//	| ((uint32_t) 0x00000001);

    /* RTC clock source selection + RTC clock enable + LSE ON
       remove BDRST */
    rcc[RCC_BDCR] = (rcc[RCC_BDCR] & (uint32_t) 0xfffe7cf8)
		| ((uint32_t) 0x00008101);

    /* Wait until RTC external clock is stabilized -> LSERDY = 1 */
    tmp = 5000000;
    while ((rcc[RCC_BDCR] & (uint32_t) 0x00000002) != (uint32_t) 0x00000002)
    {
    	tmp--;
    	if (!tmp)
    		while(1);
    }

    /* wait for synchronization (RSF) */
    rtc[RTC_CRL] &= (uint32_t) 0xfffffff7; /* mark not synced */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000008));
#else
	/* Set LSE OFF */
    //rcc[RCC_BDCR] = (rcc[RCC_BDCR] & (uint32_t) 0xfffe7cf8);

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

    /* RTC clock source selection + RTC clock enable + LSE OFF
       + remove BDRST */
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
//    rtc[RTC_PRLL] = 0x00009C3F; /* Prescaler lo = 40 000 -> 1 Hz*/
    rtc[RTC_PRLL] = 0x00004E1F; // for debugging
#endif
    rtc[RTC_CRL] = 0x00000000; /* exit CNF */
	/* wait for last write to finish (RTOFF) */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000020));
	/* wait for synchronization (RSF) (can last a minute) */
    rtc[RTC_CRL] &= (uint32_t) 0xfffffff7; /* mark not synced */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000008));
	/* Mark RTC initialized */
#ifdef RTC_USE_XTAL
	bkp[BKP_DATA1] = (bkp[BKP_DATA1] & 0xffff0000) | 0x0000a5a5;
#else
	bkp[BKP_DATA1] = (bkp[BKP_DATA1] & 0xffff0000) | 0x00005a5a;
#endif
	/* Mark no power low */
	bkp[BKP_DATA2] = (bkp[BKP_DATA2] & 0xffff0000) | 0x00000000;
}

/* ******************** */
/* RE-INIT RTC */
void reinit_rtc(void)
{
	int tmp;
	volatile uint32_t *rcc, *pwr, *bkp, *rtc, *scb, *nvic;
	rcc = (uint32_t *) RCC_BASE;
	pwr = (uint32_t *) PWR_BASE;
	bkp = (uint32_t *) BKP_BASE;
	rtc = (uint32_t *) RTC_BASE;
	scb = (uint32_t *) SCB_BASE;
	nvic = (uint32_t *) NVIC_BASE;

	bkp_data = bkp[BKP_DATA1] & 0x0000ffff;

   /* Write '1' to BDRST-bit in RCC_BDCR to reset Backup domain */
	rcc[RCC_BDCR] |= ((uint32_t) 0x00010000);
	//rcc[RCC_BDCR] &= ~((uint32_t) 0x00010000);

	/* Enable BKP */
	/* BKP must be enabled, otherwise RCC_BDCR-bits don't get written */
	rcc[RCC_APB1ENR] |= 0x01000000;

	/* Enable Backup domain access after bkp-reset
		for access to most RCC_BDCR-bits */
	pwr[PWR_CR] =(pwr[PWR_CR] & (uint32_t) 0xfffffe00)
		| ((uint32_t) 0x00000100); 	/* Set DBP-bit */

	if (bkp_data == 0x00005a5a) // LSI was ON
	{
		/* Set LSI OFF */
		rcc[RCC_CSR] &= ~0x00000001; /* LSION */

		/* Set LSE ON */
		//rcc[RCC_BDCR] = (rcc[RCC_BDCR] & (uint32_t) 0xfffe7ff8)
		//	| ((uint32_t) 0x00000001);

		/* RTC clock source selection + RTC clock enable + LSE ON
		   + remove BDRST */
		rcc[RCC_BDCR] = (rcc[RCC_BDCR] & (uint32_t) 0xfffe7cf8)
			| ((uint32_t) 0x00008101);

		/* Wait until RTC external clock is stabilized -> LSERDY = 1 */
		tmp = 50000000;
		while ((rcc[RCC_BDCR] & (uint32_t) 0x00000002) != (uint32_t) 0x00000002)
		{
			tmp--;
			if (!tmp)
				while(1);
		}

		/* RTC clock source selection + RTC clock enable */
		//rcc[RCC_BDCR] = (rcc[RCC_BDCR] & (uint32_t) 0xfffe7cf8)
		//	| ((uint32_t) 0x00008101);

		/* wait for synchronization (RSF) */
		rtc[RTC_CRL] &= (uint32_t) 0xfffffff7; /* mark not synced */
		while (!(rtc[RTC_CRL] & (uint32_t) 0x00000008));
	}
	else
	{
		/* Set LSE OFF */
		//rcc[RCC_BDCR] = (rcc[RCC_BDCR] & (uint32_t) 0xfffe7cf8);

		/* Set LSI ON */
		rcc[RCC_CSR] |= 0x00000001; /* LSION */

		/* Wait until RTC internal clock is stabilized -> LSIRDY = 1 */
		tmp = 50000000;
		while ((rcc[RCC_CSR] & (uint32_t) 0x00000002) != (uint32_t) 0x00000002)
		{
			tmp--;
			if (!tmp)
				while(1);
		}

		/* RTC clock source selection + RTC clock enable
		   + remove BDRST */
		rcc[RCC_BDCR] = (rcc[RCC_BDCR] & (uint32_t) 0xfffe7cf8)
			| ((uint32_t) 0x00008200);
		/* wait for synchronization (RSF) (can last a minute) */
		rtc[RTC_CRL] &= (uint32_t) 0xfffffff7; /* mark not synced */
		while (!(rtc[RTC_CRL] & (uint32_t) 0x00000008));
	}

	/* wait for last write to finish (RTOFF) */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000020));

	rtc[RTC_CRL] = 0x00000010; /* enter CNF */
    rtc[RTC_CNTH] = 0x00000000; /* Count high */
    rtc[RTC_CNTL] = 0x00000000; /* Count low */
	if (bkp_data == 0x00005a5a) // LSI was ON
	{
		rtc[RTC_PRLH] = 0x00000000; /* Prescaler hi */
		rtc[RTC_PRLL] = RTC_PRESCALER_XTAL; /* Prescaler lo */
	}
	else
	{
		/* Internal RC - 40 kHz typical (30 - 60) */
		/* Make faster (double) to be distinct from xtal */
		rtc[RTC_PRLH] = 0x00000000; /* Prescaler hi */
		rtc[RTC_PRLL] = 0x00004E1F; /* Prescaler lo = 40 000 -> 1 Hz*/
		//rtc[RTC_PRLL] = 0x00009C3F; /* Prescaler lo = 40 000 -> 1 Hz*/
	}
    rtc[RTC_CRL] = 0x00000000; /* exit CNF */
	/* wait for last write to finish (RTOFF) */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000020));
	/* wait for synchronization (RSF) (can last a minute) */
    rtc[RTC_CRL] &= (uint32_t) 0xfffffff7; /* mark not synced */
	while (!(rtc[RTC_CRL] & (uint32_t) 0x00000008));
	/* Mark RTC initialized */
	if (bkp_data == 0x00005a5a) // LSI was ON
	{
		/* now LSE is ON */
		bkp[BKP_DATA1] = (bkp[BKP_DATA1] & 0xffff0000) | 0x0000a5a5;
	}
	else
	{
		/* now LSI is ON */
		bkp[BKP_DATA1] = (bkp[BKP_DATA1] & 0xffff0000) | 0x00005a5a;
	}
	/* Mark no power low */
	bkp[BKP_DATA2] = (bkp[BKP_DATA2] & 0xffff0000) | 0x00000000;
}

/* ******************** */

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

	/* TODO: save data over standby */
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
	// dsb is probably a good idea before wfi
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
	tim1[TIM_PSC] = TIM1_PRESCALER;
	tim1[TIM_ARR] = 0x2710; /* 10 000 = 1 s */
	tim1[TIM_CNT] = 0x2710; /* initialize */

	tim1[TIM_CR1] = (tim1[TIM_CR1] & 0xfffffc00)
		| 0x0080; // check ARPE, URS and UDIS; CEN is needed
	tim1[TIM_CR2] &= 0xffff8002; // check MMS
	tim1[TIM_SMCR]	&= 0xffff0008;
	tim1[TIM_DIER] &= 0xffff8000;
#if 1
	tim1[TIM_EGR] = (tim1[TIM_EGR] & 0xffffff00)
		| 0x0001; // UG?
#else
	tim1[TIM_EGR] &= 0xffffff00;
#endif

	tim1[TIM_CR1] |= 0x00000001; /* CEN - counter enable */
}

#ifdef DHT_SENSOR
// TODO: Change to use input capture instead of GPIO interrupts.
//
// NOW:
// Timer 2 a free running 1 us 16-bit timer (for any use)
// Output compare for timing the start pulse
// - a GPIO with both rising and falling edge interrupts for data
// - on edge interrupt the time stamp is read from the counter and
//   the bit level is read to define if it's rising or falling edge

void init_timer2(void)
{
	volatile uint32_t *tim2, *rcc, *exti, *nvic, *afio;
	tim2 = (uint32_t *) TIM2_BASE;
	rcc = (uint32_t *) RCC_BASE;
	exti = (uint32_t *) EXTI_BASE;
	nvic = (uint32_t *) NVIC_BASE;
	afio = (uint32_t *) AFIO_BASE;

	dht_ready = 0; // not read yet
	dht_ack_wait = 0;
	dht_dbg_ints = 0;
	/* CH2, uses PA1, so let's use that just in case... */
	/* Does port A have clock? */
	if (!(rcc[RCC_APB2ENR] & 0x00000004)) /* if port A doesn't have clock */
	{
		rcc[RCC_APB2ENR] |= 0x00000004;
	}
	/* clock for timer2 */
	rcc[RCC_APB1ENR] |= ((uint32_t) 0x00000001);

	/* FCK_CNT = fCK_PSC / (PSC[15:0] + 1).
	   TIM2_PRESCALER = 71 -> 1us tick */
/* The timer clock frequencies are automatically fixed by hardware. There are two cases:
1. if the APB prescaler is 1, the timer clock frequencies are set to the same frequency as
that of the APB domain to which the timers are connected.
2. otherwise, they are set to twice (×2) the frequency of the APB domain to which the
timers are connected. RM0008, 7.2 Clocks */

	/* Time base configuration */
	tim2[TIM_PSC] = TIM2_PRESCALER;
	tim2[TIM_ARR] = 0xffff; /* 'reload' is actually limit */
	tim2[TIM_CNT] = 0x0000; /* initialize */

	tim2[TIM_CR1] = (tim2[TIM_CR1] & 0xfffffc00)
	//	| 0x0007; // URS, UDIS, CEN
		| 0x0001; // CEN

	/* compare match for 1 ms dht-reading start pulse */
	tim2[TIM_DIER] &= 0xffffa0a0; // disable interrupts
	tim2[TIM_CR2] &= 0xffffff07; // clear (check MMS)
	tim2[TIM_SMCR] &= 0xffff0008; // clear slave mode
	/* compare/capture mode */
#if 0
	tim2[TIM_CCMR1] &= 0xffff0000; // OC1M = 000
#else
	tim2[TIM_CCMR1] = (tim2[TIM_CCMR1] & 0xffff0000)
		| 0x00000010; // OC1M = 001
#endif
	tim2[TIM_EGR] = (tim2[TIM_EGR] & 0xffffffa0)
		| 1; // CC1G (enable compare match event), UG

	/* configure PA1 interrupt for both edges */
	nvic[NVIC_ICER_0] |= 0x00000080; // mask EXTI1 IRQ
	nvic[NVIC_ICPR_0] |= 0x00000080; // clear EXTI1 pending
	afio[AFIO_EXTICR1] = (afio[AFIO_EXTICR1] & 0xffffff0f)
		| 0x00000000; // PA1 to EXTI1
	exti[EXTI_IMR] &= ~2; // mask exti1 irq
	exti[EXTI_EMR] &= ~2; // mask exti1 event
	exti[EXTI_RTSR] |= 2; // exti1 rising edge irq
	exti[EXTI_FTSR] |= 2; // exti1 falling edge irq
	exti[EXTI_PR] |= 2; // clear pending event
}

void dht_request(void)
{
	/* CH2, uses PA1, so let's use that in case... */
	uint32_t tmp1; // for debug
	volatile uint32_t *tim2, *gpioa, *nvic;
	tim2 = (uint32_t *) TIM2_BASE;
	gpioa = (uint32_t *) PA_BASE;
	nvic = (uint32_t *) NVIC_BASE;

	// DHT being read
	dht_ready = 0;

	tim2[TIM_DIER] &= 0xffffa0a0; // disable interrupts

#if 1
	tim2[TIM_CCMR1] &= 0xffff0000; // OC1M = 000
#else
	tim2[TIM_CCMR1] = (tim2[TIM_CCMR1] & 0xffff0000)
		| 0x00000010; // OC1M = 001
#endif
	tim2[TIM_EGR] = (tim2[TIM_EGR] & 0xffffffa0)
		| 2; // CC1G (enable compare match event)

	/* pin PA1 = request output to AM2301 */
	gpioa[GPIO_BSRR] = 0x00000002; // PA1 high before enabling output
#if 0
	/* GPIO open-drain output, max 10 MHz */
	gpioa[GPIO_CRL] = (gpioa[GPIO_CRL] & 0xffffff0f)
			| 0x00000050;
#else
	gpioa[GPIO_CRL] = (gpioa[GPIO_CRL] & 0xffffff0f)
			| 0x00000010; // push-pull 10MHz
#endif
	gpioa[GPIO_BSRR] = 0x00020000; // PA1 low
	// match after 1 ms, even if counter overflows
	// min 800us, typ 1ms, max 20 ms
	// 1000 doesn't seem to be enough - most libraries seem to
	// use closer to 20 ms pulses.
#if 1
	// for debugging
	tmp1 = (tim2[TIM_CNT] + 18000) % 0x10000;
	tim2[TIM_CCR1] = tmp1;

#else
	tim2[TIM_CCR1] = (tim2[TIM_CNT] + 18000) % 0x10000;
#endif
	tim2[TIM_SR] &= 0xffffe1a0; // clear flags
	tim2[TIM_DIER] = (tim2[TIM_DIER] & 0xffffa0a0)
		| 2; // CC1IE (enable compare 1 interrupt)
	nvic[NVIC_ICPR_0] |= 0x10000000; // clear tim2 pending
	nvic[NVIC_ISER_0] |= 0x10000000; // enable tim2 IRQ (28)
}
#endif

void timer2_irq(void)
{
	/* CH2, uses PA1, so let's use that in case... */
	volatile uint32_t *tim2, *gpioa, *nvic, *exti;
	uint32_t tmp, tmp2, tmp3;
	tim2 = (uint32_t *) TIM2_BASE;
	gpioa = (uint32_t *) PA_BASE;
	nvic = (uint32_t *) NVIC_BASE;
	exti = (uint32_t *) EXTI_BASE;

	// tmp = tim2[TIM_SR]; // for debug
	nvic[NVIC_ICER_0] |= 0x10000000; // disable tim2 IRQ
	nvic[NVIC_ICPR_0] |= 0x10000000; // clear tim2 pending
	tim2[TIM_DIER] &= 0xffffa0a0; // disable compare interrupts
	//gpioa[GPIO_BSRR] = 0x00000002; // PA1 high (idle)
	/* Start receiving */
#if 0
	/* pin PA1 = tmr2_ch2 */
	/* input floating - AM2301 requires 1kOhm pull-up => 5 mA */
	/* STM32F103 can source/sink 25mA, and it's pull-up is ~40kOhm */
	gpioa[GPIO_CRL] = (gpioa[GPIO_CRL] & 0xffffff0f)
			| 0x00000040;
#else
	/* pin PA1 = tmr2_ch2 */
	/* Input pull-up/down: min. 30kOhm, typ. 40kOhm, max. 50kOhm */
	gpioa[GPIO_CRL] = (gpioa[GPIO_CRL] & 0xffffff0f)
			| 0x00000080;
	/* PA1 pull-up - line up to end the start pulse */
	gpioa[GPIO_ODR] |= 0x00000002;
	dht_ack_wait = 1; // the next is not yet data
#endif
	// exti[EXTI_PR] |= 2; // clear pending event (if needed)
	dht_dbg_ints = 0; // for debug
	dht_dbg_any = 0;
	dht_dbg_any2 = 0;
#ifndef DEBUG_PA1
	nvic[NVIC_ICPR_0] |= 0x00000080; // clear EXTI1 pending
	nvic[NVIC_ISER_0] |= 0x00000080; // enable EXTI1 IRQ
	exti[EXTI_PR] |= 2; // clear pending event
	exti[EXTI_IMR] |= 2; // enable exti1 irq
#else
	// debugging by polling
	nvic[NVIC_ICPR_0] |= 0x00000080; // clear EXTI1 pending
	nvic[NVIC_ICER_0] |= 0x00000080; // disable EXTI1 IRQ
	exti[EXTI_PR] |= 2; // clear pending event
	exti[EXTI_IMR] |= 2; // enable exti1 irq

	tmp2 = tim2[TIM_CNT];

	dbg_tm_idx = 0;
	while (gpioa[GPIO_IDR] & 2); // wait until sensor starts
	dbg_tms_buff[dbg_tm_idx] = tim2[TIM_CNT];
	dbg_edge_buff[dbg_tm_idx] = 0;
	dbg_tm_idx++;
	for (tmp=0; tmp<132; tmp++)
	{
		// wait until data changes
		while (((gpioa[GPIO_IDR] & 2) >> 1) == dbg_edge_buff[dbg_tm_idx-1]);
		//while (!(exti[EXTI_PR] & 2));
		//while (!(nvic[NVIC_ICPR_0] & 0x00000080));
		dbg_tms_buff[dbg_tm_idx] = tim2[TIM_CNT];
		if (gpioa[GPIO_IDR] & 2) // if PA1 = '1' rising edge
		{
			dbg_edge_buff[dbg_tm_idx] = 1;
		}
		else
		{
			dbg_edge_buff[dbg_tm_idx] = 0;
		}
		dbg_edge_buff[dbg_tm_idx] |= (exti[EXTI_PR] & 2)<< 3;
		dbg_tm_idx++;
		exti[EXTI_PR] |= 2; // clear PA1 pending
		nvic[NVIC_ICPR_0] |= 0x00000080; // clear EXTI1 pending
	}

	dbg_pa1_flag = 1;
#endif
}

void pa1_irq(void)
{
	uint32_t tmp, tmp2, tmp3;
	uint8_t i, val;
	volatile uint32_t *tim2, *gpioa, *nvic, *exti;
	tim2 = (uint32_t *) TIM2_BASE;
	gpioa = (uint32_t *) PA_BASE;
	nvic = (uint32_t *) NVIC_BASE;
	exti = (uint32_t *) EXTI_BASE;

	tmp = tim2[TIM_CNT];

	/* a look at AOSONG's own data sheet showed that
	   a state machine is actually needed. The pulse
	   widths may overlap and cause problems */

	exti[EXTI_PR] |= 2; // clear pending event
	// dont't clear NVIC pending bits:
	// "For example, the pending status of the exception will be
	// cleared and the active bit of the exception will be set."
	// "When the processor starts the interrupt handler, the active
	// is set to '1' and cleared when the interrupt return is executed."

	/* a debug as a vector of time stamps might be necessary */
	if (gpioa[GPIO_IDR] & 2) // if PA1 = '1' rising edge
	{
		ts_rising = tmp; // rising edge timestamp - start of pulse
		dht_dbg_any = 1;
		dht_dbg_any2 = 0;
	}
	else // falling edge
	{
		if (dht_ack_wait) // Not data but 'ack' from sensor
		{
			dht_dbg_ints++;
			exti[EXTI_PR] |= 2; // clear pending event
			dht_ack_wait = 0;
			return;
		}
		dht_dbg_any = 0;
		dht_dbg_any2 = tmp;

		// 0x10000 is the "length" of the 16-bit counter
		// say, length = 100, start = 10, end = 90
		// end - start = 80, (80 + 100) % 100 = 80
		// if length = 100, start = 90, end = 10
		// end - start = -80, -80 + 100 = 20, 20 % 100 = 20
		tmp = (tmp + 0x10000 - ts_rising) % 0x10000;
		// DHT-start = 80 us, '1' = 75 us, '0' = 38 us
		// msb-first, 16-bit hum + 16-bit temp + 8-bit checksum
		if (tmp > 75)
		{
			/* DHT's response to start pulse */
			dht_bits_left = 40; // bits to read
		}
		else if (tmp > 40)
		{
			/* '1' */
			i = (40 - dht_bits_left) / 8; // buffer index
			val = (dht_buff[i] << 1) | 1; // add '1'
			dht_buff[i] = val;
			dht_bits_left--;
		}
		else
		{
			/* '0' */
			i = (40 - dht_bits_left) / 8; // buffer index
			val = (dht_buff[i] << 1); // add '0'
			dht_buff[i] = val;
			dht_bits_left--;
		}
		if (!dht_bits_left)
		{
			/* end DHT reading */
			dht_ready = 1; // new DHT data read
			// stop receiving
			nvic[NVIC_ICER_0] |= 0x00000080; // mask EXTI1 IRQ
			nvic[NVIC_ICPR_0] |= 0x00000080; // clear EXTI1 pending
		}
	}
	dht_dbg_ints++;
}

void dht_read(void)
{
	// old data
	dht_request();
	wait_ms(2000);
	// current data
	dht_request();
	wait_ms(2000);
	// re-read until success
	while (!dht_ready)
	{
		dht_request();
		wait_ms(2000);
	}
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
	volatile uint32_t *exti, *nvic;
	exti = (uint32_t *) EXTI_BASE;
	nvic = (uint32_t *) NVIC_BASE;
	power_low = 1;
	exti[EXTI_PR] |= 0x00010000; // clear pending event
	nvic[NVIC_ICPR_0] |= 0x00000002; // clear EXTI1 pending
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
	/* ADON is a bit difficult, because writing 0 to it turns ADC off
	   and writing 1 to it starts conversion - there is no "leave alone"-
	   option... */
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
	// (Vrefint was 1.16V, so guess that V25 is also min = 1340mV)
	// temp = ((1430 - VSENSE) / 4.3) + 25
	// calculate in 0.1mV units
	dbg1 = adc_values[0];
	temp = (adc_values[0] * 33000) / 0x0fff;
	dbg2 = temp;
	adc_values[0] = ((13400 - temp) / 43) + 25;
#if 1
	/* Vrefint - for debug */
	adc1[ADC_SQR3] = (adc1[ADC_SQR3] & 0xc0000000) | 0x0000011;
	adc1[ADC_CR2] |= 0x00000001; // ADON - start conversion
	// adc1[ADC_SR] &= ~0x00000002 // clear EOC
	while (!(adc1[ADC_SR] & 0x00000002)); // wait for conversion
	/* read data - clear dual mode part (also resets EOC) */
	dbg3 = adc1[ADC_DR] & 0x00000fff; // 12-bit ADC
	dbg4 = dbg3;
	dbg3 = dbg3 * 33000 / 0xfff; // Vintref in 0.1mV
	// if Vrefint: 1.16V min., 1.20V typ., 1.24V max.
	// if vrefint were 1.20...
	// dbg4 * VDD / 0xfff = 1200 mV
	// VDD = 1200 mV * 0xfff / dbg4
	//dbg4 = 12000 * 0xfff / dbg4; // calculated VDD
	dbg4 = 32500; // measured
	dbg5 = dbg1 * dbg4 / 0xfff; // Vsense by VDD
//	dbg6 = ((14300 - dbg5) / 43) + 25; // Temp
	dbg6 = ((13400 - dbg5) / 43) + 25;
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

#ifdef SPI_USED
void init_spi1(void)
{
	volatile uint32_t *spi1, *gpioa, *rcc;

	spi1 = (uint32_t *)SPI1_BASE;
	gpioa = (uint32_t *) PA_BASE;
	rcc = (uint32_t *) RCC_BASE;

	/* Does port A have clock? */
	if (!(rcc[RCC_APB2ENR] & 0x00000004)) /* if port A doesn't have clock */
	{
		rcc[RCC_APB2ENR] |= 0x00000004;
	}
	rcc[RCC_APB2ENR] |= 0x00001000; /* clock for SPI1 */

#if 1
	/* pins PA4 = NSS, PA5 = SCK, PA6 = MISO, PA7 = MOSI */
	/* NSS, SCK,MOSI: Alternate function push-pull, output max 2 MHz */
	/* MISO: Input floating */
	gpioa[GPIO_CRL] = (gpioa[GPIO_CRL] & 0x0000ffff)
			| 0xa4aa0000;
#else
	/* pins PA4 = NSS, PA5 = SCK, PA6 = MISO, PA7 = MOSI */
	/* NSS, SCK,MOSI: Alternate function push-pull, output max 2 MHz */
	/* MISO: Input pull-up */
	gpioa[GPIO_CRL] = (gpioa[GPIO_CRL] & 0x0000ffff)
			| 0xa8aa0000;
	gpioa[GPIO_ODR] |= 0x00000040; /* PA6 pull-up */
#endif
	/* no BIDIR, no CRC, 8-bit, full-duplex, HW NSS,
	   msb first, disable, master, CPOL=0, CPHA=0, + baud */
	spi1[SPI_CR1] = 0x00000004 | (SPI1_BAUD << 3);
	/* TODO: check the NSS configuration */
	/* disable interrupts and DMA, set SSOE */
	spi1[SPI_CR2] = (spi1[SPI_CR2] & 0xffffff18)
			| 0x00000004;
	spi1[SPI_I2SCFGR] &= 0xfffff040; // SPI-mode
}

// buff is rx/tx buffer tx data is replaced with rx data
// and the length needs to be the bigger of the two
// if there is more to receive than to send, the rest
// of tx data in buffer needs to be filled with zeros
int xfer_spi1(char *buff, int count)
{
	volatile uint32_t *spi1;
	int res;
	uint32_t status;
	uint32_t rxi, txi;

	spi1 = (uint32_t *)SPI1_BASE;
	res = 0;
	/* if SPI is not ON */
	if ((spi1[SPI_CR1] & 0x00000044) != 0x00000044)
	{
		/* turn ON */
		spi1[SPI_CR1] |= 0x00000044;
	}
	status = spi1[SPI_SR] & 0xff;
	if (status & 0x68)
	{
		/* errors */
		res = -status;
		spi1_buff[0] = spi1[SPI_DR]; // clear error flags
		return res;
	}

	rxi = 0;
	txi = 0;
	// Let's believe that tx is at least 1 step ahead of rx
	while (rxi < count)
	{
		status = spi1[SPI_SR] & 0xff;
		if (status & 1) // receiver not empty - next 8 bits received
		{
			spi1_buff[rxi++] = spi1[SPI_DR];
		}
		if (status & 2) // transmitter empty
		{
			spi1[SPI_DR] = spi1_buff[txi++];
		}
	}
	res = rxi;
	/* turn OFF */
	spi1[SPI_CR1] &= ~0x00000044;
	return res;
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

	init_led();

    init_rtc();

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

#ifdef DHT_SENSOR
    init_timer2();
#endif
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

void pr_stack(void)
{
	uint32_t sp1, sp2, tmp1, ssz;

	asm volatile
	(
			"MRS %[ret_reg], CONTROL\n\t"
			: [ret_reg] "=r" (tmp1)::
	);
	pr_text("CONTROL: ");
	pr_word(tmp1);

	// values from linker script
	if (tmp1 & 2) // thread mode - psp
	{
		ssz = (uint32_t)&__usr_stksz;
		sp1 = (uint32_t)&__usr_stack;
	}
	else // handler mode - msp
	{
		ssz = (uint32_t)&__sys_stksz;
		sp1 = (uint32_t)&__sys_stack;
	}
	// current stack pointer
	asm volatile
	(
			"MOV %[ret_reg], r13\n\t"
			: [ret_reg] "=r" (sp2)::
	);
	pr_text("\r\nStack: ");
	pr_word(sp1);
	pr_text(" - ");
	pr_word(sp1 - ssz);
	pr_text(" sp: ");
	pr_word(sp2);
	pr_text("\r\n");
}

void main(void)
{
	int ch;
	char chr;
	int flag, i, j;
	uint32_t last, rtc_time, rtc_old, chime, systicks, rounds;
	uint32_t tmpval, dht_last, tmp1, tmp2, tmp3, tmp4;
	volatile uint32_t *gpioc, *syst, *rtc, *tim1, *tim2, *rcc, *bkp;
	gpioc = (uint32_t *) PC_BASE;
	syst = (uint32_t *) SYSTIC_BASE;
	rtc = (uint32_t *) RTC_BASE;
	tim1 = (uint32_t *) TIM1_BASE;
	tim2 = (uint32_t *) TIM2_BASE;
	rcc = (uint32_t *) RCC_BASE;
	bkp = (uint32_t *) BKP_BASE;

	init();

#if 1
	pr_stack();
	/* Time for printing */
	wait_ms(50);
#endif

#ifdef DEBUG_PA1
	dbg_pa1_flag = 0;
#endif

#ifdef DEBUG_RTC_INIT
	/* BKP data before RTC init */
	pr_text("Init - BKP data: ");
	pr_word(bkp_data);
	pr_text("\r\n");
#endif

	if ((bkp[BKP_DATA1] & 0xffff) == 0xa5a5) // crystal
	{
		pr_text("\r\nRTC LSE - 1 tic / s");
	}
	else
	{
		pr_text("\r\nRTC LSI - 2 tics / s");
	}
	pr_text("\r\n");
	/* Time for printing */
	wait_ms(50);

	flag = 0; // led is OFF

	/* debug variables */
	dht_last = ticks;
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
		chime = ticks;
		while (time_diff(ticks, chime) < 60000);
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

#if 0
	// check timer2 frequency
	tmp1 = syst[SYST_CVR];
	// systick counts DOWN from TICS_PER_MS
	while (tmp1 < (TICS_PER_MS - 20))
	{
	    tmp1 = syst[SYST_CVR];
	}
	tmp1 = syst[SYST_CVR];
	//tmp1 = ticks;
	tmp2 = tim2[TIM_CNT];
	//while (time_diff(ticks, tmp1) < 20)
	while (1)
	{
		tmp4 = tmp1 - syst[SYST_CVR];
		if (tmp4 >= (TICS_PER_MS / 2)) break;
	}
	//tmp4 = ticks;
	tmp4 = syst[SYST_CVR];
	tmp3 = tim2[TIM_CNT];
	pr_text("Systic: ");
	pr_word(tmp1);
	pr_text(" ");
	pr_word(tmp4);
	pr_text(" d: ");
	pr_word(tmp1 - tmp4);
	pr_text("\r\nTimer2: ");
	pr_word(tmp2);
	pr_text(" ");
	pr_word(tmp3);
	pr_text("\r\n");
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
			case 'x':
				{
					reinit_rtc();
					pr_text("\r\nRTC re-init");
					if ((bkp[BKP_DATA1] & 0xffff) == 0xa5a5) // crystal
					{
						pr_text("\r\nRTC LSI->LSE");
					}
					else
					{
						pr_text("\r\n->RTC LSE->LSI");
					}
					pr_text("\r\n");
					break;
				}
			case 'z':
				{
					bkp[BKP_DATA1] = 0; // force clock reset
					break;
				}
#ifdef DHT_SENSOR
			case 'd':
				{
					dht_request();
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

#ifdef DHT_SENSOR
		// TODO: embed the multiple reading until success into
		// into a routine of its own
		// DHT sensor read test about once per minute
		if (time_diff(ticks, dht_last) >= 60000)
		{
			dht_read();
			dht_last = ticks;
		}
#endif

#ifdef DEBUG_PA1
		if (dbg_pa1_flag)
		{
			// dump edges: time stamps, line values and levels
			pr_text("PA1:\r\n");
			pr_word(dbg_tms_buff[0]);
			pr_text(" ");
			pr_dec((uint32_t)dbg_edge_buff[0]);
			pr_text("\r\n");
			for (j=1; j<dbg_tm_idx; j++)
			{
				pr_word(dbg_tms_buff[j]);
				pr_text(" ");
				pr_dec((uint32_t)dbg_edge_buff[j]);
				tmp1 = (uint32_t)dbg_tms_buff[j] + 0x10000 - (uint32_t)dbg_tms_buff[j-1];
				tmp1 %= 0x10000;
				pr_text(" -> ");
				pr_dec(tmp1);
				// if previous data was '1', the time stamp is the
				// duration of high level
				if (dbg_edge_buff[j-1])
				{
					pr_text(" H\r\n");
				}
				else
				{
					pr_text(" L\r\n");

				}
				// let USART send out stuff before putting
				// more text in the tx buffer
				wait_ms(20);
			}
			pr_text("\r\n");

			dbg_pa1_flag = 0;

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
#if 1
				pr_text("dbg1, Vs, Vref: ");
				pr_word(dbg1);
				pr_text(", ");
				pr_dec(dbg2);
				pr_text(", ");
				pr_dec(dbg3);
				pr_text("\r\n");
				pr_text("VDD, Vs2, T2: ");
				pr_dec(dbg4);
				pr_text(", ");
				pr_dec(dbg5);
				pr_text(", ");
				pr_dec(dbg6);
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

#ifdef DHT_SENSOR
		if (dht_ready) // new data from DHT
		{

			// checksum
			tmpval= 0;
			for (i=0; i<4; i++)
			{
				tmpval += (uint32_t)dht_buff[i];
			}
			tmpval &= 255;
			if (tmpval != (uint32_t)dht_buff[4])
			{
				pr_text("\r\nDHT: checksum mismatch\r\n");
			}
			// RH
			tmpval = ((uint32_t)dht_buff[0]) << 8;
			tmpval |= ((uint32_t)dht_buff[1]);
			pr_text("\r\nDHT: RH=");
			pr_dec(tmpval);
			tmpval = ((uint32_t)dht_buff[2]) << 8;
			tmpval |= ((uint32_t)dht_buff[3]);
			pr_text(" T=");
			if (tmpval & 0x00008000) // negative temp
			{
				pr_text("-"); // print sign
				tmpval &= 0x00007fff; // drop sign
			}
			pr_dec(tmpval);
			pr_text("\r\n");
			dht_ready = 0; // don't read until re-fetched
			rtc_time = (rtc[RTC_CNTH] << 16) & 0xffff0000;
			rtc_time |= (rtc[RTC_CNTL] & 0x0000ffff);
			rtc_get_time();
			pr_text("\r\nRTC: ");
			pr_dec((uint32_t) rtc_hour);
			pr_text(":");
			pr_dec((uint32_t) rtc_min);
			pr_text(":");
			pr_dec((uint32_t) rtc_sec);
			pr_text("\r\n");

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
	}
}


