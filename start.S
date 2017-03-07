.syntax unified
.extern __sys_stack
.extern __usr_stack
.extern main
.extern clkint
.extern usart1_irq
.extern pwr_irq
.extern timer2_irq
.extern pa1_irq
.globl _start

.sect .vectors
@ Exception vector table
@ System devices
.word __sys_stack @ initial SP
.word _start	  @ Reset
.word gen_exc @ NMI
.word gen_exc @ Hard Fault
.word gen_exc @ Memory Fault
.word gen_exc @ Bus Fault
.word gen_exc @ Usage Fault
.word 0 @ Reserved
.word 0 @ Reserved
.word 0 @ Reserved
.word 0 @ Reserved
.word gen_exc @ SVC
.word gen_exc @ (Debug)
.word 0 @ Reserved
.word gen_exc @ PendSV
.word systmr @ SysTick
@ Peripherals
.word gen_exc @ IRQ0  WWDG
.word pwrint @ IRQ1  PVD
.word gen_exc @ IRQ2  TAMPER
.word gen_exc @ IRQ3  RTC
.word gen_exc @ IRQ4  FLASH
.word gen_exc @ IRQ5  RCC
.word gen_exc @ IRQ6  EXTI0
.word pa1_exc @ IRQ7  EXTI1
.word gen_exc @ IRQ8  EXTI2
.word gen_exc @ IRQ9  EXTI3
.word gen_exc @ IRQ10 EXTI4
.word gen_exc @ IRQ11 DMA1 CH1
.word gen_exc @ IRQ12 DMA1 CH2
.word gen_exc @ IRQ13 DMA1 CH3
.word gen_exc @ IRQ14 DMA1 CH4
.word gen_exc @ IRQ15 DMA1 CH5
.word gen_exc @ IRQ16 DMA1 CH6
.word gen_exc @ IRQ17 DMA1 CH7
.word gen_exc @ IRQ18 ADC 1_2
.word gen_exc @ IRQ19 USB HP / CAN TX
.word gen_exc @ IRQ20 USB LP / CAN RX0
.word gen_exc @ IRQ21 CAN RX0
.word gen_exc @ IRQ22 CAN SCE
.word gen_exc @ IRQ23 EXTI9_5
.word gen_exc @ IRQ24 TIM1_BRK
.word gen_exc @ IRQ25 TIM1_UP
.word gen_exc @ IRQ26 TIM1_TRG/COM
.word gen_exc @ IRQ27 TIM1_CC
.word tmr2_exc @ IRQ28 TIM2
.word gen_exc @ IRQ29 TIM3
.word gen_exc @ IRQ30 TIM4
.word gen_exc @ IRQ31 I2C1_EV
.word gen_exc @ IRQ32 I2C1_ER
.word gen_exc @ IRQ33 I2C2_EV
.word gen_exc @ IRQ34 I2C2_ER
.word gen_exc @ IRQ35 SPI1
.word gen_exc @ IRQ36 SPI2
.word usart1_exc @ IRQ37 USART1
.word gen_exc @ IRQ38 USART2
.word gen_exc @ IRQ39 USART3
.word gen_exc @ IRQ40 EXTI15_10
.word gen_exc @ IRQ41 RTC_Alarm
.word gen_exc @ IRQ42 USB_Wakeup

.text

.thumb_func
systmr:
	push {lr}
	bl clkint
	pop {pc}

.thumb_func
pwrint:
	push {lr}
	bl pwr_irq
	pop {pc}

.thumb_func
usart1_exc:
	push {lr}
	bl usart1_irq
	pop {pc}

.thumb_func
tmr2_exc:
	push {lr}
	bl timer2_irq
	pop {pc}

.thumb_func
pa1_exc:
	push {lr}
	bl pa1_irq
	pop {pc}

.thumb_func
gen_exc:
	ldr r0, =0xe000ed28 @ CFSR
	ldr r0, [r0]
	ldr r1, =0xe000ed38 @ CFAR
	ldr r1, [r1]

gen_exc_loop:
	b gen_exc_loop @ eternal loop
@cfsr_addr:
@	.word 0xe000ed28 @ CFSR

.thumb_func
dbg_exc:
	b dbg_exc @ eternal loop

@ Start-up
.thumb_func
_start:
	b main

.end
