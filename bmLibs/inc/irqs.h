/*
 * irqs.h
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */

#ifndef TECLAS_H_
#define TECLAS_H_


#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#include "cmsis_43xx.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif


#define debounce_t	uint32_t

// GPIO IRQs
#define GPIOIRQ_0	0
#define GPIOIRQ_1	1
#define GPIOIRQ_2	2
#define GPIOIRQ_3	3
#define GPIOIRQ_4	4
#define GPIOIRQ_5	5
#define GPIOIRQ_6	6
#define GPIOIRQ_7	7

#define irq_t	uint8_t
static const irq_t irq[] = {
	GPIOIRQ_0,  GPIOIRQ_1,  GPIOIRQ_2,  GPIOIRQ_3,
	GPIOIRQ_4,  GPIOIRQ_5,  GPIOIRQ_6,  GPIOIRQ_7};

// TECLASs
#define TECLA1_IRQ	0
#define TECLA2_IRQ	1
#define TECLA3_IRQ	2
#define TECLA4_IRQ	3
// GPIOs
#define GPIO0_IRQ	4
#define GPIO1_IRQ	5
#define GPIO2_IRQ	6
#define GPIO3_IRQ	7
#define GPIO4_IRQ	8
#define GPIO5_IRQ	9
#define GPIO6_IRQ	10
#define GPIO7_IRQ	11
#define GPIO8_IRQ	12

#define irqPin_t	uint8_t
static const irqPin_t irqPin[] = {
	TECLA1_IRQ, TECLA2_IRQ, TECLA3_IRQ, TECLA4_IRQ,
	 GPIO0_IRQ,  GPIO1_IRQ,  GPIO2_IRQ,  GPIO3_IRQ,
	 GPIO4_IRQ,  GPIO5_IRQ,  GPIO6_IRQ,  GPIO7_IRQ,
	 GPIO8_IRQ};


void configIrq (const irq_t irqNumber, const irqPin_t irqPinNumber, void (*isr) (void), debounce_t debounce);


void GPIO0_IRQHandler (void);
void GPIO1_IRQHandler (void);
void GPIO2_IRQHandler (void);
void GPIO3_IRQHandler (void);
void GPIO4_IRQHandler (void);
void GPIO5_IRQHandler (void);
void GPIO6_IRQHandler (void);
void GPIO7_IRQHandler (void);


#endif /* TECLAS_H_ */
