/*
 * irqs.c
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */


#include "irqs.h"
#include "cmsis_43xx.h"
#include "delays.h"


void pinStatus (irq_t irqNumber);

typedef struct {
	uint8_t port;
	uint8_t pin;
	uint8_t gpio;
	uint8_t bit;
	uint16_t mode;
} irq_io_port_t;

static const irq_io_port_t irqPins[] = {
	{ 1,  0,  0,  4, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 1,  1,  0,  8, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 1,  2,  0,  9, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 1,  6,  1,  9, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 6,  1,  3,  0, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 6,  4,  3,  3, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 6,  5,  3,  4, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 6,  7,  5, 15, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC4)},
	{ 6,  8,  5, 16, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC4)},
	{ 6,  9,  3,  5, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 6, 10,  3,  6, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 6, 11,  3,  7, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 6, 12,  2,  8, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)}};

typedef struct {
	uint8_t index;
	uint8_t irqn;
	debounce_t debounce;
	void (*funcion) (irq_t);
	void (*hook) (void);
	uint8_t port;
	uint8_t pin;
	uint8_t gpio;
	uint8_t bit;
	uint16_t mode;
} irq_config_t;

void pin_IRQHandler_Debounce (irqPin_t tecla);
void GPIOX_IRQHandler_Hook (void);

static volatile irq_config_t irqConfig[] = {
	{GPIOIRQ_0, PIN_INT0_IRQn, 0, pin_IRQHandler_Debounce, GPIOX_IRQHandler_Hook, 0, 0, 0, 0, 0},
	{GPIOIRQ_1, PIN_INT1_IRQn, 0, pin_IRQHandler_Debounce, GPIOX_IRQHandler_Hook, 0, 0, 0, 0, 0},
	{GPIOIRQ_2, PIN_INT2_IRQn, 0, pin_IRQHandler_Debounce, GPIOX_IRQHandler_Hook, 0, 0, 0, 0, 0},
	{GPIOIRQ_3, PIN_INT3_IRQn, 0, pin_IRQHandler_Debounce, GPIOX_IRQHandler_Hook, 0, 0, 0, 0, 0},
	{GPIOIRQ_4, PIN_INT6_IRQn, 0, pin_IRQHandler_Debounce, GPIOX_IRQHandler_Hook, 0, 0, 0, 0, 0},
	{GPIOIRQ_5, PIN_INT5_IRQn, 0, pin_IRQHandler_Debounce, GPIOX_IRQHandler_Hook, 0, 0, 0, 0, 0},
	{GPIOIRQ_6, PIN_INT4_IRQn, 0, pin_IRQHandler_Debounce, GPIOX_IRQHandler_Hook, 0, 0, 0, 0, 0},
	{GPIOIRQ_7, PIN_INT7_IRQn, 0, pin_IRQHandler_Debounce, GPIOX_IRQHandler_Hook, 0, 0, 0, 0, 0}};


void configIrq (const irq_t irqNumber, const irqPin_t irqPinNumber, void (*isr)(void), debounce_t debounce)
{
	if (irqPinNumber < (sizeof (irqPin) / sizeof (irqPin_t))) {
		irqConfig[irqNumber].port = irqPins[irqPinNumber].port;
		irqConfig[irqNumber].pin = irqPins[irqPinNumber].pin;
		irqConfig[irqNumber].gpio = irqPins[irqPinNumber].gpio;
		irqConfig[irqNumber].bit = irqPins[irqPinNumber].bit;
		irqConfig[irqNumber].mode = irqPins[irqPinNumber].mode;
		irqConfig[irqNumber].debounce = debounce;
		irqConfig[irqNumber].funcion = pinStatus;
		irqConfig[irqNumber].hook = isr;

		/* Initialize GPIO block */
		Chip_GPIO_Init (LPC_GPIO_PORT);

		/* Configure GPIO pin as input */
		Chip_GPIO_SetPinDIRInput (LPC_GPIO_PORT, irqConfig[irqNumber].gpio, irqConfig[irqNumber].bit);

		/* Set pin to GPIO */
		Chip_SCU_PinMuxSet (irqConfig[irqNumber].port, irqConfig[irqNumber].pin, irqConfig[irqNumber].mode);
		/* Configure interrupt channel for the GPIO pin in SysCon block */
		Chip_SCU_GPIOIntPinSel (irqConfig[irqNumber].index, irqConfig[irqNumber].gpio, irqConfig[irqNumber].bit);

		/* Configure channel interrupt as edge sensitive and falling edge interrupt */
		Chip_PININT_ClearIntStatus (LPC_GPIO_PIN_INT, PININTCH (irqConfig[irqNumber].index));
		Chip_PININT_SetPinModeEdge (LPC_GPIO_PIN_INT, PININTCH (irqConfig[irqNumber].index));
		Chip_PININT_EnableIntLow (LPC_GPIO_PIN_INT, PININTCH (irqConfig[irqNumber].index));


		/* Enable interrupt in the NVIC */
		NVIC_ClearPendingIRQ (irqConfig[irqNumber].irqn);
		NVIC_EnableIRQ (irqConfig[irqNumber].irqn);
	}
}


/**
 * @brief	Handle interrupt from GPIO pin or GPIO pin mapped to PININT
 * @return	Nothing
 */
//__attribute__ ((section(".after_vectors")))
void GPIO0_IRQHandler (void)
{
	Chip_PININT_ClearIntStatus (LPC_GPIO_PIN_INT, PININTCH (GPIOIRQ_0));

	irqConfig[GPIOIRQ_0].funcion (GPIOIRQ_0);
}


//__attribute__ ((section(".after_vectors")))
void GPIO1_IRQHandler (void)
{
	Chip_PININT_ClearIntStatus (LPC_GPIO_PIN_INT, PININTCH (GPIOIRQ_1));

	irqConfig[GPIOIRQ_1].funcion (GPIOIRQ_1);
}


//__attribute__ ((section(".after_vectors")))
void GPIO2_IRQHandler (void)
{
	Chip_PININT_ClearIntStatus (LPC_GPIO_PIN_INT, PININTCH (GPIOIRQ_2));

	irqConfig[GPIOIRQ_2].funcion (GPIOIRQ_2);
}


//__attribute__ ((section(".after_vectors")))
void GPIO3_IRQHandler (void)
{
	Chip_PININT_ClearIntStatus (LPC_GPIO_PIN_INT, PININTCH (GPIOIRQ_3));

	irqConfig[GPIOIRQ_3].funcion (GPIOIRQ_3);
}


//__attribute__ ((section(".after_vectors")))
void GPIO4_IRQHandler (void)
{
	Chip_PININT_ClearIntStatus (LPC_GPIO_PIN_INT, PININTCH (GPIOIRQ_4));

	irqConfig[GPIOIRQ_4].funcion (GPIOIRQ_4);
}


//__attribute__ ((section(".after_vectors")))
void GPIO5_IRQHandler (void)
{
	Chip_PININT_ClearIntStatus (LPC_GPIO_PIN_INT, PININTCH (GPIOIRQ_5));

	irqConfig[GPIOIRQ_5].funcion (GPIOIRQ_5);
}


//__attribute__ ((section(".after_vectors")))
void GPIO6_IRQHandler (void)
{
	Chip_PININT_ClearIntStatus (LPC_GPIO_PIN_INT, PININTCH (GPIOIRQ_6));

	irqConfig[GPIOIRQ_6].funcion (GPIOIRQ_6);
}


//__attribute__ ((section(".after_vectors")))
void GPIO7_IRQHandler (void)
{
	Chip_PININT_ClearIntStatus (LPC_GPIO_PIN_INT, PININTCH (GPIOIRQ_7));

	irqConfig[GPIOIRQ_7].funcion (GPIOIRQ_7);
}


void GPIOX_IRQHandler_Hook(void)
{

}


void pin_IRQHandler_Debounce (irq_t irq)
{
	//Silencio el warning de parametros no usados
	irq = (uint8_t)irq;
}


void pinStatus (irq_t irqNumber)
{
	debounce_ms (irqConfig[irqNumber].debounce);

	if ((0x01 & (~Chip_GPIO_GetPinState (LPC_GPIO_PORT, irqConfig[irqNumber].gpio, irqConfig[irqNumber].bit))) == 1)
		irqConfig[irqNumber].hook();
}
