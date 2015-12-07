/*
 * gpios.c
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */


#include "gpios.h"


void initGpios (void)
{
 	int idx;

 	/* Initialize GPIO block */
 	Chip_GPIO_Init (LPC_GPIO_PORT);

	for (idx = 0; idx < (sizeof (gpioGPIOPins) / sizeof (gpio_io_port_t)); ++idx) {
		initGpio (idx, INPUT);

		/* Set pin to GPIO */
		Chip_SCU_PinMuxSet (gpioGPIOPins[idx].port, gpioGPIOPins[idx].pin, gpioGPIOPins[idx].mode);
	}
}


void initGpio (const gpio_t gpioNumber, uint8_t gpioDirection)
{
 	/* Initialize GPIO block */
 	Chip_GPIO_Init (LPC_GPIO_PORT);

	if (gpioDirection == INPUT)
		/* Configure GPIO pin as input */
		Chip_GPIO_SetPinDIRInput (LPC_GPIO_PORT, gpioGPIOPins[gpioNumber].gpio, gpioGPIOPins[gpioNumber].bit);
	else
		/* Configure GPIO pin as output */
		Chip_GPIO_SetPinDIROutput (LPC_GPIO_PORT, gpioGPIOPins[gpioNumber].gpio, gpioGPIOPins[gpioNumber].bit);

	/* Set pin to GPIO */
	Chip_SCU_PinMuxSet (gpioGPIOPins[gpioNumber].port, gpioGPIOPins[gpioNumber].pin, gpioGPIOPins[gpioNumber].mode);
}


void setGpio (const gpio_t gpioNumber)
{
	if (gpioNumber < (sizeof (gpio) / sizeof (gpio_t)))
		/* Set pin */
		Chip_GPIO_SetPinState (LPC_GPIO_PORT, gpioGPIOPins[gpioNumber].gpio, gpioGPIOPins[gpioNumber].bit, (bool) true);
}

void clearGpio (const gpio_t gpioNumber)
{
	if (gpioNumber < (sizeof (gpio) / sizeof (gpio_t)))
		/* Clear pin */
		Chip_GPIO_SetPinState (LPC_GPIO_PORT, gpioGPIOPins[gpioNumber].gpio, gpioGPIOPins[gpioNumber].bit, (bool) false);
}


void toggleGpio (const gpio_t gpioNumber)
{
	if (gpioNumber < (sizeof (gpio) / sizeof (gpio_t)))
		/* Toggle pin */
		Chip_GPIO_SetPinToggle (LPC_GPIO_PORT, gpioGPIOPins[gpioNumber].gpio, gpioGPIOPins[gpioNumber].bit);
}


/* reurn => 1 : presionado / 0 : no presionado  */
uint8_t readGpio (const gpio_t gpioNumber)
{
	return (0x01 & (~Chip_GPIO_GetPinState (LPC_GPIO_PORT, gpioGPIOPins[gpioNumber].gpio, gpioGPIOPins[gpioNumber].bit)));
}
