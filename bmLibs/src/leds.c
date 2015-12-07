/*
 * leds.c
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */


#include "leds.h"


void initLeds (void)
{
 	int idx;

	for (idx = 0; idx < (sizeof (gpioLEDPins) / sizeof (led_io_port_t)); ++idx)
		initLed (led[idx]);
}


void initLed (const led_t ledNumber)
{
	if (ledNumber < (sizeof (led) / sizeof (led_t))) {
		/* Initialize GPIO block */
		Chip_GPIO_Init (LPC_GPIO_PORT);

		/* Configure GPIO pin as output */
		Chip_GPIO_SetPinDIROutput (LPC_GPIO_PORT, gpioLEDPins[ledNumber].gpio, gpioLEDPins[ledNumber].bit);
		/* Set pin to GPIO */
		Chip_SCU_PinMuxSet (gpioLEDPins[ledNumber].port, gpioLEDPins[ledNumber].pin, gpioLEDPins[ledNumber].mode);

		/* Set pin */
		setLed (ledNumber);
		/* Clear pin */
		clearLed (ledNumber);
	}
}


void setLed (const led_t ledNumber)
{
	if (ledNumber < (sizeof (led) / sizeof (led_t)))
		/* Set pin */
		Chip_GPIO_SetPinState (LPC_GPIO_PORT, gpioLEDPins[ledNumber].gpio, gpioLEDPins[ledNumber].bit, (bool) true);
}


void clearLed (const led_t ledNumber)
{
	if (ledNumber < (sizeof (led) / sizeof (led_t)))
		/* Clear pin */
		Chip_GPIO_SetPinState (LPC_GPIO_PORT, gpioLEDPins[ledNumber].gpio, gpioLEDPins[ledNumber].bit, (bool) false);
}


void toggleLed (const led_t ledNumber)
{
	if (ledNumber < (sizeof (led) / sizeof (led_t)))
		/* Toggle pin */
		Chip_GPIO_SetPinToggle (LPC_GPIO_PORT, gpioLEDPins[ledNumber].gpio, gpioLEDPins[ledNumber].bit);
}


