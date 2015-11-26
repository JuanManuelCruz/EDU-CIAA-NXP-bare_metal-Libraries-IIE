/*
 * rgbs.c
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */

#include "rgbs.h"


typedef struct {
	uint8_t port;
	uint8_t pin;
	uint8_t gpio;
	uint8_t bit;
	uint16_t mode;
} rgb_io_port_t;

static const rgb_io_port_t gpioRGBPins[] = {
	{ 2,  1,  5,  1, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC4)},
	{ 2,  0,  5,  0, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC4)},
	{ 2,  2,  5,  2, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC4)}};


void initRgbs(void)
{
 	int idx;

 	/* Initialize GPIO block */
 	Chip_GPIO_Init (LPC_GPIO_PORT);

	for (idx = 0; idx < (sizeof (gpioRGBPins) / sizeof (rgb_io_port_t)); ++idx) {
		/* Configure GPIO pin as output */
		Chip_GPIO_SetPinDIROutput (LPC_GPIO_PORT, gpioRGBPins[idx].gpio, gpioRGBPins[idx].bit);
		/* Set pin to GPIO */
		Chip_SCU_PinMuxSet (gpioRGBPins[idx].port, gpioRGBPins[idx].pin, gpioRGBPins[idx].mode);

		/* Set pin */
		Chip_GPIO_SetPinState (LPC_GPIO_PORT, gpioRGBPins[idx].gpio, gpioRGBPins[idx].bit, (bool) true);
		/* Clear pin */
		Chip_GPIO_SetPinState (LPC_GPIO_PORT, gpioRGBPins[idx].gpio, gpioRGBPins[idx].bit, (bool) false);
	}
}


void initRgb (const rgb_t rgbNumber)
{
	if (rgbNumber < (sizeof (rgb) / sizeof (rgb_t))) {
		/* Initialize GPIO block */
		Chip_GPIO_Init (LPC_GPIO_PORT);

		/* Configure GPIO pin as output */
		Chip_GPIO_SetPinDIROutput (LPC_GPIO_PORT, gpioRGBPins[rgbNumber].gpio, gpioRGBPins[rgbNumber].bit);
		/* Set pin to GPIO */
		Chip_SCU_PinMuxSet (gpioRGBPins[rgbNumber].port, gpioRGBPins[rgbNumber].pin, gpioRGBPins[rgbNumber].mode);

		/* Set pin */
		Chip_GPIO_SetPinState (LPC_GPIO_PORT, gpioRGBPins[rgbNumber].gpio, gpioRGBPins[rgbNumber].bit, (bool) true);
		/* Clear pin */
		Chip_GPIO_SetPinState (LPC_GPIO_PORT, gpioRGBPins[rgbNumber].gpio, gpioRGBPins[rgbNumber].bit, (bool) false);

	}
}


void setRgb (const rgb_t rgbNumber)
{
	if (rgbNumber < (sizeof (rgb) / sizeof (rgb_t)))
		/* Set pin */
		Chip_GPIO_SetPinState (LPC_GPIO_PORT, gpioRGBPins[rgbNumber].gpio, gpioRGBPins[rgbNumber].bit, (bool) true);
}


void clearRgb (const rgb_t rgbNumber)
{
	if (rgbNumber < (sizeof (rgb) / sizeof (rgb_t)))
		/* Clear pin */
		Chip_GPIO_SetPinState (LPC_GPIO_PORT, gpioRGBPins[rgbNumber].gpio, gpioRGBPins[rgbNumber].bit, (bool) false);
}


void toggleRgb (const rgb_t rgbNumber)
{
	if (rgbNumber < (sizeof (rgb) / sizeof (rgb_t)))
		/* Toggle pin */
		Chip_GPIO_SetPinToggle (LPC_GPIO_PORT, gpioRGBPins[rgbNumber].gpio, gpioRGBPins[rgbNumber].bit);
}
