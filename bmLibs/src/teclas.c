/*
 * teclas.c
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */


#include "teclas.h"
#include "cmsis_43xx.h"


typedef struct {
	uint8_t port;
	uint8_t pin;
	uint8_t gpio;
	uint8_t bit;
	uint16_t mode;
} tecla_io_port_t;

static const tecla_io_port_t gpioTECLAPins[] = {
	{ 1,  0,  0,  4, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0)},
	{ 1,  1,  0,  8, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0)},
	{ 1,  2,  0,  9, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0)},
	{ 1,  6,  1,  9, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0)}};


void initTeclas (void)
{
 	int idx;

 	/* Initialize GPIO block */
 	Chip_GPIO_Init (LPC_GPIO_PORT);

	for (idx = 0; idx < (sizeof (gpioTECLAPins) / sizeof (tecla_io_port_t)); ++idx)
		initTecla(idx);
}


void initTecla (const tecla_t teclaNumber)
{
 	/* Initialize GPIO block */
 	Chip_GPIO_Init (LPC_GPIO_PORT);

	/* Configure GPIO pin as input */
	Chip_GPIO_SetPinDIRInput (LPC_GPIO_PORT, gpioTECLAPins[teclaNumber].gpio, gpioTECLAPins[teclaNumber].bit);

	/* Set pin to GPIO */
	Chip_SCU_PinMuxSet (gpioTECLAPins[teclaNumber].port, gpioTECLAPins[teclaNumber].pin, gpioTECLAPins[teclaNumber].mode);
}


/* reurn => 1 : presionado / 0 : no presionado  */
uint8_t readTecla (const tecla_t teclaNumber)
{
	return (0x01 & (~Chip_GPIO_GetPinState (LPC_GPIO_PORT, gpioTECLAPins[teclaNumber].gpio, gpioTECLAPins[teclaNumber].bit)));
}


void testTecla (const tecla_t teclaNumber)
{
	readTecla (teclaNumber);
}
