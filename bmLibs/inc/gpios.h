/*
 * gpios.h
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */

#ifndef GPIOS_H_
#define GPIOS_H_


#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif

/*	GPIO0 => P6_01 en GPIO 3[ 0] *
 *  GPIO1 => P6_04 en GPIO 3[ 3] *
 *  GPIO2 => P6_05 en GPIO 3[ 4] *
 *  GPIO3 => P6_07 en GPIO 5[15] *
 *  GPIO4 => P6_08 en GPIO 5[16] *
 *  GPIO5 => P6_09 en GPIO 3[ 5] *
 *  GPIO6 => P6_10 en GPIO 3[ 6] *
 *  GPIO7 => P6_11 en GPIO 3[ 7] *
 *	GPIO8 => P6_12 en GPIO 2[ 8] */

// GPIOs
#define GPIO0	0
#define GPIO1	1
#define GPIO2	2
#define GPIO3	3
#define GPIO4	4
#define GPIO5	5
#define GPIO6	6
#define GPIO7	7
#define GPIO8	8

#define gpio_t	uint8_t
static const gpio_t gpio[] = {GPIO0, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO7, GPIO8};

#define	OUTPUT	0
#define	INPUT	1

typedef struct {
	uint8_t port;
	uint8_t pin;
	uint8_t gpio;
	uint8_t bit;
	uint16_t mode;
} gpio_io_port_t;


static const gpio_io_port_t gpioGPIOPins[] = {
	{ 6,  1,  3,  0, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 6,  4,  3,  3, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 6,  5,  3,  4, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 6,  7,  5, 15, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC4)},
	{ 6,  8,  5, 16, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC4)},
	{ 6,  9,  3,  5, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 6, 10,  3,  6, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 6, 11,  3,  7, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)},
	{ 6, 12,  2,  8, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0)}};


void initGpios (void);
void initGpio (const gpio_t gpioNumber, uint8_t gpioDirection);
void setGpio (const gpio_t gpioNumber);
void clearGpio (const gpio_t gpioNumber);
void toggleGpio (const gpio_t gpioNumber);

uint8_t readGpio (const gpio_t gpioNumber);


#endif /* GPIOS_H_ */
