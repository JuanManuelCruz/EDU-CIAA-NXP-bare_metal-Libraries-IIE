/*
 * leds.h
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */

#ifndef LEDS_H_
#define LEDS_H_


#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif

/*	LED1 => P2_10 en GPIO 0[14]	*
 *	LED2 => P2_11 en GPIO 1[11]	*
 *	LED3 => P2_12 en GPIO 1[12]	*/

// LEDs
#define LED1	0
#define LED2	1
#define LED3	2

#define led_t	uint8_t
static const led_t led[] = {LED1, LED2, LED3};

typedef struct {
	uint8_t port;
	uint8_t pin;
	uint8_t gpio;
	uint8_t bit;
	uint16_t mode;
} led_io_port_t;

static const led_io_port_t gpioLEDPins[] = {
	{ 2, 10,  0, 14, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC0)},
	{ 2, 11,  1, 11, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC0)},
	{ 2, 12,  1, 12, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC0)}};

void initLeds (void);
void initLed (const led_t ledNumber);
void setLed (const led_t ledNumber);
void clearLed (const led_t ledNumber);
void toggleLed (const led_t ledNumber);


#endif /* LEDS_H_ */
