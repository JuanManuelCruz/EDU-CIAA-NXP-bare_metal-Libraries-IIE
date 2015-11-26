/*
 * board.h
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */

#ifndef BOARD_H_
#define BOARD_H_

#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif

#include "irqs.h"
#include "leds.h"
#include "rgbs.h"
#include "teclas.h"
#include "gpios.h"
#include "pwms.h"
#include "uarts.h"
#include "delays.h"


void initBoard (void);
//void initADC (void);


#endif /* BOARD_H_ */
