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
#include "cmsis_43xx.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif


#include "irqs.h"
#include "leds.h"
#include "rgbs.h"
#include "gpios.h"
#include "rits.h"
#include "pwms.h"
#include "uarts.h"
#include "adcs.h"
#include "dacs.h"
#include "delays.h"
#include "teclas.h"


void initBoard (void);


#endif /* BOARD_H_ */
