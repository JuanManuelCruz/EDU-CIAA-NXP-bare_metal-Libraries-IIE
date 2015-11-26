/*
 * delays.h
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */

#ifndef DELAYS_H_
#define DELAYS_H_

#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif


void delay_ms (uint32_t);
void debounce_ms (uint32_t);


#endif /* DELAYS_H_ */
