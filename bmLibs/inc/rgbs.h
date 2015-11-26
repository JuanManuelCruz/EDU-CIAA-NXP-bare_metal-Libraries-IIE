/*
 * rgbs.h
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */

#ifndef RGBS_H_
#define RGBS_H_


#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif

/*	LED0G => P2_1 en GPIO 5[1]	*
 *  LED0R => P2_0 en GPIO 5[0]	*
 *	LED0B => P2_2 en GPIO 5[2]	*/

// RGBs
#define LED0G	0
#define LED0R	1
#define LED0B	2

#define rgb_t	uint8_t
static const rgb_t rgb[] = {LED0G, LED0R, LED0B};

void initRgbs (void);
void initRgb (const rgb_t rgb);
void setRgb (const rgb_t rgb);
void clearRgb (const rgb_t rgb);
void toggleRgb (const rgb_t rgb);


#endif /* RGBS_H_ */
