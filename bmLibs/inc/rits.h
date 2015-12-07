 /*
 *  pwms.h
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */

#ifndef RITS_H_
#define RITS_H_


#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#include "cmsis_43xx.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif


void configRit (uint16_t timeInterval_ms, void (*isr)(void));


#endif /* RITS_H_ */
