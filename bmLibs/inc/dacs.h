/*
 * dacs.h
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */

#ifndef DACS_H_
#define DACS_H_


#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#include "cmsis_43xx.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif

/*	ADC_CH0 => DAC				*
 * 	ADC_CH1 => ADC0_1 & ADC1_1	*
 *	ADC_CH2 => ADC0_2 & ADC1_2	*
 *	ADC_CH3 => ADC0_3 & ADC1_3	*/

// DACs

#define DAC_VALUE_SIZE		0x400

void initDac (void);
void setDacValue (uint16_t dacValue);


#endif /* DACS_H_ */
