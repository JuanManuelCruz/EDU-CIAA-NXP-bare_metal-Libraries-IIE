/*
 * adcs.c
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */


#include "adcs.h"


#define ADC0	0
#define ADC1	1

#define ADC_ID 		ADC0
#define _LPC_ADC_ID LPC_ADC0
//#define _LPC_ADC_ID LPC_ADC1

static ADC_CLOCK_SETUP_T ADCSetup;


void initAdcs (void)
{
 	int idx;

	for (idx = 1; idx < (sizeof (adc) / sizeof (adc_t)); ++idx)
		initAdc (adc[idx]);
}


void initAdc (const adc_t adcNumber)
{
	ADCSetup.adcRate = ADC_MAX_SAMPLE_RATE;
	ADCSetup.bitsAccuracy = ADC_10BITS;
	ADCSetup.burstMode = false;

	if ((adcNumber > 0) && (adcNumber < (sizeof (adc) / sizeof (adc_t)))) {

		/*ADC Init */
		Chip_SCU_ADC_Channel_Config(ADC_ID, adc[adcNumber]);				//	Channerl on ADC0
		Chip_ADC_Init(_LPC_ADC_ID, &ADCSetup);
		Chip_ADC_EnableChannel(_LPC_ADC_ID, adc[adcNumber], ENABLE);
	}
}


uint16_t readAdc (const adc_t adcNumber)
{
	uint16_t dataADC = 0;

	if ((adcNumber > 0) && (adcNumber < (sizeof (adc) / sizeof (adc_t)))) {
		/* Select using burst mode or not */
		if (ADCSetup.burstMode)
			Chip_ADC_SetBurstCmd (_LPC_ADC_ID, ENABLE);
		else
			Chip_ADC_SetBurstCmd (_LPC_ADC_ID, DISABLE);

		/* Start A/D conversion if not using burst mode */
		if (!ADCSetup.burstMode)
			Chip_ADC_SetStartMode (_LPC_ADC_ID, ADC_START_NOW, ADC_TRIGGERMODE_RISING);

		/* Waiting for A/D conversion complete */
		while (Chip_ADC_ReadStatus (_LPC_ADC_ID, adc[adcNumber], ADC_DR_DONE_STAT) != SET);

		/* Read ADC value */
		Chip_ADC_ReadValue (_LPC_ADC_ID, adc[adcNumber], &dataADC);

		/* Disable burst mode, if any */
		if (ADCSetup.burstMode)
			Chip_ADC_SetBurstCmd (_LPC_ADC_ID, DISABLE);
	}

	return (dataADC);
}

