/*
 * dacs.c
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */


#include "dacs.h"


void setDacValue (uint16_t dacValue)
{
	Chip_DAC_UpdateValue (LPC_DAC, dacValue);

	while (!(Chip_DAC_GetIntStatus (LPC_DAC)));	 					/*	0 <=> 0V & 1024 <=> 3.3V	*/
}


void initDac (void)
{
	Chip_SCU_DAC_Analog_Config();

	/* DAC Init */
	Chip_DAC_Init (LPC_DAC);
	Chip_DAC_SetDMATimeOut (LPC_DAC, 0xFFFF);
	Chip_DAC_ConfigDAConverterControl (LPC_DAC, (DAC_CNT_ENA | DAC_DMA_ENA));
	setDacValue (0);				 								/*	0 <=> 0V & 1024 <=> 3.3V	*/

}
