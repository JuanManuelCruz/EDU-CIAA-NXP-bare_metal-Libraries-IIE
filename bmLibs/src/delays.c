/*
 * delays.c
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */


#include "delays.h"


void delay_ms(uint32_t msec)
{
	uint32_t i;

	while (msec--){
		for (i=0;i<4400;i++){
			asm  ("nop");
		}
	}
}


void debounce_ms (uint32_t msec)
{
	uint32_t i;

	while (msec--){
		for (i=0; i < 4400; i++){
			asm  ("nop");
		}
	}
}
