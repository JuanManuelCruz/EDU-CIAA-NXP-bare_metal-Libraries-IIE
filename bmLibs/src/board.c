/*
 * board.c
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */


#include "board.h"
#include "delays.h"


void initBoard(void)
{
	// Setup Clock at 1/4
	Chip_SetupCoreClock(CLKIN_IRC, MAX_CLOCK_FREQ/4, true);

	// Setup SysTick Timer to interrupt at 1 msec intervals
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/(1000));

	initLeds();
	initRgbs();
	initTeclas();
	initGpios();
	Board_UART_Init(LPC_UARTX);

}

