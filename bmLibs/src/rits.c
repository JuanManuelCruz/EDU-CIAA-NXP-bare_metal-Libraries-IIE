/*
 * rits.c
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */


#include "rits.h"


typedef struct {
	uint16_t time;
	void (*hook) (void);
} rit_config_t;

void RIT_IRQHandler_Hook (void);

static volatile rit_config_t ritConfig = {1000, RIT_IRQHandler_Hook};


void configRit (uint16_t timeInterval_ms, void (*isr)(void))
{
	ritConfig.time = timeInterval_ms;
	ritConfig.hook = isr;

	/* Initialize RITimer */
	Chip_RIT_Init(LPC_RITIMER);

	/* Configure RIT for a 1s interrupt tick rate */
	Chip_RIT_SetTimerInterval(LPC_RITIMER, ritConfig.time);

	NVIC_EnableIRQ(RITIMER_IRQn);
}


//__attribute__ ((section(".after_vectors")))
void RIT_IRQHandler (void)
{
	/* Clearn interrupt */
	Chip_RIT_ClearInt (LPC_RITIMER);

	/* Toggle LED */
	ritConfig.hook ();
}


void RIT_IRQHandler_Hook (void)
{

}

