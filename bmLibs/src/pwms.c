 /*
 * pwms.c
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */

#include "pwms.h"
#include "delays.h"


typedef struct
{
    uint8_t prescaler;
    uint8_t mode;
    void (*isr)(void);   // interrupt service routine
} PWM_t;


PWM_t PWM;

typedef struct {
	uint8_t out;
	uint8_t channel;
} pwm_o_channel_t;

static const pwm_o_channel_t pwm_o_channel[] = {{2, 1}, {5, 2}, {4, 3}, {6, 4}, {7, 5}};


static uint32_t inicializado = 0;


void configPWM(uint32_t periodo_us, const pwm_t pwmNumber)
{
	if (pwmNumber < (sizeof (pwm) / sizeof (pwm_t))) {

		if(inicializado == 0) {
			Chip_SCTPWM_Init(LPC_SCT);
			Chip_SCTPWM_SetRate(LPC_SCT, (uint32_t)(1000000.0/((float)periodo_us)));
			Chip_SCTPWM_Start(LPC_SCT);

			inicializado = 1;
		}
		switch (pwmNumber) {
			case 0:
			case 1:
			case 2:
				Chip_SCU_PinMuxSet(gpioLEDPins[pwm[pwmNumber]].port, gpioLEDPins[pwm[pwmNumber]].pin, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC1));
			break;
			case 3:
				Chip_SCU_PinMuxSet(gpioGPIOPins[GPIO2].port, gpioGPIOPins[GPIO2].pin, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC1));
				break;
			case 4:
				Chip_SCU_PinMuxSet(gpioGPIOPins[GPIO8].port, gpioGPIOPins[GPIO8].pin, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC1));
				break;
		}
		Chip_SCTPWM_SetOutPin(LPC_SCT, pwm_o_channel[pwmNumber].channel, pwm_o_channel[pwmNumber].out);

		Chip_SCTPWM_SetDutyCycle(LPC_SCT, pwm_o_channel[pwmNumber].channel, 0);
	}
}


void startPWM(void)
{
	Chip_SCTPWM_Start(LPC_SCT);
}


void stopPWM(void)
{
	Chip_SCTPWM_Stop(LPC_SCT);
}


void setPWM(uint8_t duty, const pwm_t pwmNumber)
{
	if (pwmNumber < (sizeof (pwm) / sizeof (pwm_t)))
		Chip_SCTPWM_SetDutyCycle(LPC_SCT, pwm_o_channel[pwmNumber].channel, (Chip_SCTPWM_GetTicksPerCycle(LPC_SCT) * duty) / 255);
}

