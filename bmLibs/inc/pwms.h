 /*
 *  pwms.h
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */

#ifndef PWMS_H_
#define PWMS_H_


#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#include "cmsis_43xx.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif


#include "leds.h"
#include "gpios.h"


/*	LED1  => P2_10 en GPIO 0[14]	=> SCT = 2 *
 *	LED2  => P2_11 en GPIO 1[11]	=> SCT = 5 *
 *	LED3  => P2_12 en GPIO 1[12]	=> SCT = 4 *
 *	GPIO2 => P6_05 en GPIO 3[04]	=> SCT = 6 *
 *	GPIO8 => P6_12 en GPIO 2[08]	=> SCT = 7 */

#define LED1_PWM	0
#define LED2_PWM	1
#define LED3_PWM	2
#define GPIO2_PWM	3
#define GPIO8_PWM	4

#define pwm_t	uint8_t
static const pwm_t pwm[] = {LED1_PWM, LED2_PWM, LED3_PWM, GPIO2_PWM, GPIO8_PWM};

void configPWM(uint32_t periodo_us, const pwm_t pwmNumber);
void startPWM(void);
void stopPWM(void);
void setPWM(uint8_t duty, const led_t pwmNumber);

/*
SCT	PIN	Function
0	P4_2	TEC_F2
1	P2_7	ISP
1	P4_1	TEC_F1
2	P2_10	LED1
2	P4_4	LCD1
3	P4_3	TEC_F3
4	P2_12	LED3
4	P4_6	LCD3
5	P2_11	LED2
5	P4_5	LCD2
6	P1_2	TECLADO
6	P6_5	GPIO2
7	P1_1	TECLADO
7	P6_12	GPIO8
8	P1_3	SPI
8	P7_7	ENET_MDC
9	P1_4	SPI
10	P1_5	TEC_COL
11	P7_6	ISP
12	P7_5	TEC_COL2
13	P7_4	TEC_COL1
15	P7_1	U2_TXD
*/


#endif /* PWMS_H_ */
