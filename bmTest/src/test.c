/* Copyright 2015, Juan Manuel Cruz
 * Copyright 2015, Sebastian Bedin
 * Copyright 2015, Alejandro Celery
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief test blinking example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * JMC			Juan Manuel Cruz
 * SB			Sebastian Bedin
 * AC			Alejandro Celery
 * PR			Pablo Ridolfi
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20151112 v0.0.1   JMC  first functional version
 */


/*
 * Initials     Name
 * ---------------------------
 *
 */

/*==================[inclusions]=============================================*/
#include "test.h"       /* <= own header */

#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif

#include "board.h"


/*==================[macros and definitions]=================================*/
#define TEST_RGBs	0		/*	Test RGB0G, RGB0R, RGB0B					*/
#define TEST_LEDs	1		/*	Test LED1, LED2, LED3						*/
#define TEST_PWMs	2		/*	Test PWM on LED1, LED2, LED3, GPIO2 & GPIO8	*/
#define TEST_UARTs	3		/*	Test UART3 Tx => Rx with PWM on LED1		*/
#define TEST_GPIOsA	4		/*	Test A - GPIO0, GPIO1, GPIO2, GPIO3, GPIO4,	*/
							/*		     GPIO5, GPIO6, GPIO7, GPIO8			*/
#define TEST_GPIOsB	5		/*	Test B - GPIO0, GPIO1, GPIO2, GPIO3, GPIO4,	*/
							/*	         GPIO5, GPIO6, GPIO7, GPIO8			*/
#define TEST_IRQs	6		/*	Test TECLA1, TECLA2, TECLA3, TECLA4	on IRQs	*/
							/*	Test  GPIO1,  GPIO3,  GPIO5,  GPIO7 on IRQs	*/

#define TEST	TEST_IRQs	/*	Type the mane of Test to do here 			*/

/*==================[internal data declaration]==============================*/
#if (TEST == TEST_IRQs)

static void	interrupcionTecla1 (void);
static void interrupcionTecla2 (void);
static void	interrupcionTecla3 (void);
static void	interrupcionTecla4 (void);
static void	interrupcionGpio1 (void);
static void	interrupcionGpio3 (void);
static void	interrupcionGpio5 (void);
static void	interrupcionGpio7 (void);

#endif

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
 /** \brief Main function
  *
  * This is the main entry point of the software.
  *
  * \returns 0
  *
  * \remarks This function never returns. Return value is only to avoid compiler
  *          warnings or errors.
  */

#if (TEST == TEST_RGBs)
	/*	Test RGB0G, RGB0R, RGB0B					*/
int main (void)
{
	/* perform the needed initialization here */
 	uint8_t 	idx;
 	uint32_t	delay;

 	/* add your code here */
 	initBoard();

 	delay = 250;									/*	delay = 250mS			*/

 	while(1) {

	 	for (idx = 0; idx < (sizeof (rgb) / sizeof (rgb_t)); ++idx) {
	 	 	initRgb (rgb[idx]);

	 	 	setRgb (rgb[idx]);
	 	 	delay_ms (delay);

	 	 	clearRgb (rgb[idx]);
	 	 	delay_ms (delay);

	 	 	toggleRgb (rgb[idx]);
	 	 	delay_ms (delay);

	 	 	toggleRgb (rgb[idx]);
	 	 	delay_ms (delay);
	 	}
 	}

	return 0;
}
#endif


 #if (TEST == TEST_LEDs)
	/*	Test LED1, LED2, LED3						*/
int main (void)
{
	/* perform the needed initialization here */
 	uint8_t 	idx;
 	uint32_t	delay;

 	/* add your code here */
 	initBoard();

 	delay = 250;									/*	delay = 250mS			*/

 	while(1) {

	 	for (idx = 0; idx < (sizeof (led) / sizeof (led_t)); ++idx) {
	 	 	initLed (led[idx]);

	 	 	setLed (led[idx]);
	 	 	delay_ms (delay);

	 	 	clearLed (led[idx]);
	 	 	delay_ms (delay);

	 	 	toggleLed (led[idx]);
	 	 	delay_ms (delay);

	 	 	toggleLed (led[idx]);
	 	 	delay_ms (delay);
	 	}
 	}

	return 0;
}

#endif


#if (TEST == TEST_PWMs)
int main (void)
{
 	/*	Test PWM on LED1, LED2, LED3, GPIO2 & GPIO8; T = 10uS						*/
	/*	Place ORC passive probes on GPIO2 & GPIO8 pins of EDU-CIAA-NXP P2 connector	*/

	/* perform the needed initialization here */
 	uint8_t 	idx, duty;
 	uint32_t	period, delay;

 	/* add your code here */
 	initBoard();

 	period = 1;											/*	period = 1uS to 1000000uS	*/
 	delay = 50;											/*	delay = 50mS				*/

 	while(1) {
		for (idx = 0; idx < (sizeof (pwm) / sizeof (pwm_t)); ++idx) {
			configPWM (period, pwm[idx]);				/*	PWM T = 1uS	*/
			startPWM ();								/*	Start PWM	*/
			for ( duty = 0; duty < 255; duty +=5 ) {	/*	PWM duty = 0 -> 255, duty += 5 & delay = 50mS	*/
				setPWM (duty, pwm[idx]);
				delay_ms (delay);
			}

			for ( duty = 255; duty > 0; duty -=5 ) {	/*	PWM duty = 255 -> 0, duty -= 5 & delay = 50mS	*/
				setPWM (duty, pwm[idx]);
				delay_ms (delay);
			}
			stopPWM();									/*	Stop PWM	*/
		}
	}

 	return 0;
}

#endif


#if (TEST == TEST_UARTs)
	/*	Test UART3 Tx => Rx with PWM on LED1									*/
	/*	Place jumper between 232_TX & 232_RX pins of EDU-CIAA-NXP P1 connector	*/
int main (void)
{
 	/* perform the needed initialization here */
	uint8_t		idx, duty, data;
	uint32_t	period, delay;

 	/* add your code here */
 	initBoard();

 	idx = 0;										/*	PWM on LED1					*/
 	period = 10;									/*	period = 1uS to 1000000uS	*/
 	delay = 50;										/*	delay = 50mS				*/

 	configPWM (period, pwm[idx]);					/*	PWM T = period on ledPWM	*/
 	configUart (9600);								/*	UART baudrate = 9600		*/

	while(1) {
		startPWM();									/*	Start PWM					*/

		for (data = 0; data < 255; data += 5) {
			setUartTx (data);						/*	Tx = data					*/
			delay_ms (delay);
			duty = getUartData ();					/*	duty = Rx					*/
			setPWM (duty, pwm[idx]);
		}
		for (data = 255; data > 0; data -= 5) {
			setUartTx (data);						/*	Tx = data					*/
			delay_ms (delay);
			duty = getUartData ();					/*	duty = Rx					*/
			setPWM (duty,pwm[idx]);
		}

		stopPWM();									/*	Stop PWM					*/
	}

	return 0;
}

#endif


#if (TEST == TEST_GPIOsA)
	/*	Test GPIO0, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO7, GPIO8	*/
	/*	all GPIO´s => OUTPUT							*/
	/*	Place ORC passive probes on GPIOX pins of EDU-CIAA-NXP P2 connector	*/

int main (void)
{
 	/* perform the needed initialization here */
	uint8_t		idx;
 	uint32_t	delay;

 	/* add your code here */
 	initBoard();

 	delay = 10;											/*	delay = 10mS				*/
	for (idx = 0; idx < (sizeof (gpio) / sizeof (gpio_t)); ++idx)
			initGpio (gpio[idx], OUTPUT);				/*	Init all GPIO´s => OUTPUT	*/

	while(1) {

		for (idx = 0; idx < (sizeof (gpio) / sizeof (gpio_t)); ++idx) {
			toggleGpio(gpio[idx]);
			delay_ms (delay);
		}
	}

	return 0;
}

#endif


#if (TEST == TEST_GPIOsB)
	/*	Test GPIO0, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO7, GPIO8		*/
	/*	Place ORC passive probes on  GPIO8 pin  of EDU-CIAA-NXP P2 connector	*/
	/*	Place jumper between GPIO0 & GPIO2 pins of EDU-CIAA-NXP P2 connector	*/
	/*	Place jumper between GPIO1 & GPIO3 pins of EDU-CIAA-NXP P2 connector	*/
	/*	Place jumper between GPIO4 & GPIO6 pins of EDU-CIAA-NXP P2 connector	*/
	/*	Place jumper between GPIO5 & GPIO7 pins of EDU-CIAA-NXP P2 connector	*/

int main (void)
{
 	/* perform the needed initialization here */
	uint8_t		idx;
 	uint32_t	delay;

 	/* add your code here */
 	initBoard();

 	delay = 1;											/*	delay = 1mS				*/
	initGpio (gpio[0], OUTPUT);							/*	Init GPIO´s => OUTPUT 		*/
	initGpio (gpio[1], OUTPUT);
	initGpio (gpio[4], OUTPUT);
	initGpio (gpio[5], OUTPUT);
	initGpio (gpio[8], OUTPUT);

	while(1) {

		toggleGpio(gpio[0]);

		if (readGpio(gpio[2]))
			toggleGpio(gpio[1]);

		if (readGpio(gpio[3]))
			toggleGpio(gpio[4]);

		if (readGpio(gpio[6]))
			toggleGpio(gpio[5]);

		if (readGpio(gpio[7]))
			toggleGpio(gpio[8]);

		delay_ms (delay);
	}

	return 0;
}

#endif


#if (TEST == TEST_IRQs)
 	/*	Test TECLA1, TECLA2, TECLA3, TECLA4	on IRQs	*/
	/*	Test  GPIO1,  GPIO3,  GPIO5,  GPIO7 on IRQs	*/
int main (void)
{
 	/* perform the needed initialization here */

 	/* add your code here */
 	initBoard();

 	configIrq (irq[0],  irqPin[0], interrupcionTecla1, 50);		/*	IRQ0 on TECLA1, debounce = 50mS*/
	configIrq (irq[1],  irqPin[1], interrupcionTecla2, 50);		/*	IRQ1 on TECLA2, debounce = 50mS	*/
	configIrq (irq[2],  irqPin[2], interrupcionTecla3, 50);		/*	IRQ2 on TECLA3, debounce = 50mS	*/
	configIrq (irq[3],  irqPin[3], interrupcionTecla4, 50);		/*	IRQ3 on TECLA4, debounce = 50mS	*/
	configIrq (irq[4],  irqPin[5],  interrupcionGpio1, 50);		/*	IRQ4 on  GPIO1, debounce = 50mS	*/
	configIrq (irq[5],  irqPin[7],  interrupcionGpio3, 50);		/*	IRQ4 on  GPIO3, debounce = 50mS	*/
	configIrq (irq[6],  irqPin[9],  interrupcionGpio5, 50);		/*	IRQ4 on  GPIO5, debounce = 50mS	*/
	configIrq (irq[7], irqPin[11],  interrupcionGpio7, 50);		/*	IRQ4 on  GPIO7, debounce = 50mS	*/

	while(1)
		__WFI();										/*	Wait for Interrupt	*/

	return 0;
}


void interrupcionTecla1 (void)
{
	toggleRgb (rgb[0]);
}


void interrupcionTecla2 (void)
{
	toggleRgb (rgb[1]);
}


void interrupcionTecla3 (void)
{
	toggleRgb (rgb[2]);
}


void interrupcionTecla4 (void)
{
	toggleRgb (rgb[0]);
	toggleRgb (rgb[1]);
	toggleRgb (rgb[2]);
}


void interrupcionGpio1 (void)
{
	toggleRgb (rgb[0]);
}


void interrupcionGpio3 (void)
{
	toggleRgb (rgb[1]);
}


void interrupcionGpio5 (void)
{
	toggleRgb (rgb[2]);
}


void interrupcionGpio7 (void)
{
	toggleRgb (rgb[0]);
	toggleRgb (rgb[1]);
	toggleRgb (rgb[2]);
}

#endif



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

