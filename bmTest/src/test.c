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
#define TEST_SSPs	7		/*	Test SSP									*/

#define TEST	TEST_SSPs	/*	Type the mane of Test to do here 			*/

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


#if (TEST == TEST_SSPs)
/*	Test SSP	*/

#define LPC_SSP           LPC_SSP1
#define SSP_IRQ           SSP1_IRQn
#define LPC_GPDMA_SSP_TX  GPDMA_CONN_SSP1_Tx
#define LPC_GPDMA_SSP_RX  GPDMA_CONN_SSP1_Rx
#define SSPIRQHANDLER SSP1_IRQHandler

#define BUFFER_SIZE                         (0x100)
#define SSP_DATA_BITS                       (SSP_BITS_8)
#define SSP_DATA_BIT_NUM(databits)          (databits+1)
#define SSP_DATA_BYTES(databits)            (((databits) > SSP_BITS_8) ? 2:1)
#define SSP_LO_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? 0xFF:(0xFF>>(8-SSP_DATA_BIT_NUM(databits))))
#define SSP_HI_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? (0xFF>>(16-SSP_DATA_BIT_NUM(databits))):0)

#define SSP_MODE_SEL                        (0x31)
#define SSP_TRANSFER_MODE_SEL               (0x32)
#define SSP_MASTER_MODE_SEL                 (0x31)
#define SSP_SLAVE_MODE_SEL                  (0x32)
#define SSP_POLLING_SEL                     (0x31)
#define SSP_INTERRUPT_SEL                   (0x32)
#define SSP_DMA_SEL                         (0x33)

#define	EOF	(-1)


/* The DEBUG* functions are selected based on system configuration.
   Code that uses the DEBUG* functions will have their I/O routed to
   the UART, semihosting, or nowhere. */
#if defined(DEBUG_ENABLE)
#if defined(DEBUG_SEMIHOSTING)
#define DEBUGINIT()
#define DEBUGOUT(...) printf(__VA_ARGS__)
#define DEBUGSTR(str) printf(str)
#define DEBUGIN() (int) EOF

#else
#define DEBUGINIT() Board_Debug_Init()
#define DEBUGOUT(...) printf(__VA_ARGS__)
#define DEBUGSTR(str) Board_UARTPutSTR(str)
#define DEBUGIN() Board_UARTGetChar()
#endif /* defined(DEBUG_SEMIHOSTING) */

#else
#define DEBUGINIT()
#define DEBUGOUT(...)
#define DEBUGSTR(str)
#define DEBUGIN() (int) EOF
#endif /* defined(DEBUG_ENABLE) */

/* Tx buffer */
static uint8_t Tx_Buf[BUFFER_SIZE];

/* Rx buffer */
static uint8_t Rx_Buf[BUFFER_SIZE];

static SSP_ConfigFormat ssp_format;
static Chip_SSP_DATA_SETUP_T xf_setup;
static volatile uint8_t  isXferCompleted = 0;
static uint8_t dmaChSSPTx, dmaChSSPRx;
static volatile uint8_t isDmaTxfCompleted = 0;
static volatile uint8_t isDmaRxfCompleted = 0;

#if defined(DEBUG_ENABLE)
static char sspWaitingMenu[] = "SSP Polling: waiting for transfer ...\n\r";
static char sspIntWaitingMenu[]  = "SSP Interrupt: waiting for transfer ...\n\r";
static char sspDMAWaitingMenu[]  = "SSP DMA: waiting for transfer ...\n\r";

static char sspPassedMenu[] = "SSP: Transfer PASSED\n\r";
static char sspFailedMenu[] = "SSP: Transfer FAILED\n\r";

static char sspTransferModeSel[] = "\n\rPress 1-3 or 'q' to exit\n\r"
								   "\t 1: SSP Polling Read Write\n\r"
								   "\t 2: SSP Int Read Write\n\r"
								   "\t 3: SSP DMA Read Write\n\r";

static char helloMenu[] = "Hello NXP Semiconductors \n\r";
static char sspMenu[] = "SSP demo \n\r";
static char sspMainMenu[] = "\t 1: Select SSP Mode (Master/Slave)\n\r"
							"\t 2: Select Transfer Mode\n\r";
static char sspSelectModeMenu[] = "\n\rPress 1-2 to select or 'q' to exit:\n\r"
								  "\t 1: Master \n\r"
								  "\t 2: Slave\n\r";
#endif /* defined(DEBUG_ENABLE) */

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initialize buffer */
static void Buffer_Init(void)
{
	uint16_t i;
	uint8_t ch = 0;

	for (i = 0; i < BUFFER_SIZE; i++) {
		Tx_Buf[i] = ch++;
		Rx_Buf[i] = 0xAA;
	}
}

/* Verify buffer after transfer */
static uint8_t Buffer_Verify(void)
{
	uint16_t i;
	uint8_t *src_addr = (uint8_t *) &Tx_Buf[0];
	uint8_t *dest_addr = (uint8_t *) &Rx_Buf[0];

	for ( i = 0; i < BUFFER_SIZE; i++ ) {

		if (((*src_addr) & SSP_LO_BYTE_MSK(ssp_format.bits)) !=
				((*dest_addr) & SSP_LO_BYTE_MSK(ssp_format.bits))) {
				return 1;
		}
		src_addr++;
		dest_addr++;

		if (SSP_DATA_BYTES(ssp_format.bits) == 2) {
			if (((*src_addr) & SSP_HI_BYTE_MSK(ssp_format.bits)) !=
				  ((*dest_addr) & SSP_HI_BYTE_MSK(ssp_format.bits))) {
					return 1;
			}
			src_addr++;
			dest_addr++;
			i++;
		}
	}
	return 0;
}

/* Select the Transfer mode : Polling, Interrupt or DMA */
static void appSSPTest(void)
{
	int key;

	DEBUGOUT(sspTransferModeSel);

	dmaChSSPTx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, LPC_GPDMA_SSP_TX);
	dmaChSSPRx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, LPC_GPDMA_SSP_RX);

	xf_setup.length = BUFFER_SIZE;
	xf_setup.tx_data = Tx_Buf;
	xf_setup.rx_data = Rx_Buf;

	while (1) {
		key = 0xFF;
		do {
			key = DEBUGIN();
		} while ((key & 0xFF) == 0xFF);

		Buffer_Init();

		switch (key) {
		case SSP_POLLING_SEL:	/* SSP Polling Read Write Mode */
			DEBUGOUT(sspWaitingMenu);
			xf_setup.rx_cnt = xf_setup.tx_cnt = 0;

			Chip_SSP_RWFrames_Blocking(LPC_SSP, &xf_setup);

			if (Buffer_Verify() == 0) {
				DEBUGOUT(sspPassedMenu);
			}
			else {
				DEBUGOUT(sspFailedMenu);
			}
			break;

		case SSP_INTERRUPT_SEL:
			DEBUGOUT(sspIntWaitingMenu);

			isXferCompleted = 0;
			xf_setup.rx_cnt = xf_setup.tx_cnt = 0;

			Chip_SSP_Int_FlushData(LPC_SSP);/* flush dummy data from SSP FiFO */
			if (SSP_DATA_BYTES(ssp_format.bits) == 1) {
				Chip_SSP_Int_RWFrames8Bits(LPC_SSP, &xf_setup);
			}
			else {
				Chip_SSP_Int_RWFrames16Bits(LPC_SSP, &xf_setup);
			}

			Chip_SSP_Int_Enable(LPC_SSP);	/* enable interrupt */
			while (!isXferCompleted) {}

			if (Buffer_Verify() == 0) {
				DEBUGOUT(sspPassedMenu);
			}
			else {
				DEBUGOUT(sspFailedMenu);
			}
			break;

		case SSP_DMA_SEL:	/* SSP DMA Read and Write: fixed on 8bits */
			DEBUGOUT(sspDMAWaitingMenu);
			isDmaTxfCompleted = isDmaRxfCompleted = 0;

			Chip_SSP_DMA_Enable(LPC_SSP);
			/* data Tx_Buf --> SSP */
			Chip_GPDMA_Transfer(LPC_GPDMA, dmaChSSPTx,
							  (uint32_t) &Tx_Buf[0],
							  LPC_GPDMA_SSP_TX,
							  GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,
							  BUFFER_SIZE);
			/* data SSP --> Rx_Buf */
			Chip_GPDMA_Transfer(LPC_GPDMA, dmaChSSPRx,
							  LPC_GPDMA_SSP_RX,
							  (uint32_t) &Rx_Buf[0],
							  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
							  BUFFER_SIZE);

			while (!isDmaTxfCompleted || !isDmaRxfCompleted) {}
			if (Buffer_Verify() == 0) {
				DEBUGOUT(sspPassedMenu);
			}
			else {
				DEBUGOUT(sspFailedMenu);
			}
			Chip_SSP_DMA_Disable(LPC_SSP);
			break;

		case 'q':
		case 'Q':
			Chip_GPDMA_Stop(LPC_GPDMA, dmaChSSPTx);
			Chip_GPDMA_Stop(LPC_GPDMA, dmaChSSPRx);
			return;

		default:
			break;
		}

		DEBUGOUT(sspTransferModeSel);
	}

}

/* Select the SSP mode : Master or Slave */
static void appSSPSelectModeMenu(void)
{
	int key;

	DEBUGOUT(sspSelectModeMenu);

	while (1) {
		key = 0xFF;
		do {
			key = DEBUGIN();
		} while ((key & 0xFF) == 0xFF);

		switch (key) {
		case SSP_MASTER_MODE_SEL:	/* Master */
			Chip_SSP_SetMaster(LPC_SSP, 1);
			DEBUGOUT("Master Mode\n\r");
			return;

		case SSP_SLAVE_MODE_SEL:	/* Slave */
			Chip_SSP_SetMaster(LPC_SSP, 0);
			DEBUGOUT("Slave Mode\n\r");
			return;

		case 'q':
			return;

		default:
			break;
		}
		DEBUGOUT(sspSelectModeMenu);
	}

}

/* The main menu of the example. Allow user select the SSP mode (master or slave) and Transfer
   mode (Polling, Interrupt or DMA) */
static void appSSPMainMenu(void)
{
	int key;

	DEBUGOUT(helloMenu);
	DEBUGOUT(sspMenu);
	DEBUGOUT(sspMainMenu);

	while (1) {
		key = 0xFF;
		do {
			key = DEBUGIN();
		} while ((key & 0xFF) == 0xFF);

		switch (key) {
		case SSP_MODE_SEL:	/* Select SSP Mode */
			appSSPSelectModeMenu();
			break;

		case SSP_TRANSFER_MODE_SEL:	/* Select Transfer Mode */
			appSSPTest();
			break;

		default:
			break;
		}
		DEBUGOUT(sspMainMenu);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	SSP interrupt handler sub-routine
 * @return	Nothing
 */
void SSPIRQHANDLER(void)
{
	Chip_SSP_Int_Disable(LPC_SSP);	/* Disable all interrupt */
	if (SSP_DATA_BYTES(ssp_format.bits) == 1) {
		Chip_SSP_Int_RWFrames8Bits(LPC_SSP, &xf_setup);
	}
	else {
		Chip_SSP_Int_RWFrames16Bits(LPC_SSP, &xf_setup);
	}

	if ((xf_setup.rx_cnt != xf_setup.length) || (xf_setup.tx_cnt != xf_setup.length)) {
		Chip_SSP_Int_Enable(LPC_SSP);	/* enable all interrupts */
	}
	else {
		isXferCompleted = 1;
	}
}

/**
 * @brief	DMA interrupt handler sub-routine. Set the waiting flag when transfer is successful
 * @return	Nothing
 */
void DMA_IRQHandler(void)
{
	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaChSSPTx) == SUCCESS) {
		isDmaTxfCompleted = 1;
	}

	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaChSSPRx) == SUCCESS) {
		isDmaRxfCompleted = 1;
	}
}

void Board_SSP_Init(LPC_SSP_T *pSSP)
{
	if (pSSP == LPC_SSP1) {
		Chip_SCU_PinMuxSet(0x1, 5, (SCU_PINIO_FAST | SCU_MODE_FUNC5));  /* P1.5 => SSEL1 */
		Chip_SCU_PinMuxSet(0xF, 4, (SCU_PINIO_FAST | SCU_MODE_FUNC0));  /* PF.4 => SCK1 */

		Chip_SCU_PinMuxSet(0x1, 4, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC5)); /* P1.4 => MOSI1 */
		Chip_SCU_PinMuxSet(0x1, 3, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC5)); /* P1.3 => MISO1 */
	}
	else {
		return;
	}
}

int main (void)
{
 	/* perform the needed initialization here */

 	/* add your code here */
 	initBoard();

	/* SSP initialization */
	Board_SSP_Init(LPC_SSP);

	Chip_SSP_Init(LPC_SSP);

	ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
	ssp_format.bits = SSP_DATA_BITS;
	ssp_format.clockMode = SSP_CLOCK_MODE0;
        Chip_SSP_SetFormat(LPC_SSP, ssp_format.bits, ssp_format.frameFormat, ssp_format.clockMode);
	Chip_SSP_Enable(LPC_SSP);

	/* Initialize GPDMA controller */
	Chip_GPDMA_Init(LPC_GPDMA);

	/* Setting GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ(DMA_IRQn);

	/* Setting SSP interrupt */
	NVIC_EnableIRQ(SSP_IRQ);

	appSSPMainMenu();
	return 0;
}

#endif


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

