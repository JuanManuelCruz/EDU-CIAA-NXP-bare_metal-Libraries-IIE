/*
 * teclas.h
 *
 *  Created on: 12/11/2015
 *      Author: Juan Manuel Cruz
 */

#ifndef TECLAS_H_
#define TECLAS_H_


#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif

#include "cmsis_43xx.h"

/*	TECLA1 => P1_0 en GPIO 0 [4]	*
 *	TECLA2 => P1_1 en GPIO 0 [8]	*
 *	TECLA3 => P1_2 en GPIO 0 [9]	*
 *	TECLA4 => P1_6 en GPIO 1 [9]	*/

// TECLASs
#define TECLA1	0
#define TECLA2	1
#define TECLA3	2
#define TECLA4	3

#define tecla_t	uint8_t
static const tecla_t tecla[] = {TECLA1, TECLA2, TECLA3, TECLA4};


void initTeclas (void);
void initTecla (const tecla_t teclaNumber);
uint8_t readTecla (const tecla_t teclaNumber);
void testTecla (const tecla_t teclaNumber);


#endif /* TECLAS_H_ */
