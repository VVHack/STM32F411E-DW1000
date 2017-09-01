/*
 * deca_spi.h
 *
 *  Created on: Aug 18, 2017
 *      Author: vedagupt
 */

#include "stm32f4xx.h"

#ifndef DECA_SPI_H_
#define DECA_SPI_H_

#define SLOW_SPI 0
#define FAST_SPI 1


void DW_SPI_Init(int mode);

void DW_SPI_START();

void DW_SPI_END();

uint8_t DW_SPI_SEND(uint8_t data);

void initializeInterrupts();

#endif /* DECA_SPI_H_ */
