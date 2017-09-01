/*
 * deca_led.h
 *
 *  Created on: Aug 18, 2017
 *      Author: vedagupt
 */

#include "stm32f4xx.h"

#ifndef DECA_LED_H_
#define DECA_LED_H_

#define ORANGE GPIO_Pin_13
#define GREEN GPIO_Pin_12
#define RED GPIO_Pin_14
#define BLUE GPIO_Pin_15

void Delay(__IO uint32_t nCount);
void delayMiliseconds(volatile int nCount);
void led_init();
void blinkRed();
void blinkBlue();
void blinkOrange();
void blinkGreen();
void displayWithLED(int bt);

#endif /* DECA_LED_H_ */
