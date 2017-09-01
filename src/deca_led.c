/*
 * deca_led.c
 *
 *  Created on: Aug 18, 2017
 *      Author: vedagupt
 */


#include "deca_led.h"

void deca_sleep(unsigned int time_ms){
	delayMiliseconds(time_ms);
}

void led_init(){
	GPIO_InitTypeDef GPIO_LED;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_LED.GPIO_Pin = ORANGE | GREEN | RED | BLUE;
	GPIO_LED.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_LED.GPIO_OType = GPIO_OType_PP;
	GPIO_LED.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOD, &GPIO_LED);
}



void Delay(__IO uint32_t nCount){
	while(nCount--){

	}
}

void delayMiliseconds(int nCount){
	Delay(16800*nCount);
}


void blinkOrange(){
	GPIO_WriteBit(GPIOD, ORANGE, Bit_SET);
	delayMiliseconds(200);
	GPIO_WriteBit(GPIOD, ORANGE, Bit_RESET);
	delayMiliseconds(200);
}

void blinkGreen(){
	GPIO_WriteBit(GPIOD, GREEN, Bit_SET);
	delayMiliseconds(200);
	GPIO_WriteBit(GPIOD, GREEN, Bit_RESET);
	delayMiliseconds(200);
}

void blinkRed(){
	GPIO_WriteBit(GPIOD, RED, Bit_SET);
	delayMiliseconds(200);
	GPIO_WriteBit(GPIOD, RED, Bit_RESET);
	delayMiliseconds(200);
}

void blinkBlue(){
	GPIO_WriteBit(GPIOD, BLUE, Bit_SET);
	delayMiliseconds(200);
	GPIO_WriteBit(GPIOD, BLUE, Bit_RESET);
	delayMiliseconds(200);
}

void displayWithLED(int bt){
	int i=0;
	for(i=7;i>=0;--i){
		if(bt & (1<<i)){
			//blinkRed();
			blinkOrange();
		}
		else{
			blinkBlue();
		}
	}

	delayMiliseconds(1000);
}
