/*
 * Board_LED.c
 *
 *  Created on: Nov 21, 2025
 *      Author: andrewvalles
 */

#include "main.h"
#include "Board_LED_Init.h"
/*
void LED_Init(void) {
	//initialization code for the GPIO pins used for the LED
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOBEN);
	GPIOB->MODER   &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE14);
	GPIOB->MODER   |=  (GPIO_MODER_MODE7_0 | GPIO_MODER_MODE14_0);
	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT14);
	GPIOB->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED7_Pos) | (3 << GPIO_OSPEEDR_OSPEED14_Pos));
	GPIOB->BRR = (GPIO_PIN_7 | GPIO_PIN_14);
}
*/
