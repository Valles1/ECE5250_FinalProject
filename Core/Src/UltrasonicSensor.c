/*
 * UltrasonicSensor.c
 *
 *  Created on: Nov 21, 2025
 *      Author: andrewvalles
 */

#include "main.h"
#include "UltrasonicSensor.h"

void Sensor_GPIO_Init(void)
{
	//initializing GPIO pins for ultra-sonic sensors (2 Inputs, 2 outputs)

	//perform bitwise OR to give value of RCC_AHB2ENR_GPIOAEN (1) to the AHB2ENR Register
	//this enables the clock for this port
	RCC -> AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);

	//pins 2 & 3 set to general purpose output while 0 and 1 are left as inputs
	GPIOA -> MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	GPIOA -> MODER |= (GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0);

	//sets the output speed, output type, and pull up/down resistors
	GPIOA -> OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);
	GPIOA -> PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);
	GPIOA -> OSPEEDR |= ((3 << GPIO_OSPEEDR_OSPEED0_Pos) | (3 << GPIO_OSPEEDR_OSPEED1_Pos) |
			(3 << GPIO_OSPEEDR_OSPEED2_Pos) | (3 << GPIO_OSPEEDR_OSPEED3_Pos));

	GPIOA -> BRR = (GPIO_PIN_2 | GPIO_PIN_3);	//sets these pins

}

float get_distance_cm(uint16_t TRIG_PIN, uint16_t ECHO_IDR_PIN)
{
	//define variables used for ultrasonic sensor
	uint32_t duration_us = 0;
	float distance_cm;


	GPIOA -> BSRR |= (TRIG_PIN);	//trigger a HIGH
	delay_us( 10 );					//delay for 10 microseconds
	GPIOA -> BRR = (TRIG_PIN);	//trigger a LOW

	uint32_t timeout = 30000;
	while(!(GPIOA->IDR & ECHO_IDR_PIN) && timeout--);	//wait for ECHO to go HIGH
	if (timeout == 0) return -1.0f;


	while(GPIOA->IDR & ECHO_IDR_PIN)
	{
		delay_us( 1 );
		duration_us++;
		if (duration_us > 3000) break;
	}

	distance_cm =  (duration_us * 20) / 58.0f;

	return distance_cm;
}

void SysTick_Init(void) {
	SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk |     	// enable SysTick Timer
                      SysTick_CTRL_CLKSOURCE_Msk); 	// select CPU clock
	SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);  	// disable interrupt


}

void delay_us(const uint32_t time_us) {
	// set the counts for the specified delay

	SysTick->LOAD = (uint32_t)((time_us * (SystemCoreClock / 1000000)) - 1);
	SysTick->VAL = 0;                                  	 // clear timer count
	SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);    	 // clear count flag
	while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // wait for flag

}
