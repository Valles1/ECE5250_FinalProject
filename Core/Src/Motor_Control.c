/*
 * Motor_Control.c
 *
 *  Created on: Nov 23, 2025
 *      Author: andrewvalles
 */

#include "main.h"
#include "Motor_Control.h"

void Stop_MotorControl(void) {
	// 1) Set direction: IN1=1, IN2=0  (PB11=SET, PB14=RESET)
	 GPIOB -> BRR = GPIO_PIN_2;    // IN1
	 GPIOB -> BRR = GPIO_PIN_6;  // IN2

	 GPIOB -> BRR = GPIO_PIN_10;    // IN3 (right)
	 GPIOB -> BRR = GPIO_PIN_11;  // IN4 (right)

	 GPIOD -> BRR = GPIO_PIN_4;    // IN1
	 GPIOD -> BRR = GPIO_PIN_5;  // IN2

	 GPIOD -> BRR = GPIO_PIN_6;    // IN3
	 GPIOD -> BRR = GPIO_PIN_3;  // IN4
}

