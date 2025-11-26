/*
 * UltrasonicSensor.h
 *
 *  Created on: Nov 20, 2025
 *      Author: andrewvalles
 */

#ifndef INC_ULTRASONICSENSOR_H_
#define INC_ULTRASONICSENSOR_H_

void Sensor_GPIO_Init(void);	//function to initialize pins used for sensors
void delay_us(const uint32_t time_us);	//function for us delays
void SysTick_Init(void);	//function for initializing SysTick
float get_distance_cm(uint16_t TRIG_PIN, uint16_t ECHO_IDR_PIN);	//function to get distance measurements

#endif /* INC_ULTRASONICSENSOR_H_ */
