/*
 * servo.h
 *
 *  Created on: Mar 20, 2018
 *      Author: Newhb
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "driverlib/pwm.h"

#define SERVO_1_PIN GPIO_PIN_6
#define SERVO_2_PIN GPIO_PIN_7
#define SERVO_PINS SERVO_1_PIN | SERVO_2_PIN

// Period = (80 MHz / 16) / (Desired Frequency Hz) - 1
#define PWM_PERIOD (50000)                                     //20,0000us (Servo DutyCycle Max)

#define SERVO_DUTYCYCLE_MAX 2000                              //10us
#define SERVO_DUTYCYCLE_USEC (PWM_PERIOD / SERVO_DUTYCYCLE_MAX) //ticks per 10us
#define SERVO_DUTYCYCLE_START (60 * SERVO_DUTYCYCLE_USEC)     //1000us in ticks
#define SERVO_DUTYCYCLE_END (250 * SERVO_DUTYCYCLE_USEC)       //2000us in ticks

#define SERVO_DUTYCYCLE_DEGREES ((SERVO_DUTYCYCLE_END - SERVO_DUTYCYCLE_START)/180) //DEGREES

#define SERVO_1 PWM_OUT_0
#define SERVO_2 PWM_OUT_1

void Servo_Init(uint32_t init_1, uint32_t init_2);
void Servo_Set(uint32_t servo, uint32_t dutyCycle);
void Servo_Set_Degrees(uint32_t servo, uint32_t degrees);


#endif /* SERVO_H_ */
