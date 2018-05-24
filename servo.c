/*
 * servo.c
 * 
 * Handles servo related functions
 *
 *  Created on: Mar 20, 2018
 *      Author: Michael Graves
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "servo.h"

void PWM_Init(uint32_t init_1, uint32_t init_2) {
    // Set the PWM CLOCK divider to /32 of the main clock
    SysCtlPWMClockSet(SYSCTL_PWMDIV_32);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    // Wait for the PWM module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) {}

    // Configure the PWM mode
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);

    // Configure the frequency to 100Hz
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_PERIOD);

    // Init duty cycle
    PWMPulseWidthSet(PWM0_BASE, SERVO_1, (SERVO_DUTYCYCLE_START + SERVO_DUTYCYCLE_END)/2);
    PWMPulseWidthSet(PWM0_BASE, SERVO_2, (SERVO_DUTYCYCLE_START + SERVO_DUTYCYCLE_END)/2);

    // Turning on PWM
    PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

}

void Servo_Init(uint32_t init_1, uint32_t init_2) {
    // Enable Port B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Wait for the GPIO module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

    GPIODirModeSet(GPIO_PORTB_BASE, SERVO_PINS, GPIO_DIR_MODE_OUT);

    GPIOPinTypePWM(GPIO_PORTB_BASE, SERVO_PINS);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);

    PWM_Init(init_1, init_2);
}

// Function to set <servo> to position <dutyCycle>
//  Use "Servo_Set_Degrees" to avoid problems if possible
void Servo_Set(uint32_t servo, uint32_t dutyCycle) {
    PWMPulseWidthSet(PWM0_BASE, servo, dutyCycle);
}

// Function to set <servo> to position <degrees> which is scaled by 10 (900 = 90 degrees)
void Servo_Set_Degrees(uint32_t servo, uint32_t degrees) {
    uint32_t dutyCycle = SERVO_DUTYCYCLE_START + (degrees * SERVO_DUTYCYCLE_DEGREES)/10;

    PWMPulseWidthSet(PWM0_BASE, servo, dutyCycle);
}
