/*
 * button.c
 * 
 * Handles button related functions
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
#include "button.h"

void (*callBackFunction_ptr)(_Bool, _Bool);

/* Initialize PortF GPIOs */
void Button_Init(void (*callBackFunction)(_Bool, _Bool)) {
    callBackFunction_ptr = callBackFunction;

    // Enable Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Wait for the GPIO module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}

    // Unlocks the NMI (non-maskable interrupt) on GPIO Port F
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    // Set SW1 and SW2 GPIO as inputs
    GPIODirModeSet(GPIO_PORTF_BASE, BTN_PINS , GPIO_DIR_MODE_IN);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_PINS);

    // Enable internal pull up resistors
    GPIOPadConfigSet(GPIO_PORTF_BASE, BTN_PINS , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Register, configure and enable the Button Interrupt handler
    GPIOIntRegister(GPIO_PORTF_BASE, Button_Handler);
    GPIOIntTypeSet(GPIO_PORTF_BASE, BTN_PINS, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTF_BASE, BTN_PINS);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);

    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();

    TimerEnable(TIMER1_BASE, TIMER_A);
}

void LEDWrite(uint32_t ledState) {
    GPIOPinWrite(GPIO_PORTF_BASE, LED_PINS, ledState);
}

/* Interrupt GPIO_PORTF when button pressed */
void Button_Handler(void){
    // Clear interrupt flag
    GPIOIntClear(GPIO_PORTF_BASE, BTN_PINS);

    // Update the current button state

    // Check if a button was pressed
    //_Bool wasAButtonPressed = (buttonState & (BUTTON_1 + BUTTON_2));
    //if(wasAButtonPressed) {
        // Reload and enable the debounce timer
        TimerLoadSet(TIMER1_BASE, TIMER_A, DEBOUNCE_PERIOD);
        TimerEnable(TIMER1_BASE, TIMER_A);
    //}
}

/* Interrupt for handling button debounce */
void Debounce_Handler(void) {
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    uint8_t buttonState = (~(GPIOPinRead(GPIO_PORTF_BASE, BTN_PINS))) & (BUTTON_1 + BUTTON_2);

    if(buttonState) {
        (*callBackFunction_ptr)((buttonState & BUTTON_1), (buttonState & BUTTON_2));
    }
}
