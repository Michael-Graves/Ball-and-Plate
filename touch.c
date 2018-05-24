/*
 * touch.c
 * 
 * Handles touch panel related functions
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
#include "driverlib/adc.h"
#include "touch.h"

/*
 * Base: ADC0_BASE, ADC1_BASE
 * Sequencer: 1,2
 * InputChannels: ADC_CTL_CH4, ADC_CTL_CH5, ADC_CTL_CH6, ADC_CTL_CH7
 */

// Initialize an ADC according to <base>, <sequence> number and <inputChannel>
void ADC_Init(uint32_t base, uint32_t sequencer, uint32_t inputChannel) {
    ADCSequenceConfigure(base, sequencer, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(base, sequencer, 0, inputChannel);
    ADCSequenceStepConfigure(base, sequencer, 1, inputChannel);
    ADCSequenceStepConfigure(base, sequencer, 2, inputChannel);
    ADCSequenceStepConfigure(base, sequencer, 3, ADC_CTL_IE | ADC_CTL_END | inputChannel);
    ADCSequenceEnable(base, sequencer);
}

/* Function to initialize the GPIO pins and ADCs
 *
 * NOTE: If PWM is using Port B and ADC is on PORT D you WILL need to modify the TIVA C board and remove
 *  resistors R9 and R10. See eval board schematic at: http://www.ti.com/lit/ug/spmu296/spmu296.pdf
 */
void Touch_Init(void) {
    //Port D handles the ADC Inputs for the touch panel

    //Enable Port D
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {}

    //Enable ADC0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {}

    //Enable ADC1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1)) {}

    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;


    ADCHardwareOversampleConfigure(ADCX_BASE, ADC_HARDWARE_OVERSAMPLE);
    ADCHardwareOversampleConfigure(ADCY_BASE, ADC_HARDWARE_OVERSAMPLE);

    //Configure X Axis ADCs
    ADC_Init(ADCX_BASE, 1, ADC_CTL_CH4);    //PD3 ADC XP
    ADC_Init(ADCX_BASE, 2, ADC_CTL_CH6);    //PD1 ADC XM

    //Configure Y Axis ADCs
    ADC_Init(ADCY_BASE, 1, ADC_CTL_CH7);    //PD0 ADC YP
    ADC_Init(ADCY_BASE, 2, ADC_CTL_CH5);    //PD2 ADC YM
}

// Sets the Touch screen pins to inputs/high impedance
void Touch_Off(uint32_t base, uint32_t ADC_P, uint32_t ADC_M, uint32_t GPIO_P, uint32_t GPIO_M) {
    GPIOPinTypeGPIOInput(base, ADC_P | ADC_M | GPIO_P | GPIO_M);
}

/* Configure the Touchscreen related pins according to the parameters
 *  <ADC_> pins will be set as ADCs, <GPIO_> will be set to 3V3 and GND
 */
void Touch_Config(uint32_t base, uint32_t ADC_P, uint32_t ADC_M, uint32_t GPIO_P, uint32_t GPIO_M) {
    //Configure the +/- pins of the touch screen and write + to 3.3V
    GPIOPinWrite(base, GPIO_P | GPIO_M, GPIO_P);
    GPIOPadConfigSet(base, GPIO_P | GPIO_M, GPIO_STRENGTH_6MA, GPIO_PIN_TYPE_STD);
    GPIOPinTypeGPIOOutput(base, GPIO_P | GPIO_M);
    GPIODirModeSet(base, GPIO_P | GPIO_M, GPIO_DIR_MODE_OUT);
    GPIOPinWrite(base, GPIO_P | GPIO_M, GPIO_P);

    //Configure ADC Inputs
    GPIOPinTypeADC(base, ADC_P | ADC_M);
    GPIOPadConfigSet(base, ADC_P | ADC_M, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_ANALOG);
    GPIODirModeSet(base, ADC_P | ADC_M, GPIO_DIR_MODE_IN);

}

uint32_t GetADCValue(uint32_t base, uint32_t sequencer) {
    uint32_t ui32ADCValue[4];

    //Trigger and read the ADC
    ADCIntClear(base, sequencer);
    ADCProcessorTrigger(base, sequencer);

    // Wait for the ADC interrupt
    while(!ADCIntStatus(base, sequencer, false)) {}
    ADCSequenceDataGet(base, sequencer, ui32ADCValue);

    // Average the 4 samples from the sequencer
    return (ui32ADCValue[0] + ui32ADCValue[1] + ui32ADCValue[2] + ui32ADCValue[3] + 2)/4;
}

/*
 * Function to check if a touch is present on the touch screen
 *  Sets one surface to GND and the other to pull up enabled inputs.
 *  Checks if input pins are LOW (Connected to first surface)
 */
_Bool Touch_Present() {
    //Set Y Surface to LOW
    GPIOPinTypeGPIOOutput(TOUCH_BASE, TOUCH_YP | TOUCH_YM);
    GPIODirModeSet(TOUCH_BASE, TOUCH_YP | TOUCH_YM, GPIO_DIR_MODE_OUT);
    GPIOPinWrite(TOUCH_BASE, TOUCH_YP | TOUCH_YM, 0x00);

    //Set X Surface to PULL UP
    GPIOPinTypeGPIOInput(TOUCH_BASE, TOUCH_XP | TOUCH_XM);
    GPIODirModeSet(TOUCH_BASE, TOUCH_XP | TOUCH_XM, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(TOUCH_BASE, TOUCH_XP | TOUCH_XM, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //Wait a short time for pullups
    SysCtlDelay(8000);

    //Check if a pin is LOW (connected to Y Surface)
    uint8_t read = GPIOPinRead(TOUCH_BASE, TOUCH_XP | TOUCH_XM);
    _Bool output = !(read == (TOUCH_XP | TOUCH_XM));
    return output;
}

uint32_t Touch_Read_Y() {
    Touch_Config(TOUCH_BASE, TOUCH_YP, TOUCH_YM, TOUCH_XP, TOUCH_XM);

    // Read the positive and negative ADCs for additional filter
    uint32_t p = GetADCValue(ADCY_BASE, 1);
    uint32_t m = GetADCValue(ADCY_BASE, 2);

    Touch_Off(TOUCH_BASE, TOUCH_YP, TOUCH_YM, TOUCH_XP, TOUCH_XM);
    // Put additional filtering/conversion code here
    return (p + m + 1)/2;
}

uint32_t Touch_Read_X() {
    Touch_Config(TOUCH_BASE, TOUCH_XP, TOUCH_XM, TOUCH_YP, TOUCH_YM);

    // Read the positive and negative ADCs for additional filter
    uint32_t p = GetADCValue(ADCX_BASE, 1);
    uint32_t m = GetADCValue(ADCX_BASE, 2);

    Touch_Off(TOUCH_BASE, TOUCH_XP, TOUCH_XM, TOUCH_YP, TOUCH_YM);
    // Put additional filtering/conversion code here
    return (p + m + 1)/2;
}
