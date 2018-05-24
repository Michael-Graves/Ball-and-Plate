/*
 * touch.h
 *
 *  Created on: Mar 20, 2018
 *      Author: Newhb
 */

#ifndef TOUCH_H_
#define TOUCH_H_

// +/- (P/M) is defined relative to GPIO voltage during reading
#define TOUCH_BASE GPIO_PORTD_BASE

#define ADCX_BASE ADC0_BASE
#define ADCY_BASE ADC1_BASE

#define TOUCH_XP GPIO_PIN_3
#define TOUCH_XM GPIO_PIN_1

#define TOUCH_YP GPIO_PIN_0
#define TOUCH_YM GPIO_PIN_2

#define ADC_HARDWARE_OVERSAMPLE 8

void Touch_Init(void);
_Bool Touch_Present();
uint32_t Touch_Read_X();
uint32_t Touch_Read_Y();

/* Internal Functions
 * void Touch_Config(uint32_t base, uint32_t ADC_P, uint32_t ADC_M, uint32_t GPIO_P, uint32_t GPIO_M);
 * uint32_t GetADCValue(uint32_t base, uint32_t sequencer);
 * void ADC_Init(uint32_t base, uint32_t sequencer, uint32_t inputChannel);
 */

#endif /* TOUCH_H_ */
