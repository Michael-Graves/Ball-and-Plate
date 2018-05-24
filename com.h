/*
 * com.h
 *
 *  Created on: Mar 20, 2018
 *      Author: Newhb
 */

#ifndef COM_H_
#define COM_H_

#define BAUD_RATE 115200

void UARTCharSend(char character);
void UARTIntSend(uint16_t integer);
void COM_Init(void);

#endif /* COM_H_ */
