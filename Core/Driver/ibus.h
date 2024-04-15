/*
 * ibus.h
 *
 *  Created on: 10 thg 2, 2023
 *      Author: sudo
 */

#ifndef LIB_IBUS_H_
#define LIB_IBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

enum index{
	CH1 = 0,
	CH2,
	CH3,
	CH4,
	CH5,
	CH6,
	CH7,
	CH8
};

void ibusGet();
int ibusFrameComplete(void);
void ibusDataReceive(uint8_t c);
void ibusInit(UART_HandleTypeDef *uartt,uint32_t baudrate);
void ibusCallback();
#ifdef __cplusplus
}
#endif

#endif /* LIB_IBUS_H_ */
