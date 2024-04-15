#ifndef _LOG_
#define _LOG_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"


void print_float(float n);
int writeln_int(UART_HandleTypeDef *huart,int n);
int write_char(UART_HandleTypeDef *huart,uint8_t *);

#ifdef __cplusplus
}
#endif
#endif
