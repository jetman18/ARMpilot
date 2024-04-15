#ifndef SIMULATION_H
#define SIMULATION_H
#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

void HITL_init(UART_HandleTypeDef *huart);
UART_HandleTypeDef* HITL_uart_port();
void HITL_callback();
#ifdef __cplusplus
}
#endif
#endif
