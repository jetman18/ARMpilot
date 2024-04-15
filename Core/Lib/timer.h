
#ifndef LIB_TIMER_H_
#define LIB_TIMER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "stdio.h"
typedef struct{
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
}bootTime_t;

extern TIM_HandleTypeDef *htimmz;
extern uint32_t _micros;

void timer_callback();
void timer_start(TIM_HandleTypeDef *htimz);
TIM_HandleTypeDef *timer_name();

#define micros() (_micros + (__HAL_TIM_GET_COUNTER(htimmz)))
#define millis() (micros() / 1000)
#define TIMER_CALLBACK()  (_micros += 65535)
#ifdef __cplusplus
}
#endif
#endif
