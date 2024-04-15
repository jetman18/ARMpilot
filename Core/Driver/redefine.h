
#ifndef REDEFINE_H_
#define REDEFINE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "timer.h"


void HAL_IncTick(void)
{
  //resetCounter();
  uwTick += uwTickFreq;
}



#ifdef __cplusplus
}
#endif
#endif
