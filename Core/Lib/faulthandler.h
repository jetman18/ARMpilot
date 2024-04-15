#ifndef FAULTHANDLER_H
#define FAULTHANDLER_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"

void fault_pc13_blink(int delay);


#ifdef __cplusplus
}
#endif

#endif

