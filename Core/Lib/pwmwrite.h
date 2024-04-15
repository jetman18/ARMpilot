
#ifndef PWMWRITE_H
#define PWMWRITE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

enum pwm_channel{
	ch1 = TIM_CHANNEL_1,
	ch2 = TIM_CHANNEL_2,
	ch3 = TIM_CHANNEL_3,
	ch4 = TIM_CHANNEL_4
};
void initPWM(TIM_HandleTypeDef *htim);
void initOneshot125(TIM_HandleTypeDef *htim);
void motoIdle();
void writePwm(uint32_t Channel,int16_t dulty);
void writeOneshot125(uint32_t Channel,int16_t dulty);
#ifdef __cplusplus
}
#endif

#endif /* LIB_PWMWIRITE_H_ */
