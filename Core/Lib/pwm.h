
#ifndef PWM_H
#define PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

#define INVERT 1
#define NOT_INVERT 0

enum pwm_channel{
	ch1 = TIM_CHANNEL_1,
	ch2 = TIM_CHANNEL_2,
	ch3 = TIM_CHANNEL_3,
	ch4 = TIM_CHANNEL_4
};


void initPWM(TIM_HandleTypeDef *htim);
void writePwm(uint32_t Channel,int16_t dulty,int8_t invert);
void write_pwm_ctrl(uint16_t throtlle,uint16_t servoL,uint16_t servoR);
void pwm_systick_callback();
#ifdef __cplusplus
}
#endif

#endif /* LIB_PWMWIRITE_H_ */
