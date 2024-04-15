#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx.h"
#include "stdio.h"

typedef struct pid{
	float kp;
	float ki;
	float kd;
	float i_term;
	float last_input;
	float D_filted;
	float I_range;
	float f_cut_D;
	float dt;
	uint8_t init;
}pid_t;
void  pid_init(pid_t  *pid,float kp, float ki, float kd, float f_cut_D, float maxI,float dt);
float pid_cal(pid_t *pid,float input, float setpoint);
void  pid_reset(pid_t *t);
#ifdef __cplusplus
}
#endif

#endif
