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
	float I;
	float pre_value;
	float D_filted;
	float max_I;
	float f_cut_D;
	uint32_t last_call_us;
}pid__t;
void initPid(pid__t  *pid,float kp, float ki, float kd, float f_cut_D, float maxI);
float pidCalculate(pid__t *gain,float setpoint,float intput);
void resetPID(pid__t *t);
#ifdef __cplusplus
}
#endif

#endif
