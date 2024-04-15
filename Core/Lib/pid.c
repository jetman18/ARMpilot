#include "pid.h"
#include "filter.h"
#include "maths.h"
#include "timer.h"
#include "string.h"

#define usTosec(x)    (x *(1e-06f))
#define MAX_WAIT_TIME 500000

void pid_init(pid_t  *pid,float kp, float ki, float kd, float f_cut_D, float maxI,float dt){
  memset(pid,0,sizeof(pid_t));
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->f_cut_D = f_cut_D;
  pid->I_range = maxI;
  pid->last_input = 0;
  pid->D_filted = 0;
  pid->init = 1;
  pid->dt = dt;
}

float pid_cal(pid_t *pid,float input, float setpoint){
   if(pid->init){
       pid->last_input = input;
       pid->init = 0;
       return 0.0f;
   }

   float error = input - setpoint;
   float output = error*pid->kp;

   if(pid->ki > 0){
      pid->i_term += error *pid->ki *pid->dt;
      pid->i_term = constrainf(pid->i_term,-pid->I_range,pid->I_range);
      output += pid->i_term;
   }
   if(pid->kd > 0){
        // low pass filter
        float RC = 1.0f / (2 *M_PIf *pid->f_cut_D);
        float gain_lpf = pid->dt/(RC + pid->dt);
        float delta =  (input - pid->last_input)*pid->kd;
        pid->last_input = input;
        delta /= pid->dt;
        pid->D_filted += gain_lpf*(delta - pid->D_filted);
        output += pid->D_filted;
   }
   return output;
}

void pid_reset(pid_t *pid)
{
	pid->i_term = 0.0f;
	pid->last_input = 0.0f;
    pid->D_filted = 0.0f;
}
