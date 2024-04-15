#include "pid.h"
#include "filter.h"
#include "maths.h"
#include "timer.h"
#include "string.h"

#define usTosec(x)    (x *(1e-06f))
#define MAX_WAIT_TIME 500000
#define MAX_P_DEFAULT 500.0f
#define MAX_I_DEFAULT 500.0f

void initPid(pid__t  *pid,float kp, float ki, float kd, float f_cut_D, float maxI){
  memset(pid,0,sizeof(pid__t));
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->f_cut_D = f_cut_D;
  pid->max_I = maxI;
}
/*
void pidCalculate(pid__t *pid_temp,float input,float control,uint32_t Dt_time)
{
	float error,P,D;
    float RC = 1.0f / (2 *M_PIf *pid_temp->f_cut_D);
	float gain_lpf = Dt_time*(1e-06f)/(RC + Dt_time*(1e-06f));
	                              
    error =  input - control;
    pid_temp->e = error;
    P  =  error*pid_temp->kp;
	P =  constrainf(P,-pid_temp->max_P,pid_temp->max_P);
    
	error  = fapplyDeadband(error,pid_temp->I_deadband);
    pid_temp->I   += error*pid_temp->ki*Dt_time*TOSEC;
    pid_temp->I   = constrainf(pid_temp->I,-pid_temp->max_I,pid_temp->max_I);

    D  = (input - pid_temp->pre_value)*pid_temp->kd/(Dt_time*TOSEC);
	D =  constrainf(D,-pid_temp->max_D,pid_temp->max_D);

	//Noise filtering for D-term -> low pass filter
    pid_temp->D_smooth = pid_temp->D_smooth* (1-gain_lpf) + gain_lpf*D;

    pid_temp->PID = (P + pid_temp->I + pid_temp->D_smooth);
    pid_temp->PID = constrainf(pid_temp->PID,-pid_temp->max_pid,pid_temp->max_pid);
    pid_temp->pre_value = input;
}
*/
float pidCalculate(pid__t *pid,float setpoint, float input){
   float output;
   uint32_t dt = micros() - pid->last_call_us;
   if(pid->last_call_us == 0 || dt > MAX_WAIT_TIME){
       pid->last_call_us = micros();
       pid->I = 0.0f;
       pid->pre_value =  input;
       return 0.0f;
   }
   float error = setpoint - input;
   output += error*pid->kd;

   if(pid->ki > 0){
      pid->I += error *pid->ki *usTosec(dt);
      if(pid->I > pid->max_I)
           pid->I = pid->max_I;
      else if(pid->I < -pid->max_I)
           pid->I = -pid->max_I;
      output += pid->I;
   }
   if(pid->kd > 0){
        // lowpass filter
        float RC = 1.0f / (2 *M_PIf *pid->f_cut_D);
        float gain_lpf = usTosec(dt)/(RC + usTosec(dt));
        float delta =  (input - pid->pre_value)*pid->kd;
        delta /= usTosec(dt);
        pid->D_filted += gain_lpf*(delta - pid->D_filted);
        output += pid->D_filted;
   }
   return output;
}

void resetPID(pid__t *pid)
{
	pid->I = 0.0f;
	pid->pre_value = 0.0f;
    pid->D_filted = 0.0f;
    pid->pre_value = 0.0f;
}
