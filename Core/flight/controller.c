#include "pid.h"
#include "imu.h"
#include "ibus.h"
#include "maths.h"
#include "pwmwrite.h"
#include "timer.h"
#include "plane.h"
#include "utils.h"

float max_pid_value = 1000;
static float max_tilt_angle = 45;  
static float max_intergal_value =  200;
static float Dt = 0.004f;

uint16_t pitch_cmd,roll_cmd;

/*------Roll------------------*/
float roll_kpf = 1;
float roll_kp  = 1;
float roll_ki  = 0.5;
float roll_kd = 0;
float roll_I_term = 0;
float roll_last_rate = 0;
float roll_f_cut_D = 30;
float roll_D_filted = 0;
float roll_rate_limit = 100;

/*------Pitch------------------*/
float pitch_kpf = 1;
float pitch_kp  = 1;
float pitch_ki  = 0.5;
float pitch_kd = 0;
float pitch_I_term = 0;
float pitch_last_rate = 0;
float pitch_f_cut_D = 30;
float pitch_D_filted = 0;
float pitch_rate_limit = 100;

/* axis control with ppid controller
 * input set angle
 * return PID value
 */
int32_t roll_control(float target_roll){
   // angle error
   float Delat_angle = (target_roll - AHRS.roll)*roll_kpf;
   // rate limited
   Delat_angle = constrainf(Delat_angle,-roll_rate_limit,roll_rate_limit);

   // rate error
   float Delta_rate_angle = Delat_angle - AHRS.roll_rate;

   // calculate proportional term
   float P_term =  Delta_rate_angle*roll_kp;

   // calculate derivative term
   float D_temp =  (AHRS.roll_rate - roll_last_rate)/Dt;
   roll_last_rate = AHRS.roll_rate;

   // Apply low pass filter 
   float RC = 1.0f / (2 *M_PIf *roll_f_cut_D);
   float gain_lpf = Dt/(RC + Dt);
   roll_D_filted += gain_lpf*(D_temp - roll_D_filted);
   float D_term = roll_D_filted*roll_kd;

   // calculate intergal term
   roll_I_term += Delta_rate_angle*roll_ki*Dt;
   //float max_i = max_pid_value - P_term - D_term;
   //roll_I_term = constrainf(roll_I_term,-max_i,max_i);
   roll_I_term = constrainf(roll_I_term,-max_intergal_value,max_intergal_value);
   
   int32_t PID = (int32_t)(P_term  + D_term + roll_I_term);
   PID = constrain(PID,-max_pid_value,max_pid_value);
   // Return PID value
   return PID;
}

/* axis control with ppid controller
 * input target angle
 * return PID value
 */
int32_t pitch_control(float target_pitch){
   // angle error
   float Delat_angle = (target_pitch - AHRS.pitch)*pitch_kpf;
   // rate limted
   Delat_angle = constrainf(Delat_angle,-pitch_rate_limit,pitch_rate_limit);

   // rate error
   float Delta_rate_angle = Delat_angle - AHRS.pitch_rate;

   // calculate proportional term
   float P_term =  Delta_rate_angle*pitch_kp;

   // calculate derivative term
   float D_temp =  (AHRS.pitch_rate - pitch_last_rate)/Dt;
   pitch_last_rate = AHRS.pitch_rate;
   // Apply low pass filter 
   float RC = 1.0f / (2 *M_PIf *pitch_f_cut_D);
   float gain_lpf = Dt/(RC + Dt);
   pitch_D_filted += gain_lpf*(D_temp - pitch_D_filted);
   float D_term = pitch_D_filted*pitch_kd;

   // calculate intergal term
   pitch_I_term += Delta_rate_angle*pitch_ki*Dt;
   // float max_i = max_pid_value - P_term - D_term;
   // pitch_I_term = constrainf( pitch_I_term,-max_i,max_i);
   pitch_I_term = constrainf(pitch_I_term,-max_intergal_value,max_intergal_value);
   
   int32_t PID = (int32_t)(P_term  + D_term + roll_I_term);
   PID = constrain(PID,-max_pid_value,max_pid_value);
   // Return PID value
   return PID;
}

int32_t altitudeControl(){


}
int32_t speedControl(){

    
}



void heading_control(float yaw_cmd){


}



