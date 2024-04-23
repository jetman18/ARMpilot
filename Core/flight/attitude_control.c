#include "plane.h"
#include "pid.h"
#include "imu.h"
#include "maths.h"
#include "pwm.h"
#include "timer.h"
#include "ibus.h"
#define loop_s 0.01 // 100h


#ifdef SIMULATION
extern attitude_t AHRS;
#include "simulation.h"
   extern sim_attitude arrow;
#else
extern attitude_t AHRS;
#endif
float roll_cmd;
float pitch_cmd;
static pid_t roll_rate,pitch_rate,yaw_rate;
static pid_t roll_angle,pitch_angle,yaw_angle;

void attitude_ctrl_init(){
   // init pid 
   pid_init(&roll_rate,3,0,0,100,100,loop_s);
   pid_init(&roll_angle,10,0,0,100,100,loop_s);

   pid_init(&pitch_rate,3,0,0,100,100,loop_s);
   pid_init(&pitch_angle,10,0,0,100,100,loop_s);


}

// 100 hz
void attitude_ctrl(){ 
#ifdef SIMULATION
    float roll_r = arrow.roll_rate;
    float pitch_r = arrow.pitch_rate;
    float yaw_r = arrow.yaw_rate;
    float roll = arrow.roll;
    float pitch = arrow.pitch;
    float yaw = arrow.yaw;
    float velocity = arrow.velocity;
#else
    float roll_r = AHRS.roll_rate;
    float pitch_r = AHRS.pitch_rate;
    float yaw_r = AHRS.yaw_rate;
    float roll = AHRS.roll;
    float pitch = AHRS.pitch;
    float yaw = AHRS.yaw;
    float velocity = 0;
    float roll_cmd = ((int)ibusChannelData[0] - 1500)*0.1f;
	float pitch_cmd = ((int)ibusChannelData[1] - 1500)*-0.1f;
#endif
    
	uint16_t servoL,servoR;
	static float roll_pid_filted= 0,pitch_pid_filted = 0;
    static float roll_trim = 0,pitch_trim = 10;
	 static int16_t smooth_ch1=0, smooth_ch2=0;
    if(ibusChannelData[CH5] > 1600 ){

        float roll_pid_gain = ((int)ibusChannelData[CH7] - 1000)*0.001f;
		float pitch_pid_gain = ((int)ibusChannelData[CH8] - 1000)*0.001f;
        
        // roll axis
        float r_angle_pid = pid_cal(&roll_angle,roll,roll_cmd + roll_trim);
        float r_rate_pid  = -pid_cal(&roll_rate,-roll_r,r_angle_pid);
        //pitch axis
        float p_angle_pid = pid_cal(&pitch_angle,pitch,pitch_cmd + pitch_trim);
        float p_rate_pid  = -pid_cal(&pitch_rate,-pitch_r,p_angle_pid);

        // need to fix
        float scale_pid = 900.0f/MAX(900,velocity*velocity);
        scale_pid = 1;
        
        r_rate_pid = r_rate_pid*scale_pid*roll_pid_gain;
        p_rate_pid = p_rate_pid*scale_pid*pitch_pid_gain;
        
        roll_pid_filted  += 0.4*(r_rate_pid - roll_pid_filted);
        pitch_pid_filted += 0.4*(p_rate_pid - pitch_pid_filted);

        servoL = 1500 + roll_pid_filted - pitch_pid_filted;
        servoR = 1500 - roll_pid_filted - pitch_pid_filted;
        
    }
    // manual mode
 
    else{
        int s1 = 1500 - ibusChannelData[CH1];
        int s2 = 1500 - ibusChannelData[CH2];

        smooth_ch1 += 0.5*(s1 - smooth_ch1);
        smooth_ch2 += 0.5*(s2 - smooth_ch2);
            
        servoL = 1500 + smooth_ch1 + smooth_ch2;
        servoR = 1500 - smooth_ch1 + smooth_ch2;
        
    }


//#define MANUAL_CTRL

#ifdef MANUAL_CTRL_JOYSTICK
    float ch2 = AHRS.roll/50;
	float ch3 = AHRS.ppitch/50;

    ch2 = 0.5*pow(ch2,3) + 0.15*ch2;
    ch3 = 0.5*pow(ch3,3) + 0.15*ch3; 
    servoL  = 1500 - ch2*500 + ch3*500;
    servoR  = 1500 + ch2*500 + ch3*500;
#endif
    servoL = constrain(servoL,1000,2000);
    servoR = constrain(servoR,1000,2000);

#ifdef SIMULATION
   dynamic_control(2000,servoL,servoR);
   dynamic_loop(loop_s);
   // send attitude to flightgear
   mavlink_send_attitude(arrow.roll,arrow.pitch,arrow.yaw, 
                          arrow.lat,arrow.lon,arrow.alt);
   // mavlink_send_attitude(arrow.roll,0,arrow.yaw, 
   //                       37.472,-121.170,90);
#else

write_pwm_ctrl(ibusChannelData[CH3],servoL,servoR);
#endif

}



