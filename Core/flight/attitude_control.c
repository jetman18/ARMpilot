#include "plane.h"
#include "pid.h"
#include "imu.h"
#include "maths.h"
#include "pwm.h"
#include "timer.h"
#include "ibus.h"
#define loop_s 0.02 // 100h

extern attitude_t AHRS;

float roll_cmd;
float pitch_cmd;

static pid_t roll_rate,pitch_rate,yaw_rate;
static pid_t roll_angle,pitch_angle,yaw_angle;
static float roll_pid_filted= 0,pitch_pid_filted = 0;
static float roll_trim = 0,pitch_trim = 10;

static float last_ab_velocity = 0;
static float absolute_velocity_filter;
static uint8_t gps_lost;

float const acc_threshold; 
uint16_t servoL,servoR;
static int16_t smooth_ch1=0, smooth_ch2=0;


void attitude_ctrl_init(){
   gps_lost = 1;
   // init pid 
   pid_init(&roll_rate,3,0,0,100,100,loop_s);
   pid_init(&roll_angle,10,0,0,100,100,loop_s);

   pid_init(&pitch_rate,3,0,0,100,100,loop_s);
   pid_init(&pitch_angle,10,0,0,100,100,loop_s);

}

void attitude_ctrl(){ 

    float roll_r = AHRS.roll_rate;
    float pitch_r = AHRS.pitch_rate;
    float yaw_r = AHRS.yaw_rate;
    float roll = AHRS.roll;
    float pitch = AHRS.pitch;
    float yaw = AHRS.yaw;
    float roll_cmd = ((int)ibusChannelData[0] - 1500)*0.1f;
	float pitch_cmd = ((int)ibusChannelData[1] - 1500)*-0.1f;


    /*
    // pid scale with gps velocity 
    if(_gps.fix > 1){
        float vn = (float)_gps.velocity[0]/100;  // m
        float ve = (float)_gps.velocity[1]/100;  // m
        //float vd = (float)_gps.velocity[3]/100;

        float absolute_velocity = sqrtf(vn*vn + ve*ve);
        if(gps_lost){
            last_ab_velocity = absolute_velocity;
            gps_lost = 0;
        }
        // max speed 120 km/h -> 33 m/s
        absolute_velocity = constrainf(absolute_velocity,0,33); 
        // calculate acceleration 
        float acc_ = (absolute_velocity - last_ab_velocity)/Dt;
        // apply threshold
        if (abs(acc_) > acc_threshold){
            absolute_velocity = last_ab_velocity + sign(acc_)*0.3f;
        }
        // apply filter 
        absolute_velocity_filter += 0.4f*(absolute_velocity - absolute_velocity_filter);
        last_ab_velocity = absolute_velocity;
        
    }
    else{
        gps_lost = 1;
    }
    */
    // stablize mode
    if(ibusChannelData[CH5] > 1600 ){

        float roll_pid_gain = ((int)ibusChannelData[CH7] - 1000)*0.001f;
		float pitch_pid_gain = ((int)ibusChannelData[CH8] - 1000)*0.001f;
        
        // roll axis
        float r_angle_pid = pid_cal(&roll_angle,roll,roll_cmd + roll_trim);
        float r_rate_pid  = -pid_cal(&roll_rate,-roll_r,r_angle_pid);
        //pitch axis
        float p_angle_pid = pid_cal(&pitch_angle,pitch,pitch_cmd + pitch_trim);
        float p_rate_pid  = -pid_cal(&pitch_rate,-pitch_r,p_angle_pid);
       

        float scale_velocity = 1.0;
       
		
        r_rate_pid = r_rate_pid*scale_velocity*roll_pid_gain;
        p_rate_pid = p_rate_pid*scale_velocity*pitch_pid_gain;
        
        roll_pid_filted  += 0.4*(r_rate_pid - roll_pid_filted);
        pitch_pid_filted += 0.4*(p_rate_pid - pitch_pid_filted);
		
		int s1 = 1500 - ibusChannelData[CH2];

        servoL = 1500 + roll_pid_filted + s1;// - pitch_pid_filted;
        servoR = 1500 - roll_pid_filted + s1;// - pitch_pid_filted;
        
    }
    // manual mode
    else{
        int s1 = 1500 - ibusChannelData[CH1];
        int s2 = 1500 - ibusChannelData[CH2];

        smooth_ch1 += 0.8*(s1 - smooth_ch1);
        smooth_ch2 += 0.8*(s2 - smooth_ch2);
            
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



