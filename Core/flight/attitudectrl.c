#include "plane.h"
#include "pid.h"
#include "imu.h"
#include "maths.h"
#include "pwmwrite.h"

#ifdef SIMULATION
#include "simulation.h"
extern sim_attitude arrow;
extern attitude_t AHRS;
#else
extern attitude_t AHRS;
#endif
#define loop_s 0.02 // 100hz

static pid_t roll_rate,pitch_rate,yaw_rate;
static pid_t roll_,pitch_,yaw_;
static float velocity;
float roll_cmd,pitch_cmd;
static float kenh2,kenh3;

void attitude_ctrl_init(){
   // init pid 
   pid_init(&roll_rate,1,0,1,100,100,loop_s);
   pid_init(&roll_,3,0,0,100,100,loop_s);

   pid_init(&pitch_rate,3,0,2,100,100,loop_s);
   pid_init(&pitch_,3,0,0,100,100,loop_s);

   //pid_init(&yaw_rate,0,0,0,20,100,loop_s);
   //pid_init(&yaw_,0,0,0,20,100,loop_s);

}

// 100 hz
void attitude_ctrl(){ 
	

    float roll_r,pitch_r,yaw_r;
    float roll,pitch,yaw;

#ifdef SIMULATION
    roll_r = arrow.roll_rate;
    pitch_r = arrow.pitch_rate;
    yaw_r = arrow.yaw_rate;
    roll = arrow.roll;
    pitch = arrow.pitch;
    yaw = arrow.yaw;
    velocity = arrow.velocity;
#else
    roll_r = AHRS.roll_rate;
    pitch_r = AHRS.pitch_rate;
    yaw_r = AHRS.yaw_rate;
    roll = AHRS.roll;
    pitch = AHRS.pitch;
    yaw = AHRS.yaw;
#endif
	
    // roll axis
    float r_angle_pid = pid_cal(&roll_,roll,AHRS.r);
    float r_rate_pid  = pid_cal(&roll_rate,r_angle_pid,roll_r);
    //pitch axis
    float p_angle_pid = pid_cal(&pitch_,pitch,AHRS.p);
    float p_rate_pid  = pid_cal(&pitch_rate,p_angle_pid,pitch_r);

	float scale_pid = 900.0f/MAX(900,velocity*velocity);

    r_rate_pid = r_rate_pid*scale_pid;
    p_rate_pid = p_rate_pid*scale_pid;

    uint16_t servoL = 1500 + r_rate_pid - p_rate_pid;
    uint16_t servoR = 1500 - r_rate_pid - p_rate_pid;


//#define MANUAL_CTRL

#ifdef MANUAL_CTRL
    float ch2 = AHRS.r/50;
	float ch3 = AHRS.p/50;

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
   writePwm(0,1000); // throtlle
   writePwm(1,servoL);
   writePwm(2,servoR);
#endif

}



