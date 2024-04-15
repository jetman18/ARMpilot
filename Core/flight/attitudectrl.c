#include "plane.h"
#include "pid.h"
#include "imu.h"
#include "maths.h"
#include "pwmwrite.h"
#ifdef SIMULATION
#include "simulation.h"
extern sim_attitude arrow;
#elif
extern attitude_t AHRS;
#endif
#define loop_s 0.01 // 100hz

static pid_t roll_rate,pitch_rate,yaw_rate;
static pid_t roll_a,pitch_a,yaw_a;
float velocity;
float roll_cmd,pitch_cmd;
static float kenh2,kenh3;
void attitude_ctrl_init(){
   // init pid 
   pid_init(&roll_rate,3,0,0.3,20,100,loop_s);
   pid_init(&roll_a,5,0,0,20,100,loop_s);

   pid_init(&pitch_rate,3,0,0.1,20,100,loop_s);
   pid_init(&pitch_a,3,0,0,20,100,loop_s);

   pid_init(&yaw_rate,0,0,0,20,100,loop_s);
   pid_init(&yaw_a,0,0,0,20,100,loop_s);

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
#elif
    roll_r = AHRS.roll_rate;
    pitch_r = AHRS.pitch_rate;
    yaw_r = AHRS.yaw_rate;
    roll = AHRS.roll;
    pitch = AHRS.pitch;
    yaw = AHRS.yaw;
#endif
    // roll axis
    float r_angle_pid = pid_cal(&roll_a,roll,roll_cmd);
    float r_rate_pid  = pid_cal(&roll_rate,r_angle_pid,roll_r);
    // pitch axis
    float p_angle_pid = pid_cal(&pitch_a,pitch,pitch_cmd);
    float p_rate_pid  = pid_cal(&pitch_rate,p_angle_pid,pitch_r);

    float scale_pid = 900/MAX(900,velocity*velocity);
    r_rate_pid = r_rate_pid*scale_pid;
    p_rate_pid = p_rate_pid*scale_pid;

    uint16_t servoL = 1500 + r_rate_pid - p_rate_pid;
    uint16_t servoR = 1500 - r_rate_pid - p_rate_pid;
   
#ifdef MANUAL_CTRL
    kenh2 = 0.5*pow(kenh2,3) + 0.15*kenh2
    kenh3 = 0.5*pow(kenh3,3) + 0.15*kenh3 
    servoL  = 1500 - ch2*500 + ch3*500;
    servoR  = 1500 + ch2*500 + ch3*500;
#endif
    servoL = constrain(servoL,1000,2000);
    servoR = constrain(servoR,1000,2000);
#ifdef SIMULATION
   dynamic_control(1000,servoL,servoR);
   dynamic_loop(loop_s);
   // send attitude to flightgear
   mavlink_send_attitude(arrow.roll,arrow.pitch,arrow.yaw, 
                             arrow.lat,arrow.lon,arrow.alt);
#elif
   writePwm(0,1000); // throtlle
   writePwm(1,servoL);
   writePwm(2,servoR);
#endif

}