#include "maths.h"
#include "utils.h"
#include "imu.h"
#include "ibus.h"
#define Dt 0.0125f
// constan variables
static float pi = 3.14159;
static float eart_radius =  6371000;  // m
static float gravity      = -9.81 ;
static float toDeg = 57.29577;
static float toRad = 0.01745;

// dynamic parameters
float air_density     = 1.293;
float mass            = 0.9;         // weight of aircraft
float wing_area       = 0.21;        // m*m  ref area
float wing_ctrl_area  = 0.015;       // control surface area
float rudder_area     = 0.0168;      // rudder surface ares
float Cd_o            = 0.14;        // drag coeffient zero
float Cl_0            = 0.01;
float aileron_Cl      = 0.011;
float aileron_Cd      = 0.0013;
float dis_ruderr2CG   = 0.12;
float dis_aile2center = 0.12;
float dis_ele2center  = 0.2;
float max_aileron_angle = 20;        // max control angle

static float swap180(float val);
static float swap360(float val);

// attitude
extern float cosx,cosy,cosz,sinx,siny, sinz,tany;
extern attitude_t AHRS;
static uint8_t isFlying = 0;
uint8_t pre_run = 1;
float alpha,beta;   // sideslip angle

static float pitch = 0;//10*toRad;
static float yaw = 0;

float vel_estimate_North,vel_estimate_East,vel_estimate_Up;
float pos_estimate_North,pos_estimate_East,pos_estimate_Up;
const float max_thrust = 0.8;   // N
float thrust;


/*
 *   Estimate velocity and postion in earth frame
 **/
void estimates_start(){
    if(pre_run){
        vel_estimate_North = 0, vel_estimate_East = 0,vel_estimate_Up = 0;
        pos_estimate_North = 0, pos_estimate_East = 0,pos_estimate_Up = 0;
        alpha = 0, beta = 0;
        thrust    = 0;
        pre_run = 0;
        return;
    }
    // update thrust
    thrust = ((float)ibusChannelData[CH3] - 1000)/1000*max_thrust;

    // alpha
    float v_horizon = sqrt(vel_estimate_North*vel_estimate_North + vel_estimate_East*vel_estimate_East);
    float temp_a = atan2_approx(vel_estimate_Up,v_horizon)*toDeg;
    temp_a =  AHRS.pitch - temp_a;

     // beta
    float temp_beta  = abs(atan2(vel_estimate_East,vel_estimate_North)*toDeg);
    float beta_t = 0;
    if  (vel_estimate_East >= 0)
        beta_t = temp_beta;
    else if  (vel_estimate_East <= 0)
        beta_t = 360 - temp_beta;
    beta_t = AHRS.yaw - beta_t;  

    if (beta_t < -180)
        beta_t = beta_t + 360;
    else if (beta_t > 180)
        beta_t = beta_t - 360;

    alpha =  temp_a*cosx + beta_t*sinx;
    beta  = -temp_a*sinx + beta_t*cosx;

    float Cd = (pow(abs(alpha),3.7)/125 + alpha)*3/3625 + Cd_o;
    float Cl = 0.01*alpha + Cl_0;
    float Cl_rudder = 0.01*beta;
    //Cl = constrainf(Cl,-1.3,1.3);

    // absolute velocity
    float Vsqr = vel_estimate_North*vel_estimate_North + vel_estimate_East*vel_estimate_East + vel_estimate_Up*vel_estimate_Up;
    float dynamic_p = 0.5*air_density*Vsqr;
    float L =  dynamic_p*wing_area*Cl;
    float D = -dynamic_p*wing_area*Cd *0.5;
    float Side_force = dynamic_p*rudder_area*Cl_rudder;

    float sinA = sin_approx(alpha*toRad);
    float cosA = cos_approx(alpha*toRad);
    float cosB = cos_approx(beta*toRad);
    float sinB = sin_approx(beta*toRad);

    // rotate aero(or wind frame) to body frame
    float Fbx =  L*sinA + D*cosA*cosB + Side_force*cosA*sinB;
    float Fby =  Side_force*cosB - D*sinB;
    float Fbz =  L*cosA - D*cosB*sinA - Side_force*sinA*sinB;

    // rotate gravity to body frame
    float g_bx =  gravity*siny;
    float g_by = -gravity*cosy*sinx;
    float g_bz =  gravity*cosx*cosy;

    Fbx += mass*g_bx + thrust;
    Fby += mass*g_by;
    Fbz += mass*g_bz;

    float acc_bx = Fbx/mass;
    float acc_by = Fby/mass;
    float acc_bz = Fbz/mass;

    // rotate acc body  to inertial frame
    float accEx = acc_bx*cosy*cosz - acc_bz*(sinx*sinz + cosx*cosz*siny) - acc_by*(cosx*sinz - cosz*sinx*siny);
    float accEy = acc_by*(cosx*cosz + sinx*siny*sinz) + acc_bz*(cosz*sinx - cosx*siny*sinz) + acc_bx*cosy*sinz;
    float accEz = acc_bx*siny + acc_bz*cosx*cosy - acc_by*cosy*sinx;

    // zero acce z on ground
    if (accEz > 0 && isFlying == 0)
        isFlying = 1;
    else if (accEz < 0 && isFlying == 0)
        accEz = 0;

    pos_estimate_North += vel_estimate_North*Dt + 0.5*accEx*Dt*Dt;
    pos_estimate_East += vel_estimate_East*Dt + 0.5*accEy*Dt*Dt;
    pos_estimate_Up += vel_estimate_Up*Dt + 0.5*accEz*Dt*Dt;

    //arrow.lat= pos_estimate_North/eart_radius*toDeg + init_latitude;
    //arrow.lon = pos_estimate_East/eart_radius*toDeg + init_longitude;
    //arrow.alt = pos_estimate_Up + init_altitude;

    vel_estimate_North +=  accEx*Dt;
    vel_estimate_East +=  accEy*Dt;
    vel_estimate_Up +=  accEz*Dt;


}