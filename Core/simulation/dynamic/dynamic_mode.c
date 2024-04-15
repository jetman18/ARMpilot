#include"dynamic_mode.h"
#include "maths.h"
#include "utils.h"
#define Dt 0.01f
// constan variables
static float pi = 3.14159;
float eart_radius =  6371000;  // m
static float gravity      = -9.81 ;
static float toDeg = 57.29577;
static float toRad = 0.01745;

// dynamic parameters
float air_density     = 1.293;
float mass            = 0.9;         // weight of aircraft
float wing_area       = 0.21;        // m*m  ref area
float wing_ctrl_area  = 0.015;       // control surface area
float Cd_o            = 0.14;        // drag coeffient zero
float aileron_Cl      = 0.011;
float aileron_Cd      = 0.0013;
float dis_aile2center = 0.12;
float dis_ele2center  = 0.2;
float max_aileron_angle = 20;        // max control angle

float cd_moment_x     = 0.05;
float cd_moment_y     = 0.1;
float cd_moment_z     = 0.012;
float Cm_o            = -0.0;
float Ixx             = 0.0106;
float Iyy             = 0.018;
float Izz             = 0.0251;
float init_latitude  = 37.628715674334124;  //deg
float init_longitude = -122.39334575867426; //deg
float init_altitude  =  500;                //m

static float swap180(float val);
static float swap360(float val);

// attitude
uint8_t isFly = 0;
float alpha = 0;  // attack of angle
float beta = 0;   // sideslip angle

float roll = 0;
float pitch = 0;//10*toRad;
float yaw = 0;//358*toRad;

float vx,vy,vz;
float px,py,pz;
float P,Q,R;

float T = 10;  // thrust
float ctrl_left = 0;
float ctrl_right = 0;


void dynamic_loop(float ch2,float ch3){
    float cosx,cosy,cosz,sinx,siny,sinz;
    float tany;
    cosx = cos_approx(roll);
    cosy = cos_approx(pitch);
    cosz = cos_approx(yaw);
    sinx = sin_approx(roll);
    siny = sin_approx(pitch);
    sinz = sin_approx(yaw);
    tany = tan_approx(pitch);

    // alpha
    float v_horizon = sqrt(vx*vx + vy*vy);
    float temp_a = atan2_approx(vz,v_horizon)*toDeg;
    temp_a =  pitch*toDeg - temp_a;

    // beta
    float temp_beta  = abs(atan2(vy,vx)*toDeg);
    float beta_t = 0;
    if  (vy >= 0)
        beta_t = temp_beta;
    else if  (vy <= 0)
        beta_t = 360 - temp_beta;
    beta_t = yaw*toDeg - beta_t;  // beta 0 - 359

    alpha =  temp_a*cosx + beta_t*sinx;
    beta  = -temp_a*sinx + beta_t*cosx;

    float Cd = (pow(abs(alpha),3.7)/125 + alpha)*3/3625 + Cd_o;
    float Cl = 0.01*alpha;
    Cl = constrainf(Cl,-1.3,1.3);

    // absolute velocity
    float Vsqr = vx*vx + vy*vy + vz*vz;
    float dynamic_p = 0.5*air_density*Vsqr;
    float L =  dynamic_p*wing_area*Cl;
    float D = -dynamic_p*wing_area*Cd *0.5;

    float sinA = sin_approx(alpha*toRad);
    float cosA = cos_approx(alpha*toRad);
    float cosB = cos_approx(beta*toRad);
    float sinB = sin_approx(beta*toRad);

    // rotate aero(or wind frame) to body frame
    float Fbx =  L*sinA + D*cosA*cosB + T;
    float Fby = -D*sinB;
    float Fbz =  L*cosA - D*cosB*sinA;

    // rotate body frame to inertial frame
    float Fex = Fbx*cosy*cosz - Fbz*(sinx*sinz + cosx*cosz*siny) - Fby*(cosx*sinz - cosz*sinx*siny);
    float Fey = Fby*(cosx*cosz + sinx*siny*sinz) + Fbz*(cosz*sinx - cosx*siny*sinz) + Fbx*cosy*sinz;
    float Fez = Fbx*siny + Fbz*cosx*cosy - Fby*cosy*sinx + mass*gravity;

    float accEx = Fex/mass;
    float accEy = Fey/mass;
    float accEz = Fez/mass;

    // zero acce z on ground
    if (accEz > 0 && isFly == 0)
        isFly = 1;
    else if (accEz < 0 && isFly == 0)
        accEz = 0;

    px += vx*Dt + 0.5*accEx*Dt*Dt;
    py += vy*Dt + 0.5*accEy*Dt*Dt;
    pz += vz*Dt + 0.5*accEz*Dt*Dt;

    vx +=  accEx*Dt;
    vy +=  accEy*Dt;
    vz +=  accEz*Dt;


   
    /* moment
      -  <---- CH2 -----> +
                +
                |
               ch3
                |
                -
    */

    ctrl_left  = -ch2 + ch3;
    ctrl_right =  ch2 + ch3;
    ctrl_left = constrainf(ctrl_left,-1,1);
    ctrl_right = constrainf(ctrl_right,-1,1);
    //scale to deg
    ctrl_left  *= 20;
    ctrl_right *= 20;

    float lift_left   = dynamic_p*wing_ctrl_area*aileron_Cl*ctrl_left;
    float lift_right  = dynamic_p*wing_ctrl_area*aileron_Cl*ctrl_right;
    float drag_left   = dynamic_p*wing_ctrl_area*aileron_Cd*ctrl_left;
    float drag_right  = dynamic_p*wing_ctrl_area*aileron_Cd*ctrl_right;

    // pitching moment
    float Cm_p = (0.002f*pow(alpha,3) + 0.2f*alpha)*0.0002f;
    float Pitching_moment = dynamic_p*wing_area*Cm_p;
    float yaw_st = dynamic_p*0.01f*beta*0.01f;
    float Mx_total = (lift_right - lift_left)*dis_aile2center -sign(P)*P*P*cd_moment_x;
    float My_total = (lift_right + lift_left)*dis_ele2center - Pitching_moment  -sign(Q)*Q*Q*cd_moment_y;
    float Mz_total = (fabs(drag_left) - fabs(drag_right))*dis_aile2center - yaw_st*0.01f  - sign(R)*R*R*cd_moment_z;

    float P_dot = Mx_total/Ixx;
    float Q_dot = My_total/Iyy;
    float R_dot = Mz_total/Izz;
	
    P += P_dot*Dt;
    Q += Q_dot*Dt;
    R += R_dot*Dt;

    // cvt body rate to euler rate
    float r_dot   = P + R*cosx*tany + Q*sinx*tany;
    float p_dot   = Q*cosx - R*sinx;
    float y_dot   = R*cosx/cosy + Q*sinx/cosy;

    roll  += r_dot*Dt;
    pitch += p_dot*Dt;
    yaw   += y_dot*Dt;

    yaw   = swap360(yaw*toDeg)*toRad;
    roll  = swap180(roll*toDeg)*toRad;
    pitch = swap180(pitch*toDeg)*toRad;

}


static float swap180(float val){
    if(val > 179)
        val = -179;
    else if (val < -179)
        val = 179;
    return val;
}

static float swap360(float val){
    if(val > 359)
        val = 0;
    else if (val < 0)
        val = 359;
    return val;
}