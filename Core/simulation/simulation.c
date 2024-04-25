/*
#include"simulation.h"
#include "maths.h"
#include "utils.h"
#define Dt 0.02f
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

float cd_moment_x     = 0.05;
float cd_moment_y     = 0.1;
float cd_moment_z     = 0.2;
float Cm_o            = -0.0;
float Ixx             = 0.0106;
float Iyy             = 0.018;
float Izz             = 0.0251;
//float init_latitude  = 37.628715674334124;  //deg
//float init_longitude = -122.39334575867426; //deg
float init_latitude  = 37.472;
float init_longitude = -121.170;
float init_altitude  =  50;                //m

static float swap180(float val);
static float swap360(float val);

// attitude
static uint8_t isFlying = 0;
uint8_t pre_run = 1;
float alpha,beta;   // sideslip angle

static float roll = 0;
static float pitch = 0;//10*toRad;
static float yaw = 3.1415;

static float vex,vey,vez;
static float pex,pey,pez;
static float P,Q,R;

static float T;  // thrust
static float T_max = 10;
static float ctrl_left = 0;
static float ctrl_right = 0;
sim_attitude arrow;
void dynamic_control(uint16_t thrust,uint16_t servoL,uint16_t servoR)
{
    ctrl_left  = (servoL - 1500)/500.0f;
    ctrl_right = (servoR - 1500)/500.0f;
    T = T_max*(thrust - 1000)/1000.0f;
}

void dynamic_loop(float dt){
    if(pre_run){
        vex = 0, vey = 0,vez = 0;
        pex = 0, pey = 0,pez = 0;
        P   = 0, Q   = 0, R = 0;
        alpha = 0, beta = 0;
        T    = 0;
        pre_run = 0;
        return;
    }
    float cosx = cos_approx(roll);
    float cosy = cos_approx(pitch);
    float cosz = cos_approx(yaw);
    float sinx = sin_approx(roll);
    float siny = sin_approx(pitch);
    float sinz = sin_approx(yaw);
    float tany = tan_approx(pitch);

    // alpha
    float v_horizon = sqrt(vex*vex + vey*vey);
    float temp_a = atan2_approx(vez,v_horizon)*toDeg;
    temp_a =  pitch*toDeg - temp_a;

    // beta
    float temp_beta  = abs(atan2(vey,vex)*toDeg);
    float beta_t = 0;
    if  (vey >= 0)
        beta_t = temp_beta;
    else if  (vey <= 0)
        beta_t = 360 - temp_beta;
    beta_t = yaw*toDeg - beta_t;  // beta 0 - 359

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
    float Vsqr = vex*vex + vey*vey + vez*vez;
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

    Fbx += mass*g_bx + T;
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

    pex += vex*Dt + 0.5*accEx*Dt*Dt;
    pey += vey*Dt + 0.5*accEy*Dt*Dt;
    pez += vez*Dt + 0.5*accEz*Dt*Dt;

    arrow.lat= pex/eart_radius*toDeg + init_latitude;
    arrow.lon = pey/eart_radius*toDeg + init_longitude;
    arrow.alt = pez + init_altitude;

    vex +=  accEx*Dt;
    vey +=  accEy*Dt;
    vez +=  accEz*Dt;


   
     // moment
      -  <---- CH2 -----> +
                +
                |
               ch3
                |
                -
    
    //scale to deg
    ctrl_left  *= 40;
    ctrl_right *= 40;

    float lift_left   = dynamic_p*wing_ctrl_area*aileron_Cl*ctrl_left;
    float lift_right  = dynamic_p*wing_ctrl_area*aileron_Cl*ctrl_right;
    float drag_left   = dynamic_p*wing_ctrl_area*aileron_Cd*ctrl_left;
    float drag_right  = dynamic_p*wing_ctrl_area*aileron_Cd*ctrl_right;

    // pitching moment coefficient
    float Cm_p = (0.002f*pow(alpha,3) + 0.2f*alpha)*0.0002f;
    float pitching_moment = dynamic_p*wing_area*Cm_p;
    float yawing_moment  = Side_force*dis_ruderr2CG;
    float Mx_total = (lift_right - lift_left)*dis_aile2center -sign(P)*P*P*cd_moment_x;
    float My_total = (lift_right + lift_left)*dis_ele2center - pitching_moment  -sign(Q)*Q*Q*cd_moment_y;
    float Mz_total = (fabs(drag_left) - fabs(drag_right))*dis_aile2center - yawing_moment  - sign(R)*R*R*cd_moment_z;

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

    arrow.velocity = sqrtf(Vsqr);

    arrow.roll = roll*toDeg;
    arrow.pitch= pitch*toDeg;
    arrow.yaw = yaw*toDeg;

    arrow.roll_rate  = r_dot*toDeg;
    arrow.pitch_rate = p_dot*toDeg;
    arrow.yaw_rate   = y_dot*toDeg;
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
*/