#ifndef DYNAMIC_H
#define DYNAMIC_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
typedef struct dynamic_mode
{
    float roll;
    float pitch;
    float yaw;

    float roll_rate;
    float pitch_rate;
    float yaw_rate;

    float lat;
    float lon;
    float alt;

    float velocity;
}sim_attitude;


void dynamic_control(uint16_t thrust,uint16_t servoL,uint16_t servoR);
void dynamic_loop(float dt);

#ifdef __cplusplus
}
#endif
#endif
