#ifndef _AXIS_H_
#define _AXIS_H_
#include "stdio.h"
#include "stm32f1xx_hal.h"
typedef enum {
    X = 0,
    Y,
    Z
} axis_e;

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
}axis3_t;

typedef struct{
    float x;
    float y;
    float z;
}faxis3_t;


#define XYZ_AXIS_COUNT 3

typedef enum {
    FD_ROLL = 0,
    FD_PITCH,
    FD_YAW
} flight_dynamics_index_t;

#define FLIGHT_DYNAMICS_INDEX_COUNT 3

typedef enum {
    AI_ROLL = 0,
    AI_PITCH
} angle_index_t;

#define ANGLE_INDEX_COUNT 2

#define GET_DIRECTION(isReversed) ((isReversed) ? -1 : 1)

#endif
