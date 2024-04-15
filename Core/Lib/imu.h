#ifndef _IMU_H_
#define _IMU_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"
#include "axis.h"

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
}quaternion_t;

typedef struct{
    float roll;
	  float pitch;
    float yaw;

    float roll_rate;
    float pitch_rate;
    float yaw_rate;

    float acc_x;
    float acc_y;
    float acc_z;
}attitude_t;

typedef struct{
    float roll;
    float pitch;
    float yaw;
}euler_t;

typedef struct{
    int16_t accx;
    int16_t accy;
    int16_t accz;

    int16_t gyrox;
    int16_t gyroy;
    int16_t gyroz;
}IMU_raw_t;

typedef struct{
    float gyro_f_cut;
    float acc_f_cut;
    float cpl_gain;
    float gyro_slew_threshold;
    float acc_slew_threshold;
    uint32_t dt;
    float gyr_lsb;
}imu_config_t;
extern attitude_t AHRS;
void ahrs_update();
void imuCalibrate();
#ifdef __cplusplus
}
#endif

#endif
