#ifndef _HMC5883_H_
#define _HMC5883_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "../lib/axis.h"
#include "stm32f1xx_hal.h"

typedef struct{
    int16_t mx;
    int16_t my;
    int16_t mz;
    float   heading;
}MAG_t;

void hmc5883_init(I2C_HandleTypeDef *i2cport);
//void magnet_sensor_calibrate();
void hmc_get_raw(axis3_t *as);
#ifdef __cplusplus
}
#endif

#endif

