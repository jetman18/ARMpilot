#ifndef _QMC5883_H_
#define _QMC5883_H_
#include "../lib/axis.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

//////////////////////QMC5883L////////////////////////////////////////


void qmc5883_init(I2C_HandleTypeDef *i2cport);
void compass_get(axis3_t *as);
void qmc_get_raw(axis3_t *temp);
#ifdef __cplusplus
}
#endif

#endif

