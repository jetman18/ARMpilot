#ifndef _MS5611_H_
#define _MS5611_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "i2c.h"
extern int ms5611_altitude;
extern int32_t ms5611_pressure;
void  ms5611_init(I2C_HandleTypeDef *hi2c2);
void ms5611_start();
#ifdef __cplusplus
}
#endif
#endif
