#ifndef _SENSORDETECT_
#define _SENSORDETECT_

#ifdef __cplusplus
extern "C" {
#endif
#include "i2c.h"
#define SENSOR_COUNT 5
extern uint8_t sensor_[SENSOR_COUNT];
void i2cDectect(I2C_HandleTypeDef *i2c);
int8_t get_sensor_status(uint8_t address);

#ifdef __cplusplus
}
#endif
#endif
