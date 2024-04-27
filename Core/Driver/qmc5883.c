#include "i2c.h"
#include "math.h"
#include "qmc5883.h"
#include "maths.h"
#include "filter.h"
#include "timer.h"
#include "imu.h"
#include "axis.h"

const uint8_t qmc_addres = (0x0d<<1);
static I2C_HandleTypeDef *qmc_i2cport;

void qmc5883_init(I2C_HandleTypeDef *i2cport){
	qmc_i2cport = i2cport;
    uint8_t buf[2];
    buf[0]=0x0b;
    buf[1]=0X01;
    HAL_I2C_Master_Transmit(qmc_i2cport,qmc_addres,buf,2, 1);
    buf[0]=0x09;
    buf[1]=0X1D;
    HAL_I2C_Master_Transmit(qmc_i2cport,qmc_addres,buf,2, 1);
}

void qmc_get_raw(axis3_t *axis){
	  uint8_t buf[6]={0};
	  HAL_I2C_Mem_Read(qmc_i2cport,qmc_addres,0x00,1,buf,6,1);
	  axis->x=((int16_t)buf[1]<<8|buf[0]) ;
	  axis->y=((int16_t)buf[3]<<8|buf[2]);
	  axis->z=((int16_t)buf[5]<<8|buf[4]);
}

