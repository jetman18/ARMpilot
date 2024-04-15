#include "i2c.h"
#include "math.h"
#include "hmc5883.h"
#include "maths.h"
#include "filter.h"
#include "timer.h"
#include "imu.h"
#include "axis.h"
#include "blackbox.h"

#define ADRR_CFG_REG_A       0x00
#define ADRR_CFG_REG_B       0x01
#define ADRR_MODE_REG        0x02
#define ADRR_START_DATA_REG  0x03
/* CONFIGURATION FOR REGISTER A */
enum {
  NOSA1 = 0,  // default 
  NOSA2,
  NOSA4,
  NOSA8
}num_of_samples_averaged; //<<5

enum {
   RATE_0_75 = 0,
   RATE_1_5,
   RATE_3,
   RATE_7_5,
   RATE_15,  // Default
   RATE_30,
   RATE_75,
   RATE_REVERSED
}data_ouput_rate;  // <<2 

enum {
  NORMAL,  // default
  POSITIVE,
  NEGATIVE,
  REVERSED
}measure_mode;    // <<0 
/*END CONFIGURATION FOR REGISTER A */


/* CONFIGURATION FOR REGISTER B */
enum {
	GAIN_0_88GA = 0,
	GAIN_1_3GA,   //default
	GAIN_1_9GA,
	GAIN_2_5GA,
	GAIN_4_0GA,
	GAIN_4_7GA,
	GAIN_5_6GA,
	GAIN_8_1GA
}gain_cfg;  // <<5
/*END  CONFIGURATION FOR REGISTER B */

/* CONFIGURATION FOR REGISTER MODE */
enum {
  CONTINUOUS = 0, 
  SINGLE_MEASURE,  // default
  IDLE,
  IDLE1,
}operating_mode;
/* CONFIGURATION FOR REGISTER MODE */


float gyro_yaw;
int16_t calib_axi[3];
int16_t maxval[] = {0,0,0};

const uint8_t hmc_addr = 0x1e<<1;
I2C_HandleTypeDef *qmc_i2cport;
float heading;
uint16_t timeout = 1000; // ms
uint16_t read_timeout = 1000; //ms

/*  
 * Configuration hmc5833l
 */
void hmc5883_init(I2C_HandleTypeDef *i2cport){
	qmc_i2cport = i2cport;
    uint8_t buf[2];
	// Configuration Register A
    buf[0] = (uint8_t)ADRR_CFG_REG_A;
    buf[1] = (NOSA1<<5) | (RATE_75<<2) | (NORMAL<<0);
    HAL_I2C_Master_Transmit(qmc_i2cport,hmc_addr,buf,2,timeout);

	// Configuration Register B
    buf[0] = (uint8_t)ADRR_CFG_REG_B;
    buf[1] = (GAIN_1_3GA<<5);
    HAL_I2C_Master_Transmit(qmc_i2cport,hmc_addr,buf,2,timeout);

	// Configuration Register Mode
    buf[0] = (uint8_t) ADRR_MODE_REG;
    buf[1] = (CONTINUOUS<<0);
    HAL_I2C_Master_Transmit(qmc_i2cport,hmc_addr,buf,2,timeout);
}


void hmc_get_raw(axis3_t *as){
	uint8_t buf[6]={0};
	HAL_I2C_Mem_Read(qmc_i2cport,hmc_addr,(uint8_t)ADRR_START_DATA_REG ,1,buf,6,read_timeout);
	as->x=((int16_t)buf[0]<<8|buf[1]);
	as->z=((int16_t)buf[2]<<8|buf[3]);
	as->y=((int16_t)buf[4]<<8|buf[5]);
}


/*
void magnet_sensor_calibrate(){

	int16_t mx,my,mz;
	uint8_t buf[6];
	for(int i=0;i<4000;i++){
			HAL_I2C_Mem_Read(qmc_i2cport,hmc_addr,(uint8_t)ADRR_START_DATA_REG,1,buf,6,read_timeout);
			mx=(int16_t)(buf[1])<<8|buf[0];
			my=(int16_t)(buf[3])<<8|buf[2];
			mz=(int16_t)(buf[5])<<8|buf[4];

			if(mx > max_val[0]) max_val[0] = mx;
			if(mx < min_val[0]) min_val[0] = mx;

			if(my > max_val[1]) max_val[1] = my;
			if(my < min_val[1]) min_val[1] = my;

			if(mz > max_val[2]) max_val[2] = mz;
			if(mz < min_val[2]) min_val[2] = mz;
	        HAL_Delay(5);
    }
	for(int i=0;i<3;i++){
		if((max_val[i]*min_val[i]) < 0){
			calib_axi[i] = (max_val[i] + min_val[i])/2;
		}
		else{
		    calib_axi[i] = (max_val[i] - min_val[i])/2;
		}
	}
}
*/

