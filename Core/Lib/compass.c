#include "stm32f1xx.h"
#include "compass.h"
#include "hmc5883.h"
#include "qmc5883.h"
#include "blackbox.h"
#include "maths.h"
#include "i2c.h"
#include "timer.h"
#include "gpio.h"
#include "faulthandler.h"
#include "string.h"
#include "axis.h"
#include "AT24Cxx_stm32_hal.h"

#define QMC5883

// blackbox store calirbate value
black_box_file_t calib_file;

static uint8_t is_calibrated = FALSE;
char file_name[] = "compassdata.txt";

const int16_t max_change = 8000;
uint16_t ignore_data;
 AT24Cxx_devices_t device_array;

static void compass_calibrate();
static void read_calibrate_file();


typedef struct{
   int32_t hard_iron_calibrate_value[3];
   float scale_factor_axis[3];
   int32_t sum_all_value;
}cali_mag_t;

cali_mag_t calibrate_value;
int8_t file_open;


/*  Init compass
 */
void compassInit(){
  calibrate_value.scale_factor_axis[X] = 1.0f;
  calibrate_value.scale_factor_axis[Y] = 1.0f;
  calibrate_value.scale_factor_axis[Z] = 1.0f;

  calibrate_value.hard_iron_calibrate_value[X] = 0;
  calibrate_value.hard_iron_calibrate_value[Y] = 0;
  calibrate_value.hard_iron_calibrate_value[Z] = 0;
	
  file_open = black_box_create_file(&calib_file,file_name);
	 
  // epprom init
  AT24Cxx_init(&device_array, 0x00, &hi2c2);
  AT24Cxx_add_dev(&device_array, 0x01, &hi2c2);

  // init sensor
#ifdef QMC5883
  qmc5883_init(&hi2c1);
#else
  hmc5883_init(&hi2c2);
#endif
  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)){
      compass_calibrate();
  }
  else{
      read_calibrate_file();
  }
}

void compass_get(axis3_t *out){
	axis3_t as;
#ifdef QMC5883
         qmc_get_raw(&as);
#else
         hmc_get_raw(&as);
#endif
    // calibrate ouput
	out->x = (as.x - calibrate_value.hard_iron_calibrate_value[X]);
	out->y = (as.y - calibrate_value.hard_iron_calibrate_value[Y]);
	out->z = (as.z - calibrate_value.hard_iron_calibrate_value[Z]);

    out->x *= calibrate_value.scale_factor_axis[X];
    out->y *= calibrate_value.scale_factor_axis[Y];
    out->z *= calibrate_value.scale_factor_axis[Z];
}

void test_compass(){
  axis3_t out;
  compass_get(&out);
	
  black_box_pack_int(&calib_file,(int)out.x);
  black_box_pack_str(&calib_file," ");
  black_box_pack_int(&calib_file,(int)out.y);
  black_box_pack_str(&calib_file," ");
  black_box_pack_int(&calib_file,(int)out.z);
  black_box_pack_str(&calib_file,"\n");

  black_box_load(&calib_file);
  black_box_sync(&calib_file);

}



/* read calibrate data from eprrom
 * 
 */

void read_calibrate_file(){
	int32_t sum_all = 0;
    AT24Cxx_read_byte_buffer(device_array.devices[0],(uint8_t*)&calibrate_value,0x0010,sizeof(cali_mag_t));
   	sum_all += calibrate_value.scale_factor_axis[X];
    sum_all += calibrate_value.scale_factor_axis[Y];
    sum_all += calibrate_value.scale_factor_axis[Z];

    sum_all += calibrate_value.hard_iron_calibrate_value[X];
    sum_all += calibrate_value.hard_iron_calibrate_value[Y];
    sum_all += calibrate_value.hard_iron_calibrate_value[Z];

    if(ABS(sum_all - calibrate_value.sum_all_value) > 40){
      while(1){
	   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
       HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
       HAL_Delay(50); // 10 Hz loop
      }
    }

}


/* Calibrate function
 * write calibrate value to sd card
 */
static void compass_calibrate(){
    int16_t max_val[] = {-32767,-32767,-32767};
    int16_t min_val[] = {32767, 32767, 32767};
    uint8_t fist_data = TRUE;
    cali_mag_t calibrate_temp;
    ignore_data = 0;
    int16_t last_axis[3];
    axis3_t as;

	while(!file_open){
 // read data from sensor
#ifdef QMC5883
         qmc_get_raw(&as);
#else
         hmc_get_raw(&as);
#endif
        if(fist_data){
          last_axis[X] = as.x;
          last_axis[Y] = as.y;
          last_axis[Z] = as.z;
          fist_data = FALSE;
          continue;
        }
        int16_t delta_x_ = as.x -  last_axis[X];
        int16_t delta_y_ = as.y -  last_axis[Y];
        int16_t delta_z_ = as.z -  last_axis[Z];
        // ignore wrong value and set to zero
        int16_t ckec = sqrt(sq(delta_x_) + sq(delta_y_) + sq(delta_z_));
        if(ckec > max_change){
            fist_data = TRUE; 
            ignore_data ++;
            continue;
        }
        last_axis[X] = as.x;
        last_axis[Y] = as.y;
        last_axis[Z] = as.z;
        // get max value each axis
        if(as.x > max_val[X]) max_val[X] = as.x;
        if(as.y > max_val[Y]) max_val[Y] = as.y;
        if(as.z > max_val[Z]) max_val[Z] = as.z;

        // min value
        if(as.x < min_val[X]) min_val[X] = as.x;
        if(as.y < min_val[Y]) min_val[Y] = as.y;
        if(as.z < min_val[Z]) min_val[Z] = as.z;
		
	    // write data to sc card
        
        black_box_pack_int(&calib_file,(int)as.x);
        black_box_pack_str(&calib_file," ");
        black_box_pack_int(&calib_file,(int)as.y);
        black_box_pack_str(&calib_file," ");
        black_box_pack_int(&calib_file,(int)as.z);
        black_box_pack_str(&calib_file,"\n");
		
		black_box_load(&calib_file);
        black_box_sync(&calib_file);
        
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
        HAL_Delay(20); // 10 Hz loop

       if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_5) == 0){
              break;
          }
	  }

    // check all min value is negative sign
    if(min_val[X] > 0 || min_val[Y] > 0 || min_val[Z] > 0){
          // error
    }
    // check all max value is positive sign
    if(max_val[X] < 0 || max_val[Y] < 0 || max_val[Z] < 0){
          // error
    }
    int calibrate_data[3];
    // calibrate value for each axis
    calibrate_temp.hard_iron_calibrate_value[X] = (int)(max_val[X] + min_val[X])/2;
    calibrate_temp.hard_iron_calibrate_value[Y] = (int)(max_val[Y] + min_val[Y])/2;
    calibrate_temp.hard_iron_calibrate_value[Z] = (int)(max_val[Z] + min_val[Z])/2;

    // calculate scale factor for each axis
    int16_t x_ = abs(max_val[X]) + abs(min_val[X]);
    int16_t y_ = abs(max_val[Y]) + abs(min_val[Y]);
    int16_t z_ = abs(max_val[Z]) + abs(min_val[Z]);

    // get largest value
    int16_t max_value = 0;
    if(x_ > y_)
        max_value = x_;
    else
        max_value = y_;
    if(max_value < z_)
        max_value = z_;

    // caculate scale
    calibrate_temp.scale_factor_axis[X] = (float)x_/max_value;
    calibrate_temp.scale_factor_axis[Y] = (float)y_/max_value;
    calibrate_temp.scale_factor_axis[Z] = (float)z_/max_value;

	calibrate_temp.sum_all_value += calibrate_temp.scale_factor_axis[X];
    calibrate_temp.sum_all_value += calibrate_temp.scale_factor_axis[Y];
    calibrate_temp.sum_all_value += calibrate_temp.scale_factor_axis[Z];

    calibrate_temp.sum_all_value += calibrate_temp.hard_iron_calibrate_value[X];
    calibrate_temp.sum_all_value += calibrate_temp.hard_iron_calibrate_value[Y];
    calibrate_temp.sum_all_value += calibrate_temp.hard_iron_calibrate_value[Z];

    // write data to eprrom
    AT24Cxx_write_byte_buffer(device_array.devices[0],(uint8_t*)&calibrate_temp, 0x0010, sizeof(cali_mag_t));
	
	// write data to sc card
	black_box_pack_str(&calib_file,"calibrate \n");
    black_box_pack_int(&calib_file, calibrate_temp.hard_iron_calibrate_value[X]);
    black_box_pack_str(&calib_file," ");
    black_box_pack_int(&calib_file, calibrate_temp.hard_iron_calibrate_value[Y]);
    black_box_pack_str(&calib_file," ");
    black_box_pack_int(&calib_file, calibrate_temp.hard_iron_calibrate_value[Z]);
    black_box_pack_str(&calib_file,"\n");

    black_box_pack_float(&calib_file,calibrate_temp.scale_factor_axis[X],3);
    black_box_pack_str(&calib_file," ");
    black_box_pack_float(&calib_file,calibrate_temp.scale_factor_axis[Y],3);
    black_box_pack_str(&calib_file," ");
	black_box_pack_float(&calib_file,calibrate_temp.scale_factor_axis[Z],3);
    black_box_pack_str(&calib_file,"\n");

    black_box_pack_str(&calib_file,"\n");
  
    black_box_load(&calib_file);
    black_box_close(&calib_file);
	
	while(1){
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      HAL_Delay(1000); // 10 Hz loop
	}
}





