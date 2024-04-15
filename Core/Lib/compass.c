#include "stm32f1xx.h"
#include "compass.h"
#include "hmc5883.h"
//#include "blackbox.h"
#include "maths.h"
#include "i2c.h"
#include "timer.h"
#include "gpio.h"
#include "faulthandler.h"
#include "string.h"

// blackbox store calirbate value
#define BUFFER_LENGTH 20
//black_box_file_t calib_file;
//char buffer_[BUFFER_LENGTH];

static char file_name[] = "compass_calibarte_data.txt";
// calibrate
static uint8_t is_calibrated = FALSE;
uint8_t calibrated_file_vaild = FALSE;
int hard_iron_calibrate_value[3];
float scale_factor_axis[3];

// Set time for calibrating (in minutes)
const uint8_t cali_tim_minutes = 3;
const int16_t max_change = 300;
uint16_t ignore_data;

static void string2integer(char* str, int *in);
static int stringToInteger(char* str);
static void read_calibrate_file();
static void compass_calibrate();


/*  Init compass
 */
void compassInit(){

  scale_factor_axis[X] = 1.0f;
  scale_factor_axis[Y] = 1.0f;
  scale_factor_axis[Z] = 1.0f;

  hard_iron_calibrate_value[X] = 0;
  hard_iron_calibrate_value[Y] = 0;
  hard_iron_calibrate_value[Z] = 0;
  // init sensor
  hmc5883_init(&hi2c2);
  // read calibrate value

  //read_calibrate_file();
  compass_calibrate();
}
/*
void read_calibrate_file(){
   int status = black_box_read(&calib_file,file_name,buffer_,BUFFER_LENGTH);
   if(!status){
       calibrated_file_vaild = TRUE;
	   string2integer(buffer_,(int*)hard_iron_calibrate_value);
   }
   // calibrate data not valid
   else{
       calibrated_file_vaild = FALSE;
       fault_pc13_blink(200);
   }
}
*/

/* Calibrate function
 * write calibrate value to sd card
 */
static void compass_calibrate(){
    int16_t max_val[] = {-32767,-32767,-32767};
    int16_t min_val[] = {32767, 32767, 32767};
    uint8_t fist_data = TRUE;
    int calibrate_value[3];
    ignore_data = 0;
    int16_t last_axis[3];
    uint8_t start = TRUE ;
    axis3_t as;
    int step = 0;

    // create file
	/*
    if(start){
        black_box_create_file(&calib_file,file_name);
        delay_ms(200);
        start = FALSE;
    }
	*/
	  while(TRUE){
        // read data from sensor
        hmc_get_raw(&as);	
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
        /*
        black_box_pack_int(&calib_file,(int)as.x);
        black_box_pack_str(&calib_file," ");
        black_box_pack_int(&calib_file,(int)as.y);
        black_box_pack_str(&calib_file," ");
        black_box_pack_int(&calib_file,(int)as.z);
        black_box_pack_str(&calib_file,"\n");
		
		black_box_load(&calib_file);
        black_box_sync(&calib_file);
        */
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100); // 10 Hz loop
        // wait to 
        step ++;
        if( step/600 >= cali_tim_minutes ){
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
    calibrate_data[X] = (int)(max_val[X] + min_val[X])/2;
    calibrate_data[Y] = (int)(max_val[Y] + min_val[Y])/2;
    calibrate_data[Z] = (int)(max_val[Z] + min_val[Z])/2;

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
    scale_factor_axis[X] = (float)x_/max_value;
    scale_factor_axis[Y] = (float)y_/max_value;
    scale_factor_axis[Z] = (float)z_/max_value;

    // write data to sc card
	/*
	black_box_pack_str(&calib_file,"calibrate \n");
    black_box_pack_int(&calib_file,calibrate_data[X]);
    black_box_pack_str(&calib_file," ");
    black_box_pack_int(&calib_file,calibrate_data[Y]);
    black_box_pack_str(&calib_file," ");
    black_box_pack_int(&calib_file,calibrate_data[Z]);
    black_box_pack_str(&calib_file,"\n");

    black_box_pack_float(&calib_file,scale_factor_axis[X],3);
    black_box_pack_str(&calib_file," ");
    black_box_pack_float(&calib_file,scale_factor_axis[Y],3);
    black_box_pack_str(&calib_file," ");
	black_box_pack_float(&calib_file,scale_factor_axis[Z],3);
    black_box_pack_str(&calib_file,"\n");

    black_box_pack_str(&calib_file,"wrong data: ");
    black_box_pack_int(&calib_file,(int)ignore_data);
    black_box_pack_str(&calib_file,"\n");
  
    black_box_load(&calib_file);
    black_box_close(&calib_file);
	 */
	fault_pc13_blink(1000);
}



void string2integer(char* str, int *in){
  int step = 0;
  int count= 0;
  int shift_space = 0;
  char index[3];
  memset(index,0,3);
  while (str[shift_space++] == ' ')
  {
    step ++;
  }
  index[count++] = 0;
  
  while(str[step]){
    if( str[step] == ' ' && str[step+1] != ' '){
        index[count++] = step;
    }
    step++;
  }
  for(int i=0;i<3;i++){
     in[i] = stringToInteger(&str[index[i]]);
  }
}


int stringToInteger(char* str) {
    int result = 0; 
    int sign = 1; 

    while (*str == ' ') {
        str++;
    }

    if (*str == '-') {
        sign = -1;
        str++;
    } else if (*str == '+') {
        str++;
    }

    while (*str >= '0' && *str <= '9') {
        result = result * 10 + (*str - '0');
        str++;
    }

    result *= sign;

    return result;
}
