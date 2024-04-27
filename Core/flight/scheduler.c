#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "scheduler.h"
#include "plane.h"
//#include "simulation.h"
#include "redefine.h"
#include "string.h"
#include "faulthandler.h"
#include "log.h"
#include "imu.h"
#include "pid.h"
#include "ibus.h"
#include "pwm.h"
#include "gps.h"
#include "imu.h"
#include "timer.h"
#include "utils.h"
#include "fatfs.h"

#include "mpu6050.h"
#include "ms5611.h"
#include "hmc5883.h"
#include "qmc5883.h"
#include "interrupt.h"
#include "sensordetect.h"
#include "blackbox.h"
#include "compass.h"
#include "maths.h"

// Loop init variable
#define LOOP_FEQ 50

enum{
  Feq_50hz = 1,
  Feq_25hz  = 2,
  Feq_10hz  = 5,
  Feq_1hz  = 50
}RT_task;

uint32_t max_excution_time_us;
uint32_t num_tasks;
uint16_t loop_us;
black_box_file_t data_test;

extern int8_t isSdcard_valid;
extern int8_t isSdcard_write;
uint32_t loop_timeout;
// Loop init variable


/*
 * Test function 
 */
static void task_check_gps_fix()
{
	//if(_gps.fix != 0){
	//if (ibusChannelData[4] > 1600){
	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	//}
}
static void tonggle_red_led()
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
}
static void tonggle_green_led()
{
	if(data_test.file.err != 1 && isSdcard_valid != 1 && ibusChannelData[CH10] > 1600 ){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	}
}

static void task_mavlink(){
   //mavlink_send_control_cmd(AHRS.roll*0.011f,AHRS.pitch*0.011f,1);
   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

extern float roll_cmd;
extern float pitch_cmd;
static void task_black_box(){ 	
  if (ibusChannelData[CH10] > 1600){   
	 static int count = 0;
	  count ++;
	  if(count >= 50){
	      count = 0;
	  }
    /*****roll_cmd roll pitch_cmd pitch lat lon alt baro_alt gps_fix sat v_g v_g_ v_z */  
	if(count %5 == 0){
		black_box_pack_int(&data_test,roll_cmd);
		black_box_pack_str(&data_test," ");

		black_box_pack_int(&data_test,AHRS.roll);
		black_box_pack_str(&data_test," ");
		
		black_box_pack_int(&data_test,pitch_cmd);
		black_box_pack_str(&data_test," ");
		 
		black_box_pack_int(&data_test,AHRS.pitch);
		black_box_pack_str(&data_test," "); 

		black_box_pack_int(&data_test,_gps.position[0]);
		black_box_pack_str(&data_test," ");

		black_box_pack_int(&data_test,_gps.position[1]);
		black_box_pack_str(&data_test," ");
			
		black_box_pack_int(&data_test,_gps.altitude_msl);
		black_box_pack_str(&data_test," ");
		
		black_box_pack_int(&data_test,ms5611_altitude);
		black_box_pack_str(&data_test," ");
		
		black_box_pack_int(&data_test,_gps.fix);
		black_box_pack_str(&data_test," ");
		
		black_box_pack_int(&data_test,_gps.numSat);
		black_box_pack_str(&data_test," ");
		
		black_box_pack_int(&data_test,_gps.velocity[0]);	
		black_box_pack_str(&data_test," ");
		
		black_box_pack_int(&data_test,_gps.velocity[1]);
		black_box_pack_str(&data_test," ");

		black_box_pack_int(&data_test,_gps.velocity[2]);
		black_box_pack_str(&data_test," ");

		black_box_pack_int(&data_test,ms5611_pressure); 
		black_box_pack_str(&data_test," ");
		  
		black_box_pack_int(&data_test,_gps.ground_course); 
		black_box_pack_str(&data_test,"\n");

		black_box_load(&data_test);
    }
	else if(count == 46){
        black_box_sync(&data_test);
	}
  }

}


// task init
task_t task[]={ 
  {imu_update_ahrs,    0,0,0,Feq_50hz}, 
  {attitude_ctrl,    0,0,0,Feq_50hz},
  {ms5611_start, 0,0,0,Feq_50hz},
  {task_black_box,     0,0,0,Feq_50hz},
  {tonggle_green_led,    0,0,0,Feq_25hz},
  {ibus_run,           0,0,0,Feq_50hz},
  {task_check_gps_fix,    0,0,0,Feq_10hz}
};

void init_scheduler(){
  // init backbox for logging data
  if(!black_box_init()){	  
	  black_box_open_file(&data_test,"flightdata.txt",FA_OPEN_ALWAYS | FA_WRITE);
	  black_box_pack_str(&data_test,"-------------------new section-------------------\n");
	  black_box_load(&data_test);
	  black_box_sync(&data_test);
   }
  else{
     while(1){
	    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
        HAL_Delay(50); // 10 Hz loop
   }
  }
  timer_start(&htim7); 
  initPWM(&htim3); 
  ibus_init(&huart1,115200);

  //mavlinkInit(22,10,&huart1,115200);
  attitude_ctrl_init();
  mpu6050_init(&hi2c2); 
  i2cDectect(&hi2c2); 
  ms5611_init(&hi2c2); 
  gps_init(&huart3,57600);
  compassInit();
  
  //
  HAL_Delay(2000);     // wait a second after connected battery 
  imu_calibrate();     // calibrate mpu6050 
 
  
  
  /* scheduler parameters */
  loop_timeout = 0;
  max_excution_time_us = 0;
	num_tasks  = sizeof(task)/sizeof(task_t);
	loop_us = (1.0f/ LOOP_FEQ)*1e+6 ;
}

void start_scheduler() {
	
  static int counter = 0;
  static uint32_t timeUs;
  uint32_t time_1;
  uint32_t total_factor_excution_us = 0;
  for (int i = 0; i < num_tasks; i++){
      if((task[i].exec != NULL) && (counter % task[i].period == 0)){
        time_1 = micros();
        task[i].execution_cycle_us = micros() - task[i].last_exec_time_us;
        task[i].last_exec_time_us = time_1;
        task[i].exec();
        task[i].execution_time_us = micros() - time_1;
        total_factor_excution_us += task[i].execution_time_us;
		  
      }
  }
  //if(max_excution_time_us < total_factor_excution_us){
       max_excution_time_us = total_factor_excution_us;
 // }
  counter ++;
  if(counter ==  LOOP_FEQ)counter = 0;
  while((int)(micros() - timeUs) < loop_us);
  timeUs = micros();

}

