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
#include "pwmwrite.h"
#include "pid.h"
#include "ibus.h"
#include "gps.h"
#include "imu.h"
#include "timer.h"
#include "utils.h"

#include "mpu6050.h"
#include "ms5611.h"
#include "hmc5883.h"
#include "interrupt.h"
#include "sensordetect.h"
#include "blackbox.h"
#include "compass.h"
#include "maths.h"

// Loop init variable
#define LOOP_FEQ 100

enum{
  Feq_100hz = 1,
  Feq_50hz  = 2,
  Feq_25hz  = 4,
  Feq_10hz  = 10,
  Feq_5hz   = 20,
  Feq_1hz   = 100,
}RT_task;

uint32_t max_excution_time_us;
uint32_t num_tasks;
uint16_t loop_us;
black_box_file_t data_test;
// Loop init variable

/*
 * Test function blink led pc13
 */
static void tongepin13()
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
static void tonggle_red_led()
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
}
static void tonggle_green_led()
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

static void task_mavlink(){
   //mavlink_send_control_cmd(AHRS.roll*0.011f,AHRS.pitch*0.011f,1);
   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

static void task_black_box(){  // take about 10 ms
  if (ibusChannelData[8] > 1600){
      // position
      black_box_pack_str(&data_test,"lat ");
      black_box_pack_int(&data_test,_gps.position[0]);
      black_box_pack_str(&data_test,",lon ");
      black_box_pack_int(&data_test,_gps.position[1]);
      black_box_pack_str(&data_test,",alt ");
      
      black_box_pack_int(&data_test,_gps.altitude_msl);
      black_box_pack_str(&data_test,",baro alt ");
      black_box_pack_int(&data_test,0);
      black_box_pack_str(&data_test,",gps_fix ");
      black_box_pack_int(&data_test,_gps.fix);
      
      black_box_pack_str(&data_test,",v1 ");
      black_box_pack_int(&data_test,_gps.velocity[0]);
      black_box_pack_str(&data_test,",v2 ");
      black_box_pack_int(&data_test,_gps.velocity[1]);
      black_box_pack_str(&data_test,",v3 ");
      black_box_pack_int(&data_test,_gps.velocity[2]);

      black_box_pack_str(&data_test,"\n");
      black_box_load(&data_test);
      black_box_sync(&data_test);
    }

}

// task init
task_t task[]={ 
  {imu_update_ahrs,      0,0,0,Feq_100hz}, 
  //{attitude_ctrl,    0,0,0,Feq_50hz},
  {tongepin13,          0,0,0,Feq_1hz},
  {task_black_box,   0,0,0,Feq_10hz},
   //{ms5611_start,         0,0,0,1},
  {ibus_run,         0,0,0,Feq_50hz}   
   // {tongepin13,         0,0,0,100},  
};

void init_scheduler(){
  // init backbox for logging data
  if(!black_box_init()){
       black_box_create_file(&data_test,"fdata.txt");
   }
  timer_start(&htim7); 
  initPWM(&htim3);  
  ibus_init(&huart2,115200);

  //mavlinkInit(22,10,&huart1,115200);
  attitude_ctrl_init();

  mpu6050_init(&hi2c2); 
  i2cDectect(&hi2c2); 
  HAL_Delay(2000);     // wait a second after connected battery 
  imu_calibrate();     // calibrate mpu6050 
 
  //compassInit();
  ms5611_init(&hi2c2); 
  gps_init(&huart1,57600);

  /* scheduler parameters */
	num_tasks = zeroSet();
  max_excution_time_us = zeroSet();
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
  max_excution_time_us = total_factor_excution_us;
  counter ++;
  if(counter ==  LOOP_FEQ)counter = 0;
  while((int)(micros() - timeUs) < loop_us);
  timeUs = micros();
}

