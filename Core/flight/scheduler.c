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
#include "interrupthandler.h"
#include "sensordetect.h"
#include "blackbox.h"
#include "compass.h"
#include "maths.h"

// Loop init variable
#define LOOP_FEQ 200

enum{
  HARD_REAL_TIME = 0,
  SOFT_REAL_TIME,
}RT_task;

uint16_t slot_take_time_us[LOOP_FEQ];
uint32_t max_excution_time_us;

uint32_t num_tasks;
uint16_t loop_us;

int cout;
uint32_t timeUs;
// Loop init variable

/*
 * Test function blink led pc13
 */
static void tongepin13()
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
static void tongepin4()
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
}
static void tongepin5()
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

static void mavlink(){
   mavlink_send_control_cmd(AHRS.roll*0.011f,AHRS.pitch*0.011f,1);
   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

task_t task[]={ 
   {ahrs_update,      0,0,0,1}, 
	
  //{pidUpdate,       0,0,0,1},

   //{hmc_get_raw,         0,0,0,3},
  //{ms5611_start,         0,0,0,1},
  //{hmc_get_raw,         0,0,0,10},   
  // {tongepin13,         0,0,0,100},
   {tongepin5,          0,0,0,20},
   {tongepin4,          0,0,0,10} 
  //{HITL_thread,         0,0,0,25} 
};

void init_scheduler(){
  // init backbox for logging data
  if(black_box_init()){
      // init error
      //HAL_GPIO_WritePin(GPIOX,GPIO_PIN_X,SET);
   }
 

   timer_start(&htim7); // hardwave init
   //initPWM(&htim3);    // for pwm
    //HITL_init(&huart2);
    mavlinkInit(22,10,&huart1,19200);

  /*------sensor init----------*/ 
  mpu6050_init(&hi2c2); 
  imuCalibrate();     // calibrate mpu6050 
  //i2cDectect(&hi2c2); // mpu configure i2c bus pass through in oder to connect with hmc5883 
  //delay_ms(2000);     // wait a second after connected battery 
 
  //fault_pc13_blink();
  //compassInit();
  //ms5611_init(&hi2c2); // inti ms5611
  //gpsInit(&huart1,57600);


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
  uint32_t total_ex_run = 0;
  memset(slot_take_time_us,0,200);

  uint32_t l = micros();
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
  slot_take_time_us[counter] =  micros()- l;
  counter ++;
  if(counter ==  LOOP_FEQ)counter = 0;
  while((int)(micros() - timeUs) < loop_us);
  timeUs = micros();
}

