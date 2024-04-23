#include "pwm.h"
#include "tim.h"
#include "maths.h"
#include "timer.h"


#define PWM_RELOAD_AFFTER 10000     // 100 hz
#define SAFE_TIMEOUT 1000

TIM_HandleTypeDef *htimm;
static uint32_t pwm_last_update_ms;
static uint8_t pr_status;
uint8_t pwm_lock_status;

/*
 * init pwm 
 */
void initPWM(TIM_HandleTypeDef *htim){
	htimm = htim;
	pr_status = 1;
	pwm_lock_status = 0;
	pwm_last_update_ms= 0;
	HAL_TIM_PWM_Start(htim,ch1);
	HAL_TIM_PWM_Start(htim,ch2);
	HAL_TIM_PWM_Start(htim,ch3);
	//HAL_TIM_PWM_Start(htim,ch4);
	__HAL_TIM_SetAutoreload(htimm,PWM_RELOAD_AFFTER);
    __HAL_TIM_SetCompare (htimm,ch1,1000);
	__HAL_TIM_SetCompare (htimm,ch2,1500);
	__HAL_TIM_SetCompare (htimm,ch3,1500);
}


void writePwm(uint32_t Channel,int16_t dulty,int8_t invert)
{     if( invert){
	    dulty -= 1000;
	    dulty  = 2000 - dulty; 
      }
	  dulty = constrain(dulty,1000,2000);
	__HAL_TIM_SetCompare (htimm,Channel,dulty);
}

void write_pwm_ctrl(uint16_t throtlle,uint16_t servoL,uint16_t servoR){
      if(pr_status || pwm_lock_status){
           pwm_last_update_ms = HAL_GetTick();
		   pr_status = 0;
		   return;
	  }
	 writePwm(TIM_CHANNEL_1,throtlle,INVERT);
     writePwm(TIM_CHANNEL_2,servoL,INVERT);
     writePwm(TIM_CHANNEL_3,servoR,NOT_INVERT);
	pwm_last_update_ms = HAL_GetTick();
}


void pwm_systick_callback(){
	if(pr_status){
		return;
	}
    if((HAL_GetTick() - pwm_last_update_ms) > SAFE_TIMEOUT){
		pwm_lock_status = 1; // lock
		// reset all chanel
		writePwm(TIM_CHANNEL_1,1000,NOT_INVERT);
		writePwm(TIM_CHANNEL_2,1500,NOT_INVERT);
		writePwm(TIM_CHANNEL_3,1500,INVERT);
	}
}



