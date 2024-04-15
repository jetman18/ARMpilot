#include "faulthandler.h"
#include "gpio.h"
#include "timer.h"
#include "maths.h"
#include "tim.h"


void fault_pc13_blink(int delay){
	
    while(1){
		HAL_Delay(delay);
	    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);		
   }
}

void fault_buzz(){
    while(1){

    }

}