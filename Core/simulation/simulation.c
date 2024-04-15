#include "simulation.h"
#include "stm32f1xx_hal.h"
#include "timer.h"
#include "usart.h"
#include "blackbox.h"

#define DATA_LENGTH 52
#define HITL_TIMEOUT 500 // us
const int magic_number = 305419896;

int k = 1e+6;
typedef struct{
   float roll_rate;
   float pitch_rate;
   float yaw_rate;
   float roll;
   float pitch;
   float yaw;
   
   // gps data
   float latitude;
   float longitude;
   float altitude;
   //float v_north;
   //float v_east;
   //float v_down;
}data_t;

typedef union {
  data_t get;
  char buf[DATA_LENGTH];
}pack;
static int parseMSG(const uint8_t c);

static uint8_t c;
pack msg;

static UART_HandleTypeDef *uart;

UART_HandleTypeDef* HITL_uart_port()
{
   return uart;
} 

void HITL_init(UART_HandleTypeDef *huart){
   uart = huart;
   HAL_UART_Receive_IT(uart,&c,1);
}


void HITL_callback(){
   parseMSG(c);
   HAL_UART_Receive_IT(&huart2,&c,1);
}


uint32_t dt_time_hitl;
static uint32_t time_ms;
static int parseMSG(const uint8_t c){
    static int step = 0;
    static int frame_index = 3;
    static int frame_start = 0;
    static char buff[4];
    static int count = 3;
    if(frame_start == 0){
        buff[count] = c;
        if(count > 0){
            count --;
        }
        else{
            int magic = *(int*)buff;
            // shift bytes
            for(int i = 3; i > 0; i--){
                buff[i] = buff[i - 1];
               }
            if(magic == magic_number){
                frame_start = 1;
              }      
            }
    }else{
		msg.buf[frame_index] = c;
		if(frame_index == step){
			step += 4;
			frame_index = 4;
		    frame_index += step;
			if(step == 36){
			    step = 0;
				frame_index = 3;
				frame_start = 0;
                count = 3;
				dt_time_hitl = millis() - time_ms; 
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
				time_ms = millis();
                return 0;
			}			
		}
		frame_index --;
    }
    return -1;
}