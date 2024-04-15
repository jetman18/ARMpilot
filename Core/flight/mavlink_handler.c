#include "plane.h"
#include "timer.h"
#include "imu.h"
#include "mavlink.h"
#include "usart.h"
#define MAX_LENGHT 100

static mavlink_named_value_int_t  val_int;
static mavlink_named_value_float_t  val_float;
static mavlink_attitude_t attitude;
static mavlink_message_t msg;
static mavlink_status_t msg_status;
static uint8_t data;
static uint8_t index_;
uint8_t sys_id,com_id;
static UART_HandleTypeDef *uart;
uint8_t buffer__[MAX_LENGHT];
static int isTxcpl;

void mavlinkInit(uint8_t syss_id, uint8_t comm_id,UART_HandleTypeDef *uartt,uint32_t baudrate){
    isTxcpl = 1;
    index_ =0;
	sys_id  = syss_id;
    com_id  = comm_id;
	uart = uartt;
    uartt->Init.BaudRate = baudrate;
	HAL_UART_Init(uartt);
	HAL_UART_Receive_IT(uart, &data,1);
}


UART_HandleTypeDef *mavlink_uart_port(){
    return uart;
}


void mavlinkCallback(){
    if (mavlink_parse_char(MAVLINK_COMM_0,data,&msg, &msg_status))
    {
        switch (msg.msgid)
        {
        case MAVLINK_MSG_ID_ATTITUDE:
            mavlink_msg_attitude_decode((const mavlink_message_t*)&msg,&attitude);
            break;
        case MAVLINK_MSG_ID_NAMED_VALUE_INT:
        	mavlink_msg_named_value_int_decode((const mavlink_message_t*)&msg,&val_int);
            break;
        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
        	mavlink_msg_named_value_float_decode((const mavlink_message_t*)&msg,&val_float);
            break;
        }
    }
    HAL_UART_Receive_IT(uart, &data,1);
}


void mavlink_send_attitude(float roll,float pitch, float yaw, float lat,float lon, float alt){
	//if(isTxcpl){
		uint32_t boot_time = millis();
		mavlink_message_t msg_send;
        mavlink_msg_command_long_pack(sys_id,com_id,&msg_send,1,1,1,0,roll,pitch,yaw,lat,lon,alt,0);
		uint16_t len = mavlink_msg_to_send_buffer(buffer__,&msg_send);
		HAL_UART_Transmit(uart,buffer__,len,10);

	//	isTxcpl = 0;
	//}
}

void mavlink_tx_cpl_callback()
{
	isTxcpl = 1;
}

