#ifndef MAINLOOP_H
#define MAINLOOP_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"
//#define SIMULATION
// attitude controll


// navigation
//extern uint16_t autopilot_stick;
//extern uint16_t circle_stick;
//extern uint16_t rtHome_stick;
//void Autopilot();

// mavlink handler
void mavlinkInit(uint8_t syss_id, uint8_t comm_id,UART_HandleTypeDef *uartt,uint32_t baudrate);
UART_HandleTypeDef *mavlink_uart_port();
void mavlinkCallback();
void mavlink_tx_cpl_callback();
void mavlink_send_attitude(float roll,float pitch, float yaw, float lat,float lon, float alt);

// attitude contrller
void attitude_ctrl();
void attitude_ctrl_init();
// mainloop
void main_loop();

//estimate 
void estimates_start();

#ifdef __cplusplus
}
#endif

#endif

