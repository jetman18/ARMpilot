#ifndef _GPS_H_
#define _GPS_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"

typedef struct{
    int32_t   position[2]; // *10^7
    int32_t   velocity[3]; //  cm
    int32_t   Gspeed;

    int32_t   speedAccuracy;
    int32_t   headingAccuracy;
    uint32_t  horizontalAccuracy;
    uint32_t  VerticalAccuracy;
    
    uint32_t  posUpdateTime;
    uint32_t  timer_;
    uint16_t  altitude_msl;
    uint16_t  ground_course;
    uint8_t   numSat;
    uint8_t   fix;
}gpsData_t;
extern gpsData_t _gps;
UART_HandleTypeDef *gps_uart_port();
void gps_init(UART_HandleTypeDef *uartt,uint32_t baudrate);
void gps_callback(void);
#ifdef __cplusplus
}
#endif
#endif
