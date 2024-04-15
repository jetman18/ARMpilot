#ifndef _GPS_H_
#define _GPS_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"


typedef struct{
    int32_t   position[2];
    int32_t   velocity[3];
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

void gpsInit(UART_HandleTypeDef *uartt,uint32_t baudrate);
void gpsCallback(void);
uint8_t getSat();
uint8_t  getFix();
uint32_t getUpdateTime();
#ifdef __cplusplus
}
#endif
#endif
