
#ifndef REDEFINE_H_
#define REDEFINE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "timer.h"
extern SD_HandleTypeDef hsd;
#include "bsp_driver_sd.h"

//int ii=0;
//uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
//{
//for (ii = 1; ii <= 5; ii ++){
//   if (HAL_SD_WriteBlocks(&hsd, (uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout) == HAL_OK){
//       return (MSD_OK); // Succeeded
//    }else{
//		HAL_SD_Abort(&hsd); //clear error flag
//   }
//}

//return (MSD_ERROR);

//}


#ifdef __cplusplus
}
#endif
#endif
