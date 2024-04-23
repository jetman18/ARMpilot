#include "maths.h"
#include "ibus.h"
#include "timer.h"

#define IBUS_BUFFSIZE 32
#define IBUS_SYNCBYTE 0x20
#define FALSE 0
#define TRUE 1

//#define DMA_MODE

static int ibusFrameDone = FALSE;
uint32_t ibusChannelData[IBUS_MAX_CHANNEL];
static uint8_t ibus[IBUS_BUFFSIZE] = {0, };
static uint8_t rx_buff;
static UART_HandleTypeDef *uart;

#ifdef DMA_MODE
static uint8_t buffer_dma[2*IBUS_BUFFSIZE];
static uint8_t is_receive_cpl;
#endif


static void ibusDataReceive(uint8_t c);
static int ibusFrameComplete(void);


void ibus_init(UART_HandleTypeDef *uartt,uint32_t baudrate)
{
    for(int i = 0;i < IBUS_MAX_CHANNEL ; i++){
      ibusChannelData[i] = 1000;
    }
	uart = uartt;
    uartt->Init.BaudRate = baudrate;
	HAL_UART_Init(uartt);
#ifdef DMA_MODE
    is_receive_cpl = 0;
	HAL_UART_Receive_DMA(uart,buffer_dma,2*IBUS_BUFFSIZE);
#else 
	HAL_UART_Receive_IT(uart, &rx_buff,1);
#endif
}

UART_HandleTypeDef *ibus_uart_port(){
   return uart;
}

void ibus_run(){
#ifdef DMA_MODE
   if(is_receive_cpl){
	 
	int buffer_index = 0; 
	while(buffer_index < 2*IBUS_BUFFSIZE){
		ibusDataReceive(buffer_dma[buffer_index]);
		buffer_index ++;
	}  
	ibusFrameComplete();
	is_receive_cpl = 0;
   }
#else 
   ibusFrameComplete();
#endif
}

void ibus_calback(){
#ifdef DMA_MODE
	is_receive_cpl = 1;
#else 
	ibusDataReceive(rx_buff);
    HAL_UART_Receive_IT(uart, &rx_buff,1);
#endif

}

static void ibusDataReceive(uint8_t c)
{
    uint32_t ibusTime;
    static uint32_t ibusTimeLast;
    static uint8_t ibusFramePosition;

    ibusTime = micros();

    if ((ibusTime - ibusTimeLast) > 3000)
        ibusFramePosition = 0;

    ibusTimeLast = ibusTime;

    if (ibusFramePosition == 0 && c != IBUS_SYNCBYTE)
        return;

    ibus[ibusFramePosition] = (uint8_t)c;

    if (ibusFramePosition == IBUS_BUFFSIZE - 1) {
        ibusFrameDone = TRUE;
    } else {
        ibusFramePosition++;
    }
}

static int ibusFrameComplete(void)
{
	
    uint8_t i;
    uint16_t chksum, rxsum;

    if (ibusFrameDone) {
        ibusFrameDone = FALSE;

        chksum = 0xFFFF;

        for (i = 0; i < 30; i++)
            chksum -= ibus[i];

        rxsum = ibus[30] + (ibus[31] << 8);

        if (chksum == rxsum) {
            ibusChannelData[0] = (ibus[ 3] << 8) + ibus[ 2];
            ibusChannelData[1] = (ibus[ 5] << 8) + ibus[ 4];
            ibusChannelData[2] = (ibus[ 7] << 8) + ibus[ 6];
            ibusChannelData[3] = (ibus[ 9] << 8) + ibus[ 8];
            ibusChannelData[4] = (ibus[11] << 8) + ibus[10];
            ibusChannelData[5] = (ibus[13] << 8) + ibus[12];
            ibusChannelData[6] = (ibus[15] << 8) + ibus[14];
            ibusChannelData[7] = (ibus[17] << 8) + ibus[16];
			ibusChannelData[8] = (ibus[19] << 8) + ibus[18];
            ibusChannelData[9] = (ibus[21] << 8) + ibus[20];
			
			for(int i =0; i< IBUS_MAX_CHANNEL ;i++){
			    if(ibusChannelData[i] > 2100){
				    ibusChannelData[i] = 1000;
				}
			}
            return TRUE;
        }
    }
    return FALSE;
}


