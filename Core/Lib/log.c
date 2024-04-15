#include "log.h"
#include "math.h"
#include "maths.h"
#include "stm32f1xx_hal.h"
#include "string.h"
static int sig;
static int indexx;

static void reverse( uint8_t *str, int len)
{
    int i = sig, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}
static int intToStr(int x,  uint8_t *str, int d)
{
    while (x) {
        str[indexx++] = (x % 10) + '0';
        x = x / 10;
    }

    while (indexx < d)
        str[indexx++] = '0';
    reverse(str,indexx);
    return indexx;
}

#define afftezero 3  //float 

void print_float(float n)
{  

}

int write_char(UART_HandleTypeDef *huart,uint8_t *str)
{
    uint16_t len=0;
    while(str[len++]);
    HAL_UART_Transmit(huart,str,len-1,100);
    return len;
}
int writeln_int(UART_HandleTypeDef *huart,int x)
{
    indexx=0;
    sig = 0;
    uint8_t str_[11];
    memset(str_,0,11);
    if(x==0){
    	 str_[0]= 48;
    	 HAL_UART_Transmit(huart,str_,1,100);
    	 return 1;
    }
    if(x<0){
        x*=-1;
        str_[0] = '-';
        indexx++;
        sig = 1;
    }
   int len = intToStr(x,str_,0);
   str_[len] = '\n';
   HAL_UART_Transmit(huart,str_,len+1,2);
   return len;
}
