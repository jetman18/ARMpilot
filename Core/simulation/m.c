#include"stdio.h"
#include "string.h"
// 14 float


char data[] = {0x12,0x34,0x56,0x78,0x01,0x02,0x03,0x04,0x01,0x02,0x03,0x04,0x01,0x02,0x03,0x04,0x01,0x02,0x03,0x04,0x01,0x02,0x03,0x04,0x01,0x02,0x03,0x04,0x01,0x02,0x03,0x04,0x01,0x02,0x03,0x04,0x01,0x02,0x03,0x04,0x01,0x02,0x03,0x04,0x01,0x02,0x03,0x04,0x01,0x02,0x03,0x04,0x01,0x02,0x03,0x04};
static void cvt_to_big_endian(char *d);
int parseMSG(char c);
// xml set magic number = 0x12345678 dec = 305419896
const int magic_number = 305419896;
const char header[] = {0x12,0x34,0x56,0x78};
const int data_length = 52; // 52 byte

char buffer[52];


char l[] = {0x44,0xf6,0xfd,0xde};
//char l[] = {0xde,0xfd,0xf6,0x44};
int main(){
    //float helo = *(float*)l;
    //printf("%f",helo);
    int k = 1e+7;
    printf("%d",(int)(k*102.1234567f));
/*
    int o = 0;

    while (data[o])
    {
       parseMSG(data[o++]);
    }
    for(int i= 0; i < 52 ; i ++){
        printf(" %x",buffer[i]);
   }    
  */ 
    return 0;
}

//char data[] = {0x01,0x12,0x34,0x56,0x78,0x00,0x00,0x00,0x01};/
// xml set magic number = 0x12345678 dec = 305419896

int parseMSG(char c){

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
                printf("header ok\n");
              }      
            }
    }else{
		
		// frame_index = 3 
		static int step = 0;
		buffer[frame_index] = c;
		if(frame_index == step){
			step += 4;
			frame_index = 4;
		    frame_index += step;
			if(step == 52){
			    step = 0;
				frame_index = 3;
				frame_start = 0;
                count = 3;
                return 0;
			}			
		}
		frame_index --;
    }
    return 1;
}

