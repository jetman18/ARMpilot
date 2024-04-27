#include "fatfs.h"
#include "string.h"
#include "timer.h"
#include "utils.h"
#include "blackbox.h"
#include "math.h"
#include <stdlib.h>

#define MAX_BUFFER_SIZE 400

static FATFS *pfs;
static DWORD fre_clust;
static FATFS _fs;
int8_t isSdcard_valid;
int8_t isSdcard_write;
static void reverse( char *str, int len);
static int intToStr(black_box_file_t *fs,int x,  char *str, int d);
static int n_tu(int number, int count);
static int Float_to_string(float f, uint8_t places, char strOut[]);

/*
 * init black box
 */
int black_box_init(){
	isSdcard_write = 0;
    isSdcard_valid = 0;
    uint8_t mount = f_mount(&_fs,"", 1);
	if( mount != 0 ){
        isSdcard_valid = 1;
        return -1;
    }

    return 0;
}

int black_box_create_file(black_box_file_t *fs,char *file_name){
   if(isSdcard_valid){
       return 0;
   } 
    fs->buffer_index = 0;
    uint8_t open = f_open(&fs->file,file_name, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
    f_lseek(&fs->file,fs->file.fsize);
    if(open == FR_OK){
        return 0;
    }
   
   return -1;
}

int black_box_open_file(black_box_file_t *fs,char *file_name,uint8_t mode){
   if(isSdcard_valid){
       return 0;
   } 
    fs->buffer_index = 0;
    uint8_t open = f_open(&fs->file,file_name,mode);
    f_lseek(&fs->file,fs->file.fsize);
    if(open == FR_OK){
        return 0;
    }
   
   return -1;
}


/*  read data
 *  
 */

int black_box_read(black_box_file_t *fs, char *file_name, char *bufferr,uint8_t len)
{
    if(isSdcard_valid){
        return 0;
    }
	memset(bufferr,0,len);
    f_gets(bufferr,len,&fs->file);
	//   return -1;
	//
	return 0;
}

/*
 *  Description: Convert a float number to string and write to buffer
 */
void black_box_pack_float(black_box_file_t *fs,float val,uint8_t digit_after_point ){
    // do not write anything if sd card not valid
    if(isSdcard_valid)
        return;
    char str_[20];
    memset(str_,0,20);
    uint8_t len = Float_to_string(val,digit_after_point,str_);
    uint8_t index = 0;
    while(str_[index]){
        if((fs->buffer_index + index) > MAX_BUFFER_SIZE){
                fs->buffer_index = MAX_BUFFER_SIZE;
                return;
        }
        fs->buffer[fs->buffer_index + index] = str_[index];
        index ++;
    }
    fs->buffer_index += index;
}


/*
 *  Description: Convert a integer number to string and write to buffer
 *  Input (int type [-2147483647 2147483647] )
 */

void black_box_pack_int(black_box_file_t *fs,int val){
    if(isSdcard_valid){
        return;
	}
	fs->indexx=0;
	int len_str;
	int val_ = val;
	char str_[11];
	memset(str_,0,11);
	if(val != 0){
		val = abs(val);
		len_str = intToStr(fs,val,str_,0);
		if(val_ < 0){
			for(int i = len_str; i > 0; i--){
				str_[i] = str_[i - 1];
			}
			len_str ++;
			str_[0] = '-';
		}
	}
	else{
	   fs->buffer[fs->buffer_index] ='0';
	   len_str = 1;
	   fs->buffer_index ++;
		return;
	}
	// copy str to buffer
	int str_idx = 0;
	int index_flag;
	int max_index = fs->buffer_index + len_str;
	if(max_index <=  MAX_BUFFER_SIZE){
		max_index = fs->buffer_index + len_str;
		index_flag = 1;
	}
	else{
		max_index = MAX_BUFFER_SIZE;
		index_flag = 0;
	}
	for(int j = fs->buffer_index ; j < max_index; j++ ){
			fs->buffer[j] = str_[str_idx ++];
	}
	
	if(index_flag){
		fs->buffer_index += len_str;
	}
	else{
		fs->buffer_index += MAX_BUFFER_SIZE;
	}
}

/*
 * Description: Write str to buffer
 */
void black_box_pack_str(black_box_file_t *fs,const char *c){
    if(isSdcard_valid){
        return;
    }
    int i = 0;
    while (c[i]){
        if((fs->buffer_index + i) > MAX_BUFFER_SIZE){
                fs->buffer_index = MAX_BUFFER_SIZE;
                return;
        }
        fs->buffer[fs->buffer_index + i] = c[i];
        i ++;
    }
    fs->buffer_index += i;
}

/*
 * Description: Write buffer to sd card
 */
void black_box_load(black_box_file_t *fs)
 {
      if(isSdcard_valid)
		  return; 
	// __disable_irq();	  
      isSdcard_write = f_puts(fs->buffer,&fs->file);
	//__enable_irq();

      memset(fs->buffer,0,sizeof(fs->buffer));
      fs->buffer_index = 0;
 }


/*
 * Description: sync file
 */
void black_box_sync(black_box_file_t *fs)
 { 
	 if(isSdcard_valid)
	    return;
	 f_sync(&fs->file);
 }

/*
 * Description: close file
 */
void black_box_close(black_box_file_t *fs)
 {    
	 if(isSdcard_valid)
	    return;
      f_close(&fs->file);
 }

/*
 * Description: get buffer length
 */
 uint16_t black_box_get_buffer_lenght(const black_box_file_t *fs)
 {
     return fs->buffer_index;
 }
 
/*
 * get total space
 */
uint32_t black_box_get_total_space()
 {
     f_getfree("", &fre_clust, &pfs);
     return (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
 }
 
 /*
  * get free space
  */
uint32_t black_box_get_free_space()
 {
     f_getfree("", &fre_clust, &pfs);
     return (uint32_t)(fre_clust * pfs->csize * 0.5);
 }


static void reverse( char *str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

 static int intToStr(black_box_file_t *fs,int x,  char *str, int d)
{
    while (x) {
        str[fs->indexx++] = (x % 10) + '0';
        x = x / 10;
    }

    while (fs->indexx < d)
        str[fs->indexx++] = '0';
    reverse(str,fs->indexx);
    return fs->indexx;
}



/** Number on countu **/
static int n_tu(int number, int count)
{
    int result = 1;
    while(count-- > 0)
        result *= number;

    return result;
}


/*
 * Description: Convert float to string
 * Input: The float number to convert and how decimal places
 * Output: The char array to save the string to
 * Return: length of string
 */
static int Float_to_string(float f, uint8_t places, char strOut[])
{
    long long int length, length2, i, number, position, sign;
    float number2;

    sign = -1;   // -1 == positive number
    if (f < 0)
    {
        sign = '-';
        f *= -1;
    }

    number2 = f;
    number = f;
    length = 0;  // Size of decimal part
    length2 = 0; // Size of tenth

    /* Calculate length2 tenth part */
    while( (number2 - (float)number) != 0.0 && !((number2 - (float)number) < 0.0)  && (length2 < places))
    {
         number2 = f * (n_tu(10.0, length2 + 1));
         number = number2;

         length2++;
    }

    /* Calculate length decimal part */
    for (length = (f > 1) ? 0 : 1; f > 1; length++)
        f /= 10;

    position = length;
    length = length + 1 + length2;
    number = number2;
    if (sign == '-')
    {
        length++;
        position++;
    }

    for (i = length; i >= 0 ; i--)
    {
        if (i == (length))
            strOut[i] = '\0';
        else if(i == (position))
            strOut[i] = '.';
        else if(sign == '-' && i == 0)
            strOut[i] = '-';
        else
        {
            strOut[i] = (number % 10) + '0';
            number /=10;
        }
    }
    if( f - (int)f == 0.0){
        strOut[length++] = '0';
    }
    return length;
}

