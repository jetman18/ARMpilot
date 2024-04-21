#ifndef BLACKBOX_H
#define BLACKBOX_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"
#include "fatfs.h"
#define BUFFER_SIZE 400
// black box
typedef struct{
   int indexx;
   FIL file;
   uint16_t buffer_index;
   char buffer[BUFFER_SIZE];
}black_box_file_t;

int black_box_init();
int black_box_create_file(black_box_file_t *fs,char *file_name);
uint32_t black_box_get_total_space();
uint32_t black_box_get_free_space();
void black_box_pack_int(black_box_file_t *fs,int val);
void black_box_pack_float(black_box_file_t *fs,float val,uint8_t digit_after_point );
void black_box_pack_str(black_box_file_t *fs,const char *c);
void black_box_load(black_box_file_t *fs);
void black_box_sync(black_box_file_t *fs);
void black_box_close(black_box_file_t *fs);
int black_box_read(black_box_file_t *fs, char *file_name, char *bufferr,uint8_t len);
uint16_t black_box_get_buffer_lenght(const black_box_file_t *fs);

#ifdef __cplusplus
}
#endif

#endif

