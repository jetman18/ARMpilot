#ifndef _COMPASS_H_
#define _COMPASS_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "axis.h"

void compassInit();
void compass_get(axis3_t *out);
void test_compass();

#ifdef __cplusplus
}
#endif

#endif