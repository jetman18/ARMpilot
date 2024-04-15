#ifndef LPF_H
#define LPF_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "scheduler.h"
#include"maths.h"
#include"math.h"

// 1oder
static inline float pt1FilterGain(float f_cut, float dT){
    float RC = 1 / (2 * M_PIf * f_cut);
    return dT*(1e-06) / (RC + dT*(1e-06));
}

static inline void pt1FilterApply(float *output,float input,float f_cut,uint32_t dT)
{
    float gain_k = pt1FilterGain(f_cut,dT);
    *output  = *output  + gain_k*(input - *output );
}

// 2oder
typedef struct pt2filter{
   float pt1;
   float pt2;
}pt2filter;

static inline float pt2FilterGain(float f_cut, float dT)
{
    const float order = 2.0f;
    const float orderCutoffCorrection = 1 / sqrtf(powf(2, 1.0f / order) - 1);
    float RC = 1 / (2 * orderCutoffCorrection * M_PIf * f_cut);
    // float RC = 1 / (2 * 1.553773974f * M_PIf * f_cut);
    // where 1.553773974 = 1 / sqrt( (2^(1 / order) - 1) ) and order is 2
    return dT*(1e-06) / (RC + dT*(1e-06));
}

static inline void pt2FilterApply(pt2filter *ft,float input,float f_cut,float dT)
{
    float k_gain = pt2FilterGain(f_cut,dT);
    ft->pt1 = ft->pt1 + k_gain * (input - ft->pt1);
    ft->pt2 = ft->pt2 + k_gain * (ft->pt1 - ft->pt2);
}


// slew militer
typedef struct slew_lm{
     float value;
     float last_value;
}slew_limiter_t;
static inline void slewLimiter(slew_limiter_t *t,float slew_threshold)
{
   float temp = t->value - t->last_value;
   if(fabs(temp)>slew_threshold)t->value = t->last_value;
   t->last_value = t->value;
}

#ifdef __cplusplus
}
#endif
#endif
