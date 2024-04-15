#ifndef _SCHEDULER_
#define _SCHEDULER_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

typedef uint32_t time_us;

typedef struct {
    void (*exec)();
    time_us execution_time_us;
    time_us execution_cycle_us;
    time_us last_exec_time_us;
    uint32_t period;
} task_t;

void start_scheduler();
void init_scheduler();
#ifdef __cplusplus
}
#endif
#endif
