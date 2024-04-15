#include "ppmreceiver.h"
#include "timer.h"
#include "gpio.h"
#include "maths.h"
#define   NUM_CHANNEL 8
#include "math.h"
uint16_t rawValues[NUM_CHANNEL];
uint16_t last_rawValues[NUM_CHANNEL];
const uint16_t blankTime = 3000;
uint8_t pulseCounter=0;
uint32_t microsAtLastPulse=0;

void ppmcallback(){

    uint32_t previousMicros = microsAtLastPulse;
    microsAtLastPulse = micros();
    uint32_t time = microsAtLastPulse - previousMicros;
    if (time > blankTime) {
        // Blank detected: restart from channel 1
        pulseCounter = 0;
    }
    else {
        // Store times between pulses as channel values
        if (pulseCounter < NUM_CHANNEL) {
            rawValues[pulseCounter] = time;
            rawValues[pulseCounter]=val_change_limiter(last_rawValues[pulseCounter],rawValues[pulseCounter],-50,50);
            last_rawValues[pulseCounter] = rawValues[pulseCounter];
            ++pulseCounter;
        }
    }
}
void ppm_procces(){


}
