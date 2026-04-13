#include "utils.h"
#include <Arduino.h>

volatile bool timerFired = false;

bool timer_callback(struct repeating_timer *t) {
    timerFired = true;   // set flag consumed by the main loop
    return true;         // keep the timer repeating
}
