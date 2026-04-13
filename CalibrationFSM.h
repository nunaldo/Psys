#ifndef CALIBRATIONFSM_H
#define CALIBRATIONFSM_H

#include <Arduino.h>
#include "cancomm.h"
#include "utils.h"

#define CAL_SETTLE_TICKS 200

class CalibrationFSM {
public:
    CalibrationFSM(float (&gainMatrix)[N_NODES][N_NODES]);
    enum class Trigger { TICK, LUX_RESPONSE, DUTY_ACK };

    void begin(); // reset to idle
    void trigger(Trigger t, float data = 0.0f);
    bool isIdle() const { return _state == State::IDLE; }

private:
    enum class State {
        IDLE,
        READ_OWN_DARK,
        WAIT_DARK_FROM_2,
        WAIT_DARK_FROM_3,      // novo
        LED1_ON_SETTLE,
        READ_LUX_SELF_LED1,
        WAIT_LUX_FROM_2_LED1,
        WAIT_LUX_FROM_3_LED1,  // novo 
        WAIT_ACK_LED2_ON,
        LED2_ON_SETTLE,
        READ_LUX_SELF_LED2,
        WAIT_LUX_FROM_2_LED2,
        WAIT_LUX_FROM_3_LED2,  // novo 
        WAIT_ACK_LED2_OFF,     // novo
        WAIT_ACK_LED3_ON,      // novo
        LED3_ON_SETTLE,        // novo
        READ_LUX_SELF_LED3,    // novo
        WAIT_LUX_FROM_2_LED3,  // novo
        WAIT_LUX_FROM_3_LED3,  // novo 
    };

    State  _state = State::IDLE;
    int    _settleCount = 0;
    float  _d0[N_NODES] = {};
    float  _luxMatrix[N_NODES][N_NODES] = {};

    // external pointers
    float (*_gainMatrix)[N_NODES]; // main gainMatrix 

    void _computeGains();
    void _transitionTo(State s);
};

#endif