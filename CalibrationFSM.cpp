<<<<<<< HEAD
// CalibrationFSM.cpp
#include "CalibrationFSM.h"

extern void calSend(uint8_t, uint8_t, float);
extern void setDuty(float d);

CalibrationFSM::CalibrationFSM(float (&gainMatrix)[N_NODES][N_NODES]) 
: _gainMatrix(gainMatrix) { }

void CalibrationFSM::begin() {
    _state       = State::READ_OWN_DARK;
    _settleCount = 0;
}

void CalibrationFSM::trigger(Trigger t, float data) {
    switch (_state) {

    case State::READ_OWN_DARK:
        if (t != Trigger::TICK) break;
        _d0[0] = data;
        calSend(CMD_GET_LUX_MEASURE, 2, 0.0f);
        _transitionTo(State::WAIT_DARK_FROM_2);
        break;

    case State::WAIT_DARK_FROM_2:
        if (t != Trigger::LUX_RESPONSE) break;
        _d0[1] = data;
        calSend(CMD_GET_LUX_MEASURE, 3, 0.0f);
        _transitionTo(State::WAIT_DARK_FROM_3);
        break;

    case State::WAIT_DARK_FROM_3:
        if (t != Trigger::LUX_RESPONSE) break;
        _d0[2] = data;
        _settleCount = 0;
        setDuty(1.0f);
        _transitionTo(State::LED1_ON_SETTLE);
        break;

    case State::LED1_ON_SETTLE:
        if (t != Trigger::TICK) break;
        if (++_settleCount >= CAL_SETTLE_TICKS)
            _transitionTo(State::READ_LUX_SELF_LED1);
        break;

    case State::READ_LUX_SELF_LED1:
        if (t != Trigger::TICK) break;
        _luxMatrix[0][0] = data;
        calSend(CMD_GET_LUX_MEASURE, 2, 0.0f);
        _transitionTo(State::WAIT_LUX_FROM_2_LED1);
        break;

    case State::WAIT_LUX_FROM_2_LED1:
        if (t != Trigger::LUX_RESPONSE) break;
        _luxMatrix[1][0] = data;
        calSend(CMD_GET_LUX_MEASURE, 3, 0.0f);
        _transitionTo(State::WAIT_LUX_FROM_3_LED1);
        break;

    case State::WAIT_LUX_FROM_3_LED1:
        if (t != Trigger::LUX_RESPONSE) break;
        _luxMatrix[2][0] = data;
        setDuty(0.0f);
        calSend(CMD_SET_DUTY, 2, 1.0f);
        _transitionTo(State::WAIT_ACK_LED2_ON);
        break;

    case State::WAIT_ACK_LED2_ON:
        if (t != Trigger::DUTY_ACK) break;
        _settleCount = 0;
        _transitionTo(State::LED2_ON_SETTLE);
        break;

    case State::LED2_ON_SETTLE:
        if (t != Trigger::TICK) break;
        if (++_settleCount >= CAL_SETTLE_TICKS)
            _transitionTo(State::READ_LUX_SELF_LED2);
        break;    

    case State::READ_LUX_SELF_LED2:
        if (t != Trigger::TICK) break;
        _luxMatrix[0][1] = data;
        calSend(CMD_GET_LUX_MEASURE, 2, 0.0f);
        _transitionTo(State::WAIT_LUX_FROM_2_LED2);
        break;

    case State::WAIT_LUX_FROM_2_LED2:
        if (t != Trigger::LUX_RESPONSE) break;
        _luxMatrix[1][1] = data;
        calSend(CMD_GET_LUX_MEASURE, 3, 0.0f);
        _transitionTo(State::WAIT_LUX_FROM_3_LED2);
        break;

    case State::WAIT_LUX_FROM_3_LED2:
        if (t != Trigger::LUX_RESPONSE) break;
        _luxMatrix[2][1] = data;
        calSend(CMD_SET_DUTY, 2, 0.0f);
        _transitionTo(State::WAIT_ACK_LED2_OFF);
        break;    

    case State::WAIT_ACK_LED2_OFF:
        if (t != Trigger::DUTY_ACK) break;
        calSend(CMD_SET_DUTY, 3, 1.0f);
        _transitionTo(State::WAIT_ACK_LED3_ON);
        break;

    case State::WAIT_ACK_LED3_ON:
        if (t != Trigger::DUTY_ACK) break;
        _settleCount = 0;
        _transitionTo(State::LED3_ON_SETTLE);
        break;
    
    case State::LED3_ON_SETTLE:
        if (t != Trigger::TICK) break;
        if (++_settleCount >= CAL_SETTLE_TICKS)
            _transitionTo(State::READ_LUX_SELF_LED3);
        break;   

    case State::READ_LUX_SELF_LED3:
        if (t != Trigger::TICK) break;
        _luxMatrix[0][2] = data;
        calSend(CMD_GET_LUX_MEASURE, 2, 0.0f);
        _transitionTo(State::WAIT_LUX_FROM_2_LED3);
        break;

    case State::WAIT_LUX_FROM_2_LED3:
        if (t != Trigger::LUX_RESPONSE) break;
        _luxMatrix[1][2] = data;
        calSend(CMD_GET_LUX_MEASURE, 3, 0.0f);
        _transitionTo(State::WAIT_LUX_FROM_3_LED3);
        break;

    case State::WAIT_LUX_FROM_3_LED3:
        if (t != Trigger::LUX_RESPONSE) break;
        _luxMatrix[2][2] = data;
        calSend(CMD_SET_DUTY, 3, 0.0f);
        _computeGains();
        _transitionTo(State::IDLE);
        break;

    case State::IDLE: break;
    default: break;
    }
}

// gain[sensor node][light node]
void CalibrationFSM::_computeGains() {
    // _gainMatrix[0][0] = _luxMatrix[0][0] - _d0[0];
    // _gainMatrix[0][1] = _luxMatrix[0][1] - _d0[0];
    // _gainMatrix[1][0] = _luxMatrix[1][0] - _d0[1];
    // _gainMatrix[1][1] = _luxMatrix[1][1] - _d0[1];

    for (int s = 0; s < N_NODES; s++)
        for (int l = 0; l < N_NODES; l++)
            _gainMatrix[s][l] = _luxMatrix[s][l] - _d0[s];
}

void CalibrationFSM::_transitionTo(State s) {
    // Optional: log transition for debugging
    // Serial.printf("CAL: %d -> %d\n", (int)_state, (int)s);
    _state = s;
}
=======
// CalibrationFSM.cpp
#include "CalibrationFSM.h"

extern void calSend(uint8_t, uint8_t, float);
extern void shareCalibrationParameter(uint8_t row_index, uint8_t parameter_index, float value);
extern void setDuty(float d);

CalibrationFSM::CalibrationFSM(float (&gainMatrix)[N_NODES][N_NODES])
: _gainMatrix(gainMatrix) { }

void CalibrationFSM::begin() {
    _state       = State::READ_OWN_DARK;
    _settleCount = 0;
}

void CalibrationFSM::trigger(Trigger t, float data) {
    switch (_state) {

    case State::READ_OWN_DARK:
        if (t != Trigger::TICK) break;
        _d0[0] = data;
        calSend(CMD_GET_LUX_MEASURE, 2, 0.0f);
        _transitionTo(State::WAIT_DARK_FROM_2);
        break;

    case State::WAIT_DARK_FROM_2:
        if (t != Trigger::LUX_RESPONSE) break;
        _d0[1] = data;
        calSend(CMD_GET_LUX_MEASURE, 3, 0.0f);
        _transitionTo(State::WAIT_DARK_FROM_3);
        break;

    case State::WAIT_DARK_FROM_3:
        if (t != Trigger::LUX_RESPONSE) break;
        _d0[2] = data;
        _settleCount = 0;
        setDuty(1.0f);
        _transitionTo(State::LED1_ON_SETTLE);
        break;

    case State::LED1_ON_SETTLE:
        if (t != Trigger::TICK) break;
        if (++_settleCount >= CAL_SETTLE_TICKS)
            _transitionTo(State::READ_LUX_SELF_LED1);
        break;

    case State::READ_LUX_SELF_LED1:
        if (t != Trigger::TICK) break;
        _luxMatrix[0][0] = data;
        calSend(CMD_GET_LUX_MEASURE, 2, 0.0f);
        _transitionTo(State::WAIT_LUX_FROM_2_LED1);
        break;

    case State::WAIT_LUX_FROM_2_LED1:
        if (t != Trigger::LUX_RESPONSE) break;
        _luxMatrix[1][0] = data;
        calSend(CMD_GET_LUX_MEASURE, 3, 0.0f);
        _transitionTo(State::WAIT_LUX_FROM_3_LED1);
        break;

    case State::WAIT_LUX_FROM_3_LED1:
        if (t != Trigger::LUX_RESPONSE) break;
        _luxMatrix[2][0] = data;
        setDuty(0.0f);
        calSend(CMD_SET_DUTY, 2, 1.0f);
        _transitionTo(State::WAIT_ACK_LED2_ON);
        break;

    case State::WAIT_ACK_LED2_ON:
        if (t != Trigger::DUTY_ACK) break;
        _settleCount = 0;
        _transitionTo(State::LED2_ON_SETTLE);
        break;

    case State::LED2_ON_SETTLE:
        if (t != Trigger::TICK) break;
        if (++_settleCount >= CAL_SETTLE_TICKS)
            _transitionTo(State::READ_LUX_SELF_LED2);
        break;

    case State::READ_LUX_SELF_LED2:
        if (t != Trigger::TICK) break;
        _luxMatrix[0][1] = data;
        calSend(CMD_GET_LUX_MEASURE, 2, 0.0f);
        _transitionTo(State::WAIT_LUX_FROM_2_LED2);
        break;

    case State::WAIT_LUX_FROM_2_LED2:
        if (t != Trigger::LUX_RESPONSE) break;
        _luxMatrix[1][1] = data;
        calSend(CMD_GET_LUX_MEASURE, 3, 0.0f);
        _transitionTo(State::WAIT_LUX_FROM_3_LED2);
        break;

    case State::WAIT_LUX_FROM_3_LED2:
        if (t != Trigger::LUX_RESPONSE) break;
        _luxMatrix[2][1] = data;
        calSend(CMD_SET_DUTY, 2, 0.0f);
        _transitionTo(State::WAIT_ACK_LED2_OFF);
        break;

    case State::WAIT_ACK_LED2_OFF:
        if (t != Trigger::DUTY_ACK) break;
        calSend(CMD_SET_DUTY, 3, 1.0f);
        _transitionTo(State::WAIT_ACK_LED3_ON);
        break;

    case State::WAIT_ACK_LED3_ON:
        if (t != Trigger::DUTY_ACK) break;
        _settleCount = 0;
        _transitionTo(State::LED3_ON_SETTLE);
        break;

    case State::LED3_ON_SETTLE:
        if (t != Trigger::TICK) break;
        if (++_settleCount >= CAL_SETTLE_TICKS)
            _transitionTo(State::READ_LUX_SELF_LED3);
        break;

    case State::READ_LUX_SELF_LED3:
        if (t != Trigger::TICK) break;
        _luxMatrix[0][2] = data;
        calSend(CMD_GET_LUX_MEASURE, 2, 0.0f);
        _transitionTo(State::WAIT_LUX_FROM_2_LED3);
        break;

    case State::WAIT_LUX_FROM_2_LED3:
        if (t != Trigger::LUX_RESPONSE) break;
        _luxMatrix[1][2] = data;
        calSend(CMD_GET_LUX_MEASURE, 3, 0.0f);
        _transitionTo(State::WAIT_LUX_FROM_3_LED3);
        break;

    case State::WAIT_LUX_FROM_3_LED3:
        if (t != Trigger::LUX_RESPONSE) break;
        _luxMatrix[2][2] = data;
        calSend(CMD_SET_DUTY, 3, 0.0f);
        _computeGains();
        _transitionTo(State::IDLE);
        break;

    case State::IDLE:
        break;

    default:
        break;
    }
}

// gain[sensor node][light node]
void CalibrationFSM::_computeGains() {
    for (int s = 0; s < N_NODES; ++s) {
        for (int l = 0; l < N_NODES; ++l) {
            _gainMatrix[s][l] = _luxMatrix[s][l] - _d0[s];
        }
    }

    for (int s = 0; s < N_NODES; ++s) {
        shareCalibrationParameter((uint8_t)s, 0, _d0[s]);
        for (int l = 0; l < N_NODES; ++l) {
            shareCalibrationParameter((uint8_t)s, (uint8_t)(l + 1), _gainMatrix[s][l]);
        }
    }
}

void CalibrationFSM::_transitionTo(State s) {
    _state = s;
}

>>>>>>> 4f8de03 (Update novo3 control stack and test artifacts)
