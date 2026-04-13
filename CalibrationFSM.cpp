// CalibrationFSM.cpp
#include "CalibrationFSM.h"

extern void calSend(uint8_t, uint8_t, float);
extern void shareCalibrationParameter(uint8_t row_index, uint8_t parameter_index, float value);
extern void setDuty(float d);
extern float lux;

// CAN node id 0 = broadcast; physical index i maps to CAN id (i+1) for unicast.
static inline uint8_t nodeId(int i) { return static_cast<uint8_t>(i + 1); }

// ---------------------------------------------------------------------------
CalibrationFSM::CalibrationFSM(float (&gainMatrix)[N_NODES][N_NODES])
    : _gainMatrix(gainMatrix) {}

// ---------------------------------------------------------------------------
void CalibrationFSM::begin() {
    _state       = State::READ_OWN_DARK;
    _settleCount = 0;
    _currentLED  = 0;
    _pending     = 0;
}

// ---------------------------------------------------------------------------
void CalibrationFSM::trigger(Trigger t, CoreMail mail) {
    // Compute sensor array index once. sourceId 1..N_NODES maps to idx 0..N_NODES-1.
    // idx 0 would be self – remotes should never send sourceId==MY_NODE_ID (filtered
    // in handleResponse), so treat it as invalid alongside anything out of range.
    const uint8_t idx = (mail.sourceId > 0 && mail.sourceId <= N_NODES)
                        ? mail.sourceId - 1
                        : UINT8_MAX;  // sentinel: invalid / self-echo

    switch (_state) {

    // ── Phase 1: collect dark readings ──────────────────────────────────────

    case State::READ_OWN_DARK:
        if (t != Trigger::TICK) break;
        _d0[0] = lux;
        _broadcastGetLux();
        _pending = N_NODES - 1;
        _transitionTo(State::WAIT_DARK_REMOTES);
        break;

    case State::WAIT_DARK_REMOTES:
        if (t != Trigger::LUX_RESPONSE || idx == UINT8_MAX) break;
        _d0[idx] = mail.data;
        if (--_pending > 0) break;
        _currentLED = 0;
        _transitionTo(State::LED_ON_ISSUE);
        break;

    // ── Phase 2: per-LED illumination loop ──────────────────────────────────

    case State::LED_ON_ISSUE:
        if (_currentLED == 0) {
            _turnOnLED(0);
            _settleCount = 0;
            _transitionTo(State::LED_ON_SETTLE);
        } else {
            _turnOnLED(_currentLED);
            _transitionTo(State::WAIT_ACK_LED_ON);
        }
        break;

    case State::WAIT_ACK_LED_ON:
        if (t != Trigger::DUTY_ACK) break;
        _settleCount = 0;
        _transitionTo(State::LED_ON_SETTLE);
        break;

    case State::LED_ON_SETTLE:
        if (t != Trigger::TICK) break;
        if (++_settleCount >= CAL_SETTLE_TICKS)
            _transitionTo(State::READ_OWN_LIT);
        break;

    case State::READ_OWN_LIT:
        if (t != Trigger::TICK) break;
        _luxMatrix[0][_currentLED] = lux;
        _broadcastGetLux();
        _pending = N_NODES - 1;
        _transitionTo(State::WAIT_LIT_REMOTES);
        break;

    case State::WAIT_LIT_REMOTES: {
        if (t != Trigger::LUX_RESPONSE || idx == UINT8_MAX) break;
        _luxMatrix[idx][_currentLED] = mail.data;
        if (--_pending > 0) break;
        _turnOffLED(_currentLED);
        if (++_currentLED < N_NODES) {
            _transitionTo(State::LED_ON_ISSUE);
            trigger(Trigger::TICK, mail);  // re-enter immediately; LED_ON_ISSUE is guardless
        } else {
            _computeGains();
            _transitionTo(State::IDLE);
        }
        break;
    }

    case State::IDLE:
    default:
        break;
    }
}

// ---------------------------------------------------------------------------
void CalibrationFSM::trigger(Trigger t) {
    switch (_state) {

    case State::READ_OWN_DARK:
        if (t != Trigger::TICK) break;
        _d0[0] = lux;
        _broadcastGetLux();
        _pending = N_NODES - 1;
        _transitionTo(State::WAIT_DARK_REMOTES);
        break;

    case State::LED_ON_ISSUE:
        if (_currentLED == 0) {
            _turnOnLED(0);
            _settleCount = 0;
            _transitionTo(State::LED_ON_SETTLE);
        } else {
            _turnOnLED(_currentLED);
            _transitionTo(State::WAIT_ACK_LED_ON);
        }
        break;

    case State::LED_ON_SETTLE:
        if (t != Trigger::TICK) break;
        if (++_settleCount >= CAL_SETTLE_TICKS)
            _transitionTo(State::READ_OWN_LIT);
        break;

    case State::READ_OWN_LIT:
        if (t != Trigger::TICK) break;
        _luxMatrix[0][_currentLED] = lux;
        _broadcastGetLux();
        _pending = N_NODES - 1;
        _transitionTo(State::WAIT_LIT_REMOTES);
        break;

    case State::IDLE:
    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void CalibrationFSM::_broadcastGetLux() {
    calSend(CMD_GET_LUX_MEASURE, 0, 0.0f);  // 0 = broadcast to all nodes
}

void CalibrationFSM::_turnOnLED(int led) {
    if (led == 0)
        setDuty(1.0f);
    else
        calSend(CMD_SET_DUTY, nodeId(led), 1.0f);
}

void CalibrationFSM::_turnOffLED(int led) {
    if (led == 0)
        setDuty(0.0f);
    else
        calSend(CMD_SET_DUTY, nodeId(led), 0.0f);
}

// gain[sensor][led] = lux_with_led_on[sensor][led] - dark[sensor]
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
    // Serial.printf("CAL: %d -> %d\n", (int)_state, (int)s);
    _state = s;
}
