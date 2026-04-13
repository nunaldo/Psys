#ifndef CALIBRATIONFSM_H
#define CALIBRATIONFSM_H

#include <Arduino.h>
#include "cancomm.h"
#include "utils.h"

#define CAL_SETTLE_TICKS 200

// Node index 0 is always "self" (this MCU). Nodes 1..N_NODES-1 are remote.
// The FSM iterates over every LED node in turn, and for each one collects a
// lux reading from every sensor node (self + all remotes) before moving on.

class CalibrationFSM {
public:
    CalibrationFSM(float (&gainMatrix)[N_NODES][N_NODES]);

    enum class Trigger {
        TICK,           // periodic ISR / loop tick carrying own lux in `data`
        LUX_RESPONSE,   // a remote node replied with its lux value
        DUTY_ACK        // a remote node acknowledged a CMD_SET_DUTY command
    };

    void begin();                              // (re)start calibration
    void trigger(Trigger t, CoreMail mail);
    void trigger(Trigger t);
    bool isIdle() const { return _state == State::IDLE; }

private:
    // -----------------------------------------------------------------------
    // States – the loop index _currentLED drives which LED is active.
    //
    //  IDLE
    //   └─ begin() ──► READ_OWN_DARK
    //                     │ own dark stored; broadcast CMD_GET_LUX_MEASURE
    //                     ▼
    //               WAIT_DARK_REMOTES          (collect N_NODES-1 responses)
    //                     │ all dark values stored
    //                     │ _currentLED = 0
    //                     ▼
    //  ┌──────────── LED_ON_ISSUE ◄────────────────────────────────────┐
    //  │                 │ if LED==0: setDuty(1); else calSend SET_DUTY│
    //  │                 ▼                                              │
    //  │           LED_ON_SETTLE  (wait CAL_SETTLE_TICKS)              │
    //  │                 │                                              │
    //  │                 ▼                                              │
    //  │          READ_OWN_LIT                                          │
    //  │                 │ own lit lux stored; broadcast GET_LUX        │
    //  │                 ▼                                              │
    //  │        WAIT_LIT_REMOTES           (collect N_NODES-1 responses)│
    //  │                 │ all lit values stored                        │
    //  │                 │ turn LED off; ++_currentLED                  │
    //  │                 └────────────────────────────────► if more LEDs┘
    //  │                                                    else ──────►IDLE
    //  └─────────────────────────────────────────────────────────────────┘
    // -----------------------------------------------------------------------
    enum class State {
        IDLE,
        READ_OWN_DARK,       // capture own dark reading on next TICK
        WAIT_DARK_REMOTES,   // waiting for dark readings from remotes
        LED_ON_ISSUE,        // send "LED on" command (or setDuty for self)
        WAIT_ACK_LED_ON,     // wait for DUTY_ACK from the remote LED node
        LED_ON_SETTLE,       // wait CAL_SETTLE_TICKS before reading
        READ_OWN_LIT,        // capture own lit reading on next TICK
        WAIT_LIT_REMOTES,    // waiting for lit readings from remotes
    };

    State _state       = State::IDLE;
    int   _settleCount = 0;
    int   _currentLED  = 0;   // which LED node is currently active (0 = self)
    int   _pending     = 0;   // how many remote responses are still expected

    float _d0[N_NODES]                = {};  // dark readings, indexed by sensor node
    float _luxMatrix[N_NODES][N_NODES]= {};  // [sensor][led]

    float (*_gainMatrix)[N_NODES];           // reference to caller-owned gain matrix

    // ---- helpers -----------------------------------------------------------
    void _broadcastGetLux();     // ask all remote nodes for their lux
    void _turnOnLED(int led);    // turn on LED `led` (self or remote)
    void _turnOffLED(int led);   // turn off LED `led`
    void _computeGains();
    void _transitionTo(State s);
};

#endif // CALIBRATIONFSM_H