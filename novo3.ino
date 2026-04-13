#include "utils.h"
#include "PID.h"
#include "Sensor.h"
#include "cancomm.h"
#include "CalibrationFSM.h"
#include "controller.h"
#include <math.h>

#define CAP_SIZE 300
static float capLux[CAP_SIZE], capSetpoint[CAP_SIZE], capDuty[CAP_SIZE];
static unsigned long capDt[CAP_SIZE];
static int capIdx = 0;
static bool capturing = false;
static bool live = false;

float setpointLux = 0.0f;
float refLowBound = 5.0f;
float refHighBound = 20.0f;
float dutyCycle = 0.0f;
float lux = 0.0f;
float u = 0.0f;
float u_ff = 0.0f;

float gainMatrix[N_NODES][N_NODES] = {};
float baselineLux[N_NODES] = {};
NeighborData distributedNodeData[N_NODES];

enum State { FF, PID_STATE, DIST_STATE };
State currentState = FF;

Sensor sensor;
pid controller(0.01f, GAINS_STEADY_STATE, 0.1f, 0.0f, 4095.0f);
struct repeating_timer timer;

SuperController distributedController(0, 1.0f, gainMatrix, 0.0f);
NeighborData neighborNodeA;
NeighborData neighborNodeB;

CalibrationFSM calibration(gainMatrix);

constexpr unsigned long DIST_OPTIMIZATION_PERIOD_US = 100000;
constexpr unsigned long DIST_BROADCAST_PERIOD_US = 50000;
constexpr float DIST_OUTPUT_BLEND = 0.20f;
constexpr float DIST_OUTPUT_MAX_STEP = 0.01f;

static float distributedTargetDuty = 0.0f;
static float distributedAuxCache = 0.0f;
static unsigned long lastDistributedOptimizationUs = 0;
static unsigned long lastDistributedBroadcastUs = 0;

void WriteCorezeroone(CoreMail localCopy) {
    if (!queue_try_add(&xQueue0to1, &localCopy)) {
        Serial.println("Queue Full!");
    }
}

CoreMail createMail(uint8_t command, uint8_t targetId, uint8_t sourceId, float data) {
    CoreMail mail;
    mail.command = command;
    mail.targetId = targetId;
    mail.sourceId = sourceId;
    mail.data = data;
    mail.ackerr = -1;
    mail.response = false;
    mail.isNew = true;
    return mail;
}

CoreMail CreateResponseMail(CoreMail originalMail, float data) {
    CoreMail resp = originalMail;
    resp.response = true;
    resp.targetId = originalMail.sourceId;
    resp.sourceId = (uint8_t)MY_NODE_ID;
    resp.data = data;
    return resp;
}

void calSend(uint8_t cmd, uint8_t target, float data) {
    WriteCorezeroone(createMail(cmd, target, (uint8_t)MY_NODE_ID, data));
}

void setDuty(float d) {
    u_ff = d * 4095.0f;
}

static float clampUnitValue(float value) {
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < 0.0f) {
        return 0.0f;
    }
    return value;
}

static float moveDutyToward(float current, float target) {
    const float blended = current + (DIST_OUTPUT_BLEND * (target - current));
    const float delta = blended - current;
    if (delta > DIST_OUTPUT_MAX_STEP) {
        return clampUnitValue(current + DIST_OUTPUT_MAX_STEP);
    }
    if (delta < -DIST_OUTPUT_MAX_STEP) {
        return clampUnitValue(current - DIST_OUTPUT_MAX_STEP);
    }
    return clampUnitValue(blended);
}

static void resetDistributedLoopState() {
    distributedTargetDuty = dutyCycle;
    distributedAuxCache = 0.0f;
    lastDistributedOptimizationUs = 0;
    lastDistributedBroadcastUs = 0;
}

static uint8_t controllerIndexFromNodeId(uint8_t node_id) {
    if (node_id >= 1 && node_id <= N_NODES) {
        return node_id - 1;
    }
    return 0;
}

static float selectDistributedReference() {
    if (setpointLux > 0.0f) {
        return setpointLux;
    }
    if (refHighBound > 0.0f) {
        return refHighBound;
    }
    return refLowBound;
}

static void syncDistributedController() {
    distributedController.setReference(selectDistributedReference());
}

static void updateDistributedNodeData(uint8_t source_node_id, Command cmd, float value) {
    const uint8_t idx = controllerIndexFromNodeId(source_node_id);
    if (cmd == CMD_DIST_STATE_DUTY) {
        distributedNodeData[idx].u = value;
    } else if (cmd == CMD_DIST_STATE_AUX) {
        distributedNodeData[idx].auxiliary = value;
    }
}

static void refreshNeighborStateViews() {
    const uint8_t my_idx = controllerIndexFromNodeId((uint8_t)MY_NODE_ID);
    neighborNodeA = distributedNodeData[(my_idx + 1) % N_NODES];
    neighborNodeB = distributedNodeData[(my_idx + 2) % N_NODES];
}

static void applyCalibrationParameter(uint8_t packed_info, float value) {
    const uint8_t row_index = (packed_info >> 4) & 0x0F;
    const uint8_t parameter_index = packed_info & 0x0F;
    if (row_index >= N_NODES) {
        return;
    }

    if (parameter_index == 0) {
        baselineLux[row_index] = value;
        return;
    }

    const uint8_t gain_column = parameter_index - 1;
    if (gain_column < N_NODES) {
        gainMatrix[row_index][gain_column] = value;
    }
}

void shareCalibrationParameter(uint8_t row_index, uint8_t parameter_index, float value) {
    const uint8_t packed_info = (uint8_t)((row_index << 4) | (parameter_index & 0x0F));
    applyCalibrationParameter(packed_info, value);
    for (uint8_t target = 1; target <= N_NODES; ++target) {
        if (target == MY_NODE_ID) {
            continue;
        }
        WriteCorezeroone(createMail(CMD_DIST_MODEL, target, packed_info, value));
    }
}

static void broadcastDistributedState(float duty_unit, float auxiliary) {
    const uint8_t my_idx = controllerIndexFromNodeId((uint8_t)MY_NODE_ID);
    distributedNodeData[my_idx].u = duty_unit;
    distributedNodeData[my_idx].auxiliary = auxiliary;

    WriteCorezeroone(createMail(CMD_DIST_STATE_DUTY, 0, (uint8_t)MY_NODE_ID, duty_unit));
    WriteCorezeroone(createMail(CMD_DIST_STATE_AUX, 0, (uint8_t)MY_NODE_ID, auxiliary));
}

static bool handleInternalDistributedMessage(const CoreMail& mail) {
    switch ((Command)mail.command) {
        case CMD_DIST_STATE_DUTY:
        case CMD_DIST_STATE_AUX:
            updateDistributedNodeData(mail.sourceId, (Command)mail.command, mail.data);
            return true;

        case CMD_DIST_MODEL:
            applyCalibrationParameter(mail.sourceId, mail.data);
            return true;

        default:
            return false;
    }
}

void handleRequest(CoreMail mail) {
    const Command cmd = (Command)mail.command;
    const float data = mail.data;
    float response = 0.0f;

    switch (cmd) {
        case CMD_SET_DUTY:
            setDuty(data);
            response = 1.0f;
            break;

        case CMD_SET_LUX_REF:
            setpointLux = data;
            if (currentState == DIST_STATE) {
                syncDistributedController();
            } else {
                currentState = PID_STATE;
            }
            response = 1.0f;
            break;

        case CMD_SET_LOW_BOUND:
            refLowBound = data;
            syncDistributedController();
            response = 1.0f;
            break;

        case CMD_GET_LOW_BOUND:
            response = refLowBound;
            break;

        case CMD_SET_HIGH_BOUND:
            refHighBound = data;
            syncDistributedController();
            response = 1.0f;
            break;

        case CMD_GET_HIGH_BOUND:
            response = refHighBound;
            break;

        case CMD_GET_CURRENT_BOUND:
            response = distributedController.getReference();
            break;

        case CMD_GET_LUX_MEASURE:
            response = lux;
            break;

        case CMD_GET_LUX_REF:
            response = setpointLux;
            break;

        case CMD_GET_DUTY:
            response = dutyCycle;
            break;

        case CMD_SET_FEEDBACK:
            distributedController.setFeedbackEnabled(data >= 0.5f);
            response = distributedController.isFeedbackEnabled() ? 1.0f : 0.0f;
            break;

        case CMD_GET_FEEDBACK:
            response = distributedController.isFeedbackEnabled() ? 1.0f : 0.0f;
            break;

        case CMD_SET_ENERGY_COST:
            distributedController.setCost(data);
            syncDistributedController();
            response = 1.0f;
            break;

        case CMD_GET_ENERGY_COST:
            response = distributedController.getCost();
            break;

        case CMD_SET_CONTROL_MODE: {
            const int mode = (int)data;
            if (mode == 0) {
                currentState = FF;
                resetDistributedLoopState();
                response = 1.0f;
            } else if (mode == 1) {
                currentState = PID_STATE;
                resetDistributedLoopState();
                response = 1.0f;
            } else if (mode == 2) {
                currentState = DIST_STATE;
                distributedController.setAlgorithm(distributedController.getAlgorithm());
                syncDistributedController();
                resetDistributedLoopState();
                response = 1.0f;
            } else {
                response = 0.0f;
            }
            break;
        }

        case CMD_GET_CONTROL_MODE:
            response = (float)currentState;
            break;

        case CMD_SET_DIST_ALGO: {
            const int algo = (int)data;
            if (algo >= (int)PRIMAL_DUAL && algo <= (int)CONSENSUS) {
                distributedController.setAlgorithm((DistributedAlgorithm)algo);
                resetDistributedLoopState();
                response = 1.0f;
            } else {
                response = 0.0f;
            }
            break;
        }

        case CMD_GET_DIST_ALGO:
            response = (float)distributedController.getAlgorithm();
            break;

        case CMD_GET_NODE_INFO:
            response = (float)MY_NODE_ID;
            break;

        case CMD_CAL:
            if (MY_NODE_ID != 1) {
                response = 0.0f;
                break;
            }
            resetDistributedLoopState();
            calibration.begin();
            response = 1.0f;
            break;

        case CMD_PRINT_MATRIX:
            for (int s = 0; s < N_NODES; ++s) {
                Serial.print("d0[");
                Serial.print(s + 1);
                Serial.print("] = ");
                Serial.println(baselineLux[s]);
                for (int l = 0; l < N_NODES; ++l) {
                    Serial.print("G[");
                    Serial.print(s + 1);
                    Serial.print("][");
                    Serial.print(l + 1);
                    Serial.print("] = ");
                    Serial.println(gainMatrix[s][l]);
                }
            }
            response = 1.0f;
            break;

        case CMD_LIVE:
            live = (data == 1.0f);
            response = 1.0f;
            break;

        default:
            break;
    }

    WriteCorezeroone(CreateResponseMail(mail, response));
}

void handleResponse(CoreMail mail) {
    switch ((Command)mail.command) {
        case CMD_GET_LUX_MEASURE:
            calibration.trigger(CalibrationFSM::Trigger::LUX_RESPONSE, mail.data);
            break;

        case CMD_SET_DUTY:
            calibration.trigger(CalibrationFSM::Trigger::DUTY_ACK, mail.data);
            break;

        default:
            break;
    }
}

void executeCommand(CoreMail mail) {
    if (!mail.response && handleInternalDistributedMessage(mail)) {
        return;
    }

    if (!mail.response) {
        handleRequest(mail);
    } else {
        handleResponse(mail);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    analogWriteFreq(1000);
    analogWriteResolution(12);
    analogReadResolution(12);
    analogWriteRange(DAC_RANGE);
    pinMode(LED_PIN, OUTPUT);

    add_repeating_timer_ms(-10, timer_callback, NULL, &timer);
}

CoreMail incoming;

void loop() {
    if (queue_try_remove(&xQueue1to0, &incoming)) {
        executeCommand(incoming);
    }

    if (!timerFired) {
        return;
    }
    timerFired = false;

    static unsigned long t_prev = 0;
    const unsigned long t_now = micros();
    const unsigned long dt = t_now - t_prev;
    t_prev = t_now;

    lux = sensor.readLuxEma(LDR_PIN, ALPHA);

    distributedController.setNodeId((uint8_t)MY_NODE_ID);
    distributedController.setBaseline(
        baselineLux[controllerIndexFromNodeId((uint8_t)MY_NODE_ID)]);
    refreshNeighborStateViews();

    calibration.trigger(CalibrationFSM::Trigger::TICK, lux);

    bool broadcast_state = false;
    float aux_to_broadcast = 0.0f;

    if (!calibration.isIdle()) {
        u = u_ff;
    } else {
        switch (currentState) {
            case FF:
                u = u_ff;
                break;

            case PID_STATE:
                u = controller.compute_control(setpointLux, lux);
                break;

            case DIST_STATE: {
                if ((lastDistributedOptimizationUs == 0) ||
                    ((t_now - lastDistributedOptimizationUs) >= DIST_OPTIMIZATION_PERIOD_US)) {
                    distributedTargetDuty = distributedController.runControlCycle(neighborNodeA, neighborNodeB, lux);
                    distributedAuxCache = distributedController.getAuxiliaryData();
                    lastDistributedOptimizationUs = t_now;
                }

                const float appliedDuty = moveDutyToward(dutyCycle, distributedTargetDuty);
                u = appliedDuty * 4095.0f;

                if ((lastDistributedBroadcastUs == 0) ||
                    ((t_now - lastDistributedBroadcastUs) >= DIST_BROADCAST_PERIOD_US)) {
                    aux_to_broadcast = distributedAuxCache;
                    broadcast_state = true;
                    lastDistributedBroadcastUs = t_now;
                }
                break;
            }
        }
    }

    analogWrite(LED_PIN, (int)u);
    dutyCycle = u / 4095.0f;

    {
        const uint8_t my_idx = controllerIndexFromNodeId((uint8_t)MY_NODE_ID);
        distributedNodeData[my_idx].u = dutyCycle;
        if (currentState == DIST_STATE) {
            distributedNodeData[my_idx].auxiliary = distributedAuxCache;
        } else if (!broadcast_state) {
            distributedNodeData[my_idx].auxiliary = 0.0f;
        }
    }

    if (broadcast_state) {
        broadcastDistributedState(dutyCycle, aux_to_broadcast);
    }

    if (live) {
        Serial.print("0.0, ");
        Serial.print("30.0, ");
        Serial.print(setpointLux);
        Serial.print(' ');
        Serial.print(lux);
        Serial.print(' ');
        Serial.print(dutyCycle, 2);
        Serial.println();
    }

    if (capturing) {
        capLux[capIdx] = lux;
        capSetpoint[capIdx] = setpointLux;
        capDuty[capIdx] = dutyCycle;
        capDt[capIdx] = dt;
        ++capIdx;
        if (capIdx >= CAP_SIZE) {
            capturing = false;
            Serial.println("idx,setpoint,lux,duty,dt_us");
            for (int i = 0; i < CAP_SIZE; ++i) {
                Serial.print(i);
                Serial.print(',');
                Serial.print(capSetpoint[i]);
                Serial.print(',');
                Serial.print(capLux[i], 3);
                Serial.print(',');
                Serial.print(capDuty[i], 4);
                Serial.print(',');
                Serial.println(capDt[i]);
            }
        }
    }
}

