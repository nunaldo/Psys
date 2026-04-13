#ifndef UTILS_H
#define UTILS_H

#include <stdbool.h>
#include "PID.h"    // Gains struct needed for the constant definitions below
#include "cancomm.h"

// Calibration
#define N_NODES 3

// ── Hardware ──────────────────────────────────────────────────────────────────
#define ADC_RANGE  4096
#define DAC_RANGE  4096

#define LED_PIN  15
#define LDR_PIN  A0

// ── Sensor model constants ────────────────────────────────────────────────────
#define VCC      3.3f
#define RF       10000.0f
#define VOUT_MIN 1e-5f
#define MAX_RLDR 20e6f
#define ALPHA    0.2f

#define M  -0.8f
#define B 6.08f   // COM8
//#define B 5.16f   // COM11

//#define M -0.88f // COM6
//#define B 6.75f // COM6

// ── Plant model ───────────────────────────────────────────────────────────────
// #define K_PLANT  27.0f

// ── PID gain sets ─────────────────────────────────────────────────────────────
//                                      {kp,       b,    ki,      kd,  N }
static const Gains GAINS_STEADY_STATE = { 160.0f, 1.0f, 700.0f, 0.0f, 1.0f };
// static const Gains GAINS_TRANSIENT    = { 160.0f, 1.0f, 700.0f, 0.0f, 1.0f };

// ── Mode thresholds ───────────────────────────────────────────────────────────
static constexpr float THRESH_HIGH = 7.0f;
static constexpr float THRESH_LOW  = 2.0f;

// ── Timer ─────────────────────────────────────────────────────────────────────
extern volatile bool timerFired;

#ifdef __cplusplus
extern "C" {
#endif
bool timer_callback(struct repeating_timer *t);
#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */
