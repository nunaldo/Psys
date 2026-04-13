#ifndef SENSOR_H
#define SENSOR_H

#include "utils.h"   // pin & constant defines (VCC, RF, M, B, …)

class Sensor {
public:
    // ── EMA filter ───────────────────────────────────────────────────────────
    // alpha ∈ (0, 1]: higher = faster response, lower = more smoothing.
    // Call resetEma() whenever the pin/context changes to avoid stale state.
    void  resetEma();
    int   readEma(int pin, float alpha);   // filtered raw ADC value
    float readLuxEma(int pin, float alpha); // full pipeline with EMA

    // Read <samples> ADC values on <pin> and return their integer average.
    int   readAverage(int pin, int samples);

    // Convert a 12-bit ADC reading to a voltage [V].
    float rawToVoltage(int raw);

    // Derive the LDR resistance [Ω] from the voltage divider output voltage.
    float rldrCalc(float vout);

    // Convert LDR resistance to illuminance [lux] using the log-linear model.
    float luxCalc(float rldr);

    // Compute the model intercept B for a known resistance at 500 lux.
    float bCalc(float rldr);

    // Print raw / vout / rldr / B to Serial (calibration helper).
    void  computeB();

    // Full pipeline: read the LDR on <pin> once and return lux.
    float readLux(int pin);

    // Full pipeline: average <samples> reads on <pin> and return lux.
    float readLuxAvg(int pin, int samples);

private:
    float _ema     = -1.0f;   // -1 sentinel → not yet initialised
};

#endif /* SENSOR_H */
