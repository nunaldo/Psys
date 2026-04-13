#include "Sensor.h"
#include <Arduino.h>
#include <math.h>

// ── EMA filter ───────────────────────────────────────────────────────────────

void Sensor::resetEma() {
    _ema = -1.0f;
}

// Returns a filtered ADC reading.
// On the first call (or after resetEma()) the raw sample is used directly
// so the filter doesn't drag up from zero.
int Sensor::readEma(int pin, float alpha) {
    float raw = (float)analogRead(pin);
    if (_ema < 0.0f) {
        _ema = raw;          // seed with first real sample
    } else {
        _ema = alpha * raw + (1.0f - alpha) * _ema;
    }
    return (int)(_ema + 0.5f);  // round to nearest integer
}

// Full pipeline: EMA-filtered raw → voltage → resistance → lux.
float Sensor::readLuxEma(int pin, float alpha) {
    int   raw  = readEma(pin, alpha);
    float vout = rawToVoltage(raw);
    float rldr = rldrCalc(vout);
    return luxCalc(rldr);
}

// ── ADC helpers ──────────────────────────────────────────────────────────────

int Sensor::readAverage(int pin, int samples) {
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += analogRead(pin);
        delay(5);
    }
    return (int)(sum / samples);
}

float Sensor::rawToVoltage(int raw) {
    return VCC * ((float)raw / (ADC_RANGE - 1));
}

// ── LDR model ────────────────────────────────────────────────────────────────

float Sensor::rldrCalc(float vout) {
    if (vout <= VOUT_MIN) return MAX_RLDR;
    return RF * (VCC - vout) / vout;
}

float Sensor::luxCalc(float rldr) {
    float loglux = (log10(rldr) - B) / M;
    return pow(10.0f, loglux);
}

float Sensor::bCalc(float rldr) {
    return log10(rldr) - M * log10(500.0f);
}

// ── Calibration helper ────────────────────────────────────────────────────────

void Sensor::computeB() {
    int   raw  = readAverage(LDR_PIN, 8);
    float vout = rawToVoltage(raw);
    float rldr = rldrCalc(vout);
    float b    = bCalc(rldr);

    Serial.println("raw,vout,rldr,b");
    Serial.print(raw);       Serial.print(",");
    Serial.print(vout, 5);   Serial.print(",");
    Serial.print(rldr, 0);   Serial.print(",");
    Serial.println(b, 3);
}

// ── Convenience pipeline ─────────────────────────────────────────────────────

float Sensor::readLux(int pin) {
    int   raw  = analogRead(pin);
    float vout = rawToVoltage(raw);
    float rldr = rldrCalc(vout);
    return luxCalc(rldr);
}

float Sensor::readLuxAvg(int pin, int samples) {
    int   raw  = readAverage(pin, samples);
    float vout = rawToVoltage(raw);
    float rldr = rldrCalc(vout);
    return luxCalc(rldr);
}
