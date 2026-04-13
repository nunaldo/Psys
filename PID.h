#ifndef PID_H
#define PID_H

// ─────────────────────────────────────────────────────────────────────────────
//  PID controller with:
//    • Anti-windup  (back-calculation, gain kt)
//    • Bumpless transfer  (kp, b, ki, kd all corrected on gain switch)
//    • Derivative on output only  (no derivative kick on setpoint steps)
//    • Two operating modes: TRANSIENT / STEADY_STATE
// ─────────────────────────────────────────────────────────────────────────────

struct Gains {
    float kp;   // proportional gain
    float b;    // setpoint weight  (0 < b ≤ 1)
    float ki;   // integral gain
    float kd;   // derivative gain
    float N;    // derivative filter coefficient
};

class pid {
public:
    // ── Construction ────────────────────────────────────────────────────────
    pid(float h,
        const Gains& g,
        float kt   = 0.1f,      // anti-windup back-calculation gain
        float ulow = 0.0f,
        float uhigh= 4095.0f);

    // ── Main API ─────────────────────────────────────────────────────────────
    float compute_control(float r, float y);

    // Call before changing gains at runtime; performs bumpless correction
    void  update_gains(const Gains& new_gains);

    // Direct read-back
    const Gains& current_gains() const { return g_; }

    void reset_integrator() { i_ = 0.0f; }
    void preload_integrator(float value) { i_ = value; }
private:
    // Tuning (fixed)
    float h_;
    float kt_;
    float ulow_, uhigh_;

    // Active gains
    Gains g_;

    // Cached coefficients (recomputed on gain change)
    float bi_;              // ki * h
    float ad_, bd_;         // derivative filter
    float ao_;              // kt * h

    // State
    float i_;               // integrator state
    float d_;               // derivative state
    float y_old_;           // previous process output

    // Shadow gains for bumpless correction
    Gains g_old_;

    // Helpers
    void  recompute_coeffs();
    static float sat(float v, float lo, float hi);
};

#endif /* PID_H */