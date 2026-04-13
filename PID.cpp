#include "PID.h"
#include <cmath>    // fabsf

pid::pid(float h, const Gains& g, float kt, float ulow, float uhigh)
    : h_{h}, kt_{kt}, ulow_{ulow}, uhigh_{uhigh},
      g_{g}, g_old_{g},
      i_{0.0f}, d_{0.0f}, y_old_{0.0f}
{
    recompute_coeffs();
}

void pid::update_gains(const Gains& ng) {
    g_old_ = g_;    
    g_     = ng;
    recompute_coeffs();
}

float pid::compute_control(float r, float y) {
    i_ += g_old_.kp*(g_old_.b*r - y) - g_.kp*(g_.b*r - y);
    g_old_ = g_;

    float p = g_.kp * (g_.b * r - y);

    d_ = ad_ * d_ - bd_ * (y - y_old_);

    float v = p + i_ + d_;

    float u = sat(v, ulow_, uhigh_);

    // ── Conditional integration (anti-windup for natural zero/saturation) ──
    float di = bi_ * (r - y) + ao_ * (u - v);
    bool at_low  = (u <= ulow_);
    bool at_high = (u >= uhigh_);
    if (!(at_low  && di < 0.0f) &&
        !(at_high && di > 0.0f)) {
        i_ += di;
    }

    // ── Only back-calculation anti-windup ────────────────────────────────────
    // i_ += bi_ * (r - y) + ao_ * (u - v);

    y_old_ = y;

    return u;
}

void pid::recompute_coeffs() {
    bi_ = g_.ki * h_;
    ad_ = g_.kd / (g_.kd + g_.N * h_);
    bd_ = g_.N  * g_.kd / (g_.kd + g_.N * h_);
    ao_ = kt_   * h_;
}

float pid::sat(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}