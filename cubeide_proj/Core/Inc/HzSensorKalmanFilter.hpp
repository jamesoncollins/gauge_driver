#pragma once
#include <cstdint>
#include <cmath>
#include <limits>

#if defined(__GNUC__) && defined(__arm__)
#define KF_LOCK()
#else
#include <mutex>
static std::mutex g_kf_mutex;
#define KF_LOCK() std::lock_guard<std::mutex> kf_lock_guard(g_kf_mutex)

#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ======================= KF FILTER CLASS =======================
//
// Tick-side API (ISR context):
//   - tick(dt_ticks)
//
// Main-loop API:
//   - init(cfg, micros_fn)
//   - Snapshot s = retrieveValue();
//   - uint32_t last_dt_ticks() for debugging / plotting
//
// State model: [x; v; a] with constant-acceleration and white jerk.

template <int MAX_MEAS_Q = 16>
class HzSensorKalmanFilter {
public:
    struct Config {
        // ===================== Measurement mapping =====================
        //
        // ISR gives dt_ticks from a timer running at clock_hz.
        // Hz  = clock_hz / dt_ticks
        // units = units_per_hz * Hz + units_bias

        float units_per_hz = 1.0f;
        // Slope of Hz -> user units (mph, rpm, etc.)
        //   e.g. 3000GT speed VSS: ~0.855752625 mph/Hz
        //   e.g. RPM (rev/s):      60.0f units_per_hz

        float units_bias   = 0.0f;
        // Offset of Hz -> units. Usually 0.

        // ===================== Timer / tick source =====================

        uint32_t clock_hz  = 1'000'000;
        // Capture timer frequency in ticks/second.
        // Used only in:
        //   Hz = clock_hz / dt_ticks

        // ===================== Process model (Q) =====================
        //
        // State: [position; speed; acceleration] in "units".
        // Constant-acceleration model with white-jerk noise:
        //   jerk ~ white noise with density q_jerk
        //
        // Larger q_jerk  => faster response, less smooth.
        // Smaller q_jerk => smoother, more sluggish.

        float q_jerk       = 5.0f;  // (units^2 / s^5)

        // ===================== Measurement noise (R) =====================
        //
        // Tick-derived measurement gives a noisy speed estimate in "units".

        float meas_var     = 0.3f * 0.3f;
        // Nominal variance of a single speed measurement, in units^2.
        // Example: 0.3 mph std-dev -> meas_var = 0.09.

        // ===================== Robust gating =====================
        //
        // Huber-style gating:
        //   thr = gate_sigma * sqrt(S)
        //   if |y| > thr, inflate R (downweight measurement)

        float gate_sigma   = 3.0f;
        // Rough "N-sigma" threshold. 2–4 is typical.

        // ===================== Zero-speed detection / locking =====================
        //
        // When ticks stop and the speed is already small, we inject a
        // virtual v = 0 measurement after some timeout derived from:
        //   zero_speed_thresh_units
        //   zero_periods_without_tick

        float zero_speed_thresh_units   = 5.0f;
        // Below this |speed|, consider the wheel "effectively stopped"
        // for the purpose of the virtual-zero logic.

        float zero_periods_without_tick = 3.0f;
        // How many tick periods (at zero_speed_thresh_units) we wait
        // with no ticks before applying a virtual v=0 measurement.
        //
        // Internally, we compute:
        //   Hz_at_thresh ~ (zero_speed_thresh_units - units_bias) / units_per_hz
        //   T = 1 / Hz_at_thresh
        //   zero_timeout_s_ = zero_periods_without_tick * T

        // ===================== Physical shaping / bounds =====================

        float max_accel_units_per_s     = std::numeric_limits<float>::infinity();
        // If finite, clamps |accel| state x_[2] to this bound (units/s^2).

        float max_units                 = std::numeric_limits<float>::infinity();
        // Hard ceiling on reported speed. Output speed is clamped to [0, max_units].
    };

    struct Snapshot {
        float    units;              // estimated speed in user units
        float    accel_units_s2;     // estimated acceleration in units/s^2
        bool     have_state;         // true once initialized
        bool     updated;            // true if any real measurements processed this call
        bool     stale;              // true if we consider the measurement stream "stale"
        uint32_t last_tick_us;       // timestamp of last ISR tick (micros)
    };

    HzSensorKalmanFilter() = default;

    void init(const Config& cfg, uint32_t (*micros_fn)()) {
        KF_LOCK();

        cfg_    = cfg;
        micros_ = micros_fn;

        // State & covariance
        for (int i = 0; i < 3; ++i) {
            x_[i] = 0.0f;
            for (int j = 0; j < 3; ++j) {
                P_[i][j] = 0.0f;
            }
        }
        P_[0][0] = 1.0f;
        P_[1][1] = 50.0f;
        P_[2][2] = 50.0f;

        initialized_    = false;
        last_pred_us_   = 0;
        last_meas_us_   = 0;
        last_tick_us_   = 0;

        head_           = 0;
        tail_           = 0;
        last_dt_ticks_  = 0;

        for (int i = 0; i < MAX_MEAS_Q; ++i) {
            meas_[i].ready   = false;
            meas_[i].t_us    = 0;
            meas_[i].dt_ticks = 0;
        }

        // Derive zero_timeout_s_ from (zero_speed_thresh_units, zero_periods_without_tick)
        zero_timeout_s_ = std::numeric_limits<float>::infinity();

        if (cfg_.units_per_hz > 0.0f &&
            cfg_.zero_speed_thresh_units > 0.0f &&
            std::isfinite(cfg_.zero_periods_without_tick) &&
            cfg_.zero_periods_without_tick > 0.0f)
        {
            float hz_at_thresh =
                (cfg_.zero_speed_thresh_units - cfg_.units_bias) / cfg_.units_per_hz;

            if (hz_at_thresh > 1e-6f) {
                float period_s = 1.0f / hz_at_thresh;
                zero_timeout_s_ = cfg_.zero_periods_without_tick * period_s;
            }
        }
    }

    // ISR: dt_ticks is timer delta between edges
    inline void tick(uint32_t dt_ticks) {
        KF_LOCK();

        //if (!micros_) return;
        if (dt_ticks == 0) return;

        uint32_t t_us = micros_();
        last_tick_us_ = t_us;
        last_dt_ticks_ = dt_ticks;

        uint8_t head = head_;
        Meas &slot = meas_[head];

        // If this slot is still ready, main hasn't consumed it yet: buffer full.
        // Drop this new measurement rather than overwriting.
        if (slot.ready) {
            return;
        }

        // Write data first...
        slot.t_us     = t_us;
        slot.dt_ticks = dt_ticks;

        // ...then publish it.
        slot.ready    = true;

        head_ = static_cast<uint8_t>((head + 1u) % MAX_MEAS_Q);
    }

    // Helper: last dt_ticks captured (for plotting/debugging)
    uint32_t last_dt_ticks() const {
        return last_dt_ticks_;
    }

    Snapshot retrieveValue() {
        KF_LOCK();

        Snapshot out{};

        const uint32_t now_us = micros_();
        bool any_update       = false;

        uint8_t tail = tail_;
        Meas &oldest = meas_[tail];
        bool have_meas = oldest.ready;

        // No state and no pending measurements
        if (!initialized_ && !have_meas) {
            out.units          = 0.0f;
            out.accel_units_s2 = 0.0f;
            out.have_state     = false;
            out.updated        = false;
            out.stale          = true;
            out.last_tick_us   = last_tick_us_;
            return out;
        }

        // Initialize on first measurement (peek, don't consume yet)
        if (!initialized_ && have_meas) {
            const Meas &m0 = oldest;
            float hz0 = 0.0f;
            if (m0.dt_ticks > 0) {
                hz0 = static_cast<float>(cfg_.clock_hz) /
                      static_cast<float>(m0.dt_ticks);
            }
            const float units0 = cfg_.units_per_hz * hz0 + cfg_.units_bias;

            x_[0] = 0.0f;
            x_[1] = units0;
            x_[2] = 0.0f;

            last_pred_us_ = m0.t_us;
            last_meas_us_ = m0.t_us;
            initialized_  = true;
        }

        uint32_t t_prev = last_pred_us_;

        // Consume all ready slots in order: tail -> head-1 (contiguous ready region)
        while (true) {
            Meas &slot = meas_[tail];
            if (!slot.ready) {
                break; // no more unread data
            }

            const uint32_t t_meas_us = slot.t_us;
            if (t_meas_us > now_us) {
                // Measurement from the future relative to 'now_us':
                // leave it for the next call.
                break;
            }

            float dt_s = (t_meas_us - t_prev) * 1e-6f;
            if (dt_s > 0.0f) {
                predict_(dt_s);
                applyVirtualZero_(t_meas_us);
            }

            float hz = 0.0f;
            if (slot.dt_ticks > 0) {
                hz = static_cast<float>(cfg_.clock_hz) /
                     static_cast<float>(slot.dt_ticks);
            }

            const float z_units = cfg_.units_per_hz * hz + cfg_.units_bias;
            update_(z_units, cfg_.meas_var);
            any_update    = true;
            last_meas_us_ = t_meas_us;
            t_prev        = t_meas_us;

            // Mark consumed and advance tail
            slot.ready = false;
            tail = static_cast<uint8_t>((tail + 1u) % MAX_MEAS_Q);
        }

        // Store updated tail index
        tail_ = tail;

        // Predict up to now
        float dt_s_end = (now_us - t_prev) * 1e-6f;
        if (dt_s_end > 0.0f) {
            predict_(dt_s_end);
            applyVirtualZero_(now_us);
        }

        last_pred_us_ = now_us;

        out.units          = x_[1];
        out.accel_units_s2 = x_[2];
        out.have_state     = true;
        out.updated        = any_update;

        // Staleness based on derived timeout
        if (last_meas_us_ == 0) {
            out.stale = true;
        } else if (!std::isfinite(zero_timeout_s_)) {
            out.stale = false;
        } else {
            float age_s = (now_us - last_meas_us_) * 1e-6f;
            out.stale = (age_s > zero_timeout_s_);
        }

        // Enforce non-negative speed and max_units
        if (out.units < 0.0f) out.units = 0.0f;
        if (std::isfinite(cfg_.max_units) && out.units > cfg_.max_units) {
            out.units = cfg_.max_units;
        }

        out.last_tick_us = last_tick_us_;
        return out;
    }

private:
    struct Meas {
        uint32_t t_us;
        uint32_t dt_ticks;
        volatile bool ready;
    };

    void clampAccel_() {
        if (!std::isfinite(cfg_.max_accel_units_per_s)) return;
        float a_max = cfg_.max_accel_units_per_s;
        if (x_[2] >  a_max) x_[2] =  a_max;
        if (x_[2] < -a_max) x_[2] = -a_max;
    }

    void predict_(float dt) {
        const float dt2  = dt * dt;
        const float dt3  = dt2 * dt;
        const float dt4  = dt3 * dt;
        const float dt5  = dt4 * dt;

        float F[3][3] = {
            { 1.0f, dt,  0.5f * dt2 },
            { 0.0f, 1.0f, dt        },
            { 0.0f, 0.0f, 1.0f      }
        };

        const float q = cfg_.q_jerk;
        float Q[3][3] = {
            { q * dt5 / 20.0f, q * dt4 / 8.0f,  q * dt3 / 6.0f },
            { q * dt4 / 8.0f,  q * dt3 / 3.0f,  q * dt2 / 2.0f },
            { q * dt3 / 6.0f,  q * dt2 / 2.0f,  q * dt         }
        };

        float x_new[3];
        for (int i = 0; i < 3; ++i) {
            x_new[i] = F[i][0] * x_[0] + F[i][1] * x_[1] + F[i][2] * x_[2];
        }

        float FP[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                FP[i][j] = F[i][0] * P_[0][j] +
                           F[i][1] * P_[1][j] +
                           F[i][2] * P_[2][j];
            }
        }

        float P_new[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                P_new[i][j] = FP[i][0] * F[j][0] +
                              FP[i][1] * F[j][1] +
                              FP[i][2] * F[j][2] +
                              Q[i][j];
            }
        }

        for (int i = 0; i < 3; ++i) {
            x_[i] = x_new[i];
            for (int j = 0; j < 3; ++j) {
                P_[i][j] = P_new[i][j];
            }
        }

        clampAccel_();
    }

    void update_(float z_units, float R) {
        const float hT[3] = { 0.0f, 1.0f, 0.0f };

        float z_pred = x_[1];
        float y      = z_units - z_pred;

        float S = P_[1][1] + R;
        if (S < 1e-12f) S = 1e-12f;

        float thr = cfg_.gate_sigma * std::sqrt(S);
        float w   = 1.0f;
        if (std::fabs(y) > thr && thr > 0.0f) {
            w = thr / std::fabs(y);
        }
        float R_eff = R / (w * w + 1e-12f);

        S = P_[1][1] + R_eff;
        if (S < 1e-12f) S = 1e-12f;

        float K[3];
        K[0] = P_[0][1] / S;
        K[1] = P_[1][1] / S;
        K[2] = P_[2][1] / S;

        for (int i = 0; i < 3; ++i) {
            x_[i] += K[i] * y;
        }

        float I_KH[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                float kh_ij = K[i] * hT[j];
                I_KH[i][j] = (i == j ? 1.0f : 0.0f) - kh_ij;
            }
        }

        float M[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                M[i][j] = I_KH[i][0] * P_[0][j] +
                          I_KH[i][1] * P_[1][j] +
                          I_KH[i][2] * P_[2][j];
            }
        }

        float P_new[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                float v = M[i][0] * I_KH[j][0] +
                          M[i][1] * I_KH[j][1] +
                          M[i][2] * I_KH[j][2];
                v += K[i] * K[j] * R_eff;
                P_new[i][j] = v;
            }
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                P_[i][j] = P_new[i][j];
            }
        }

        clampAccel_();
    }

    void applyVirtualZero_(uint32_t t_us_now) {
        if (last_meas_us_ == 0) return;
        if (!std::isfinite(zero_timeout_s_)) return;

        float dt_since_meas = (t_us_now - last_meas_us_) * 1e-6f;

        if (dt_since_meas > zero_timeout_s_ &&
            std::fabs(x_[1]) < cfg_.zero_speed_thresh_units &&
            std::isfinite(cfg_.meas_var))
        {
            update_(0.0f, cfg_.meas_var);
        }
    }

private:
    Config   cfg_{};
    uint32_t (*micros_)() = nullptr;

    float    x_[3]{};
    float    P_[3][3]{};

    bool     initialized_   = false;
    uint32_t last_pred_us_  = 0;
    uint32_t last_meas_us_  = 0;
    uint32_t last_tick_us_  = 0;

    float    zero_timeout_s_ = std::numeric_limits<float>::infinity();

    Meas            meas_[MAX_MEAS_Q]{};
    volatile uint8_t head_ = 0;        // ISR write index
    volatile uint8_t tail_ = 0;        // main read index

    volatile uint32_t last_dt_ticks_ = 0;
};
