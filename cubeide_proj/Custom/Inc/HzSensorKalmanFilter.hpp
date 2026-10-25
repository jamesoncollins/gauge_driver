#pragma once
#include <cstdint>
#include <cmath>
#include <limits>

// Protect queue operations only; never run Kalman matrix work with IRQs masked.
#if defined(__GNUC__) && defined(__arm__)
#include "stm32wbxx.h"
class HzQueueGuard {
    uint32_t primask_;
public:
    HzQueueGuard() : primask_(__get_PRIMASK()) { __disable_irq(); __DMB(); }
    ~HzQueueGuard() { __DMB(); __set_PRIMASK(primask_); }
};
#else
#include <mutex>
inline std::mutex &hz_queue_mutex() { static std::mutex m; return m; }
class HzQueueGuard {
    std::lock_guard<std::mutex> lock_;
public:
    HzQueueGuard() : lock_(hz_queue_mutex()) {}
};
#endif
#define KF_LOCK() HzQueueGuard kf_lock_guard

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
        // Optional sensor input ceiling, independent of the reported-unit cap.
        // Finite positive values qualify edges before they enter the queue.
        float max_input_hz = std::numeric_limits<float>::infinity();
        uint32_t timestamp_resolution_us = 1;
        bool median_periods = false; // Three qualified periods; reject isolated missed-edge intervals.
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
        uint32_t measurement_age_us; // age incorporated in estimate
        uint32_t tick_age_us;        // age of received edge
        uint32_t dropped_measurements;
        uint32_t rejected_ticks;     // short edges and cancelled candidates
        uint32_t raw_tick_age_us;    // includes noise, unlike qualified tick age
        uint32_t last_raw_tick_us;
        uint32_t last_raw_period_ticks;
        uint32_t last_period_ticks;
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

        count_ = 0; dropped_ = 0; have_tick_ = false;
        rejected_ticks_=0; have_raw_tick_=false;
        last_raw_tick_us_=last_raw_period_ticks_=0;
        pending_valid_=false; pending_={}; pending_span_ticks_=span_ticks_=0;
        min_period_ticks_=settle_us_=0;
        period_count_=period_index_=0;
        if (cfg_.clock_hz && std::isfinite(cfg_.max_input_hz) && cfg_.max_input_hz>0) {
            const double minimum=std::ceil(double(cfg_.clock_hz)/cfg_.max_input_hz);
            if (minimum<=UINT32_MAX) {
                min_period_ticks_=uint32_t(minimum);
                // Conservative margin: quantized timestamps must not certify
                // a candidate before a complete minimum period has elapsed.
                const uint64_t duration=(uint64_t(min_period_ticks_)*1000000+cfg_.clock_hz-1)/cfg_.clock_hz;
                const uint64_t settle=duration+cfg_.timestamp_resolution_us;
                settle_us_=uint32_t(settle>UINT32_MAX?UINT32_MAX:settle);
            }
        }
        tick_sequence_ = consumed_sequence_ = 0; stale_latched_ = true;
        head_           = 0;
        tail_           = 0;
        last_dt_ticks_  = 0;

        for (int i = 0; i < MAX_MEAS_Q; ++i) {
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
        zero_timeout_us_=UINT32_MAX;
        const double timeout_us=std::ceil(double(zero_timeout_s_)*1000000.0);
        if(std::isfinite(timeout_us) && timeout_us<UINT32_MAX)
            zero_timeout_us_=uint32_t(timeout_us);
    }

    // Single producer (capture ISR), single consumer (main loop).
    // init is called before captures are enabled. Explicit-time APIs also work
    // without a clock callback, for deterministic simulation. dt_ticks must
    // include every adjacent raw capture interval, even across rejected noise;
    // the qualifier reconstructs elapsed ticks from its last confirmed edge.
    void tick(uint32_t dt_ticks) {
        if (micros_) tickAt(dt_ticks, micros_());
    }

    void tickAt(uint32_t dt_ticks, uint32_t t_us) {
        if (!dt_ticks) return;
        KF_LOCK();
        last_raw_tick_us_=t_us; last_raw_period_ticks_=dt_ticks; have_raw_tick_=true;
        if (!min_period_ticks_) {
            enqueue_(dt_ticks,t_us);
            return;
        }
        span_ticks_+=dt_ticks;
        if (dt_ticks<min_period_ticks_) {
            // The preceding candidate is part of the same impossible cluster.
            // Keep the elapsed capture ticks so the next clean edge is timed
            // from the last qualified edge, not from the last noise edge.
            rejected_ticks_+=pending_valid_?2u:1u;
            pending_valid_=false;
            return;
        }
        if (pending_valid_) confirm_pending_();
        pending_.dt_ticks=uint32_t(span_ticks_>UINT32_MAX?UINT32_MAX:span_ticks_);
        pending_.t_us=t_us;
        pending_span_ticks_=span_ticks_;
        pending_valid_=true;
    }

    uint32_t last_dt_ticks() const {
        KF_LOCK();
        return last_dt_ticks_;
    }

    Snapshot retrieveValue() { return retrieveValueImpl_(0, true); }
    Snapshot retrieveValueAt(uint32_t now_us) { return retrieveValueImpl_(now_us, false); }

private:
    Snapshot retrieveValueImpl_(uint32_t now_us, bool use_clock) {
        // Copy a bounded batch before releasing the queue. ISR can continue
        // publishing while we process matrices; no shared slot references escape.
        Meas batch[MAX_MEAS_Q];
        unsigned n = 0;
        Snapshot out{};
        bool have_tick;
        uint32_t sequence;
        {
            KF_LOCK();
            if (use_clock) now_us = micros_ ? micros_() : last_raw_tick_us_;
            if (pending_valid_ && int32_t(now_us-pending_.t_us)>=0 && now_us-pending_.t_us>=settle_us_)
                confirm_pending_();
            have_tick = have_tick_;
            sequence = tick_sequence_;
            out.last_tick_us = last_tick_us_;
            out.rejected_ticks=rejected_ticks_;
            out.last_raw_tick_us=last_raw_tick_us_;
            out.last_raw_period_ticks=last_raw_period_ticks_;
            out.raw_tick_age_us=have_raw_tick_?now_us-last_raw_tick_us_:UINT32_MAX;
            if(have_raw_tick_ && int32_t(now_us-last_raw_tick_us_)<0) out.raw_tick_age_us=0;
            out.last_period_ticks = last_dt_ticks_;
            out.dropped_measurements = dropped_;
            while (count_ && int32_t(meas_[tail_].t_us - now_us) <= 0) {
                batch[n++] = meas_[tail_];
                tail_ = (tail_ + 1u) % MAX_MEAS_Q;
                --count_;
            }
        }
        // Signed ordering assumes no scheduling gap exceeds half the clock range.
        out.tick_age_us = have_tick ? uint32_t(now_us - out.last_tick_us) : UINT32_MAX;
        const bool new_edge = sequence != consumed_sequence_;
        // An ISR may publish just after the caller samples now. Only a newly
        // received edge can be in that future; a long-stopped input stays stale.
        if (new_edge && int32_t(now_us-out.last_tick_us)<0) out.tick_age_us=0;
        if (new_edge) stale_latched_=false;
        if (!have_tick || out.tick_age_us*1e-6f > zero_timeout_s_) stale_latched_=true;
        consumed_sequence_=sequence;
        out.stale=stale_latched_;
        if (!initialized_ && !n) return out;
        if (!initialized_) {
            x_[1] = cfg_.units_per_hz * float(cfg_.clock_hz) / batch[0].dt_ticks + cfg_.units_bias;
            last_pred_us_ = last_meas_us_ = batch[0].t_us;
            initialized_ = true;
        }
        uint32_t t_prev = last_pred_us_;
        for (unsigned i = 0; i < n; ++i) {
            const auto &m = batch[i];
            const int32_t elapsed = int32_t(m.t_us - t_prev);
            if (elapsed > 0) predict_(elapsed * 1e-6f);
            // Incoming real data supersedes virtual-zero observations.
            update_(cfg_.units_per_hz * float(cfg_.clock_hz) / m.dt_ticks + cfg_.units_bias,
                    cfg_.meas_var);
            last_meas_us_ = m.t_us;
            if (elapsed >= 0) t_prev = m.t_us;
        }
        const int32_t elapsed = int32_t(now_us - t_prev);
        if (elapsed > 0) predict_(elapsed * 1e-6f);
        applyVirtualZero_(now_us);
        last_pred_us_ = now_us;
        out.units = x_[1];
        out.accel_units_s2 = x_[2];
        out.have_state = true;
        out.updated = n != 0;
        out.measurement_age_us = now_us - last_meas_us_;
        if (!std::isfinite(out.units) || !std::isfinite(out.accel_units_s2)) {
            // Reinitialize on the next usable measurement rather than propagating NaNs.
            initialized_ = false;
            for (unsigned i=0;i<3;++i) {
                x_[i]=0;
                for (unsigned j=0;j<3;++j) P_[i][j]=i==j ? (i==0 ? 1.0f : 50.0f) : 0.0f;
            }
            out.units = out.accel_units_s2 = 0; out.have_state = false;
        }
        if (out.units < 0) out.units = 0;
        if (out.units > cfg_.max_units) out.units = cfg_.max_units;
        return out;
    }

private:
    struct Meas {
        uint32_t t_us;
        uint32_t dt_ticks;

    };

    // Caller holds the queue guard. No matrix work or floating-point conversion
    // occurs on this capture-side path.
    void enqueue_(uint32_t dt_ticks,uint32_t t_us) {
        if (cfg_.median_periods) {
            // A real edge can be lost inside a rejected cluster. Median-of-three
            // prevents a single doubled period from becoming a false slowdown.
            // Retain the actual edge timestamp for freshness and model timing.
            if(have_tick_ && zero_timeout_us_!=UINT32_MAX &&
               uint32_t(t_us-last_tick_us_)>zero_timeout_us_) period_count_=period_index_=0;
            periods_[period_index_]=dt_ticks;
            period_index_=(period_index_+1)%3;
            if(period_count_<3) ++period_count_;
            if(period_count_==3) {
                uint32_t a=periods_[0],b=periods_[1],c=periods_[2];
                if(a>b){const auto t=a;a=b;b=t;}
                if(b>c){const auto t=b;b=c;c=t;}
                if(a>b)b=a;
                dt_ticks=b;
            }
        }
        ++tick_sequence_;
        last_tick_us_=t_us; last_dt_ticks_=dt_ticks; have_tick_=true;
        if (count_ == MAX_MEAS_Q) {
            tail_ = (tail_ + 1u) % MAX_MEAS_Q;
            --count_;
            ++dropped_; // discard oldest, retain the most recent periods
        }
        meas_[head_].t_us=t_us; meas_[head_].dt_ticks=dt_ticks;
        head_=(head_+1u)%MAX_MEAS_Q;
        ++count_;
    }
    void confirm_pending_() {
        enqueue_(pending_.dt_ticks,pending_.t_us);
        span_ticks_-=pending_span_ticks_;
        pending_valid_=false;
    }

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
        if (!initialized_) return;
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

    static_assert(MAX_MEAS_Q > 0 && MAX_MEAS_Q <= 255, "queue capacity must fit indices");
    uint32_t zero_timeout_us_=UINT32_MAX;
    uint32_t periods_[3]{};
    unsigned period_count_=0,period_index_=0;
    uint32_t min_period_ticks_=0,settle_us_=0,rejected_ticks_=0;
    uint32_t last_raw_tick_us_=0,last_raw_period_ticks_=0;
    bool have_raw_tick_=false,pending_valid_=false;
    uint64_t span_ticks_=0,pending_span_ticks_=0;
    Meas pending_{};
    unsigned count_ = 0;
    uint32_t dropped_ = 0;
    bool have_tick_ = false;
    uint32_t tick_sequence_=0, consumed_sequence_=0;
    bool stale_latched_=true;
    Meas            meas_[MAX_MEAS_Q]{};
    volatile uint8_t head_ = 0;        // ISR write index
    volatile uint8_t tail_ = 0;        // main read index

    volatile uint32_t last_dt_ticks_ = 0;
};
