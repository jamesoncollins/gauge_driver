#pragma once
#include <cstdint>
#include <cmath>

/**
 * HzSensorFilter
 *  - ISR feeds edges/captures
 *  - Converts to Hz, enforces physical limits, smooths, maps to units
 *  - Lock-free snapshots for main thread
 *
 * Key features:
 *  - Max-value guard: reject edges implying > max_value (with margin)
 *  - Max acceleration: clamp or reject steps beyond units/s
 *  - Auto "stale" from min_value_care: user tells the smallest useful value
 *    (e.g., 5 mph, 400 rpm) and we derive a timeout from its period
 *  - Works great with a 32-bit free-running timer (ARR=0xFFFFFFFF). Also
 *    supports 16-bit + overflow accounting via isrOnTimerUpdate().
 *
 * Usage:
 *   g_x.init(cfg, micros_fn);
 *   // in IC ISR:
 *   g_x.isrOnCaptureCCR(TIMx->CCRy);
 *   // (optional) in Update ISR if timer is not 32-bit free-running:
 *   g_x.isrOnTimerUpdate(ARR+1);
 *   // in main/TIM16 @ 2 kHz:
 *   auto s = g_x.readSnapshot(); value = s.stale ? 0 : s.value;
 */
class HzSensorFilter {
public:
    struct Config {
        // --- Timer / capture ---
        uint32_t timer_freq_hz = 1;      // ticks/second (e.g., 1'000'000)
        uint32_t timer_max_tick = 0xFFFF; // ARR value (e.g., 0xFFFFFFFF for 32-bit free-run)

        // --- Mapping: value = K*Hz + B (your "real" units) ---
        float K = 1.0f;
        float B = 0.0f;
        float idle_value = 0.0f;

        // --- Filtering / outliers ---
        float alpha_normal  = 0.25f;
        float alpha_outlier = 0.05f;
        float outlier_ratio = 0.50f;
        float min_hz = 0.0f;
        float max_hz = 2000.0f;        // safety guard rail
        uint32_t stale_timeout_us = 250000; // fallback if care-threshold not set
        uint8_t pulses_per_event = 1;  // (available for callers; not used internally)

        // --- Physical constraints ---
        // 1) Max expected value. If K>0, we derive a max Hz and drop faster edges.
        float max_value = INFINITY;     // e.g., 180 mph or 9000 rpm
        float guard_margin_ratio = 0.05f; // +5% slack before reject

        // 2) Max acceleration (units/s). Clamp or reject.
        float max_accel_up_value_per_s   = INFINITY; // e.g., 25 mph/s, 9000 rpm/s
        float max_accel_down_value_per_s = INFINITY; // separate decel cap
        bool  accel_clamp = true; // true: clamp to reachable; false: reject sample

        // --- Visibility / auto-stale from "minimum value of interest" ---
        // If you set min_value_care>0, we compute a derived timeout based on its period.
        // Alternatively, set min_hz_care_override to specify directly in Hz.
        float min_value_care = 0.0f;          // e.g., 5 mph, 400 rpm
        float min_hz_care_override = NAN;     // if set (finite), overrides computation from value
        float care_timeout_margin = 1.30f;    // multiplier on that period (adds slack)

        // --- Consecutive-agreement gate (pre-EMA, in ISR) ---
        uint8_t consec_required = 2;         // X: need this many in a row
        float   consec_pct_between = 0.20f;  // Y1: spread among the X must be <= 20%
        float   consec_pct_vs_state = 1.0; // Y2: each tick within 30% of current EMA (ignored if EMA≈0)
        uint32_t consec_max_gap_us = 1000000; // reset the chain if no edge for this long
    };

    struct Snapshot {
        float hz;
        float value;
        float hz_raw;
        uint32_t last_edge_us;
        bool stale;     // true if beyond derived timeout (no recent edges)
        uint8_t quality;
    };

    HzSensorFilter() = default;

    void init(const Config& cfg, uint32_t (*micros_fn)()) {
        cfg_ = cfg;
        micros_ = micros_fn;

        hz_ema_ = 0.0f;
        hz_last_raw_ = 0.0f;
        value_ = cfg_.idle_value;

        last_edge_us_ = micros_ ? micros_() : 0u;
        last_filt_us_ = last_edge_us_;
        last_value_filt_ = value_;

        have_prev_ = false;
        pending_overflow_ticks_ = 0;
        last_ccr_ = 0;
        last_ticks_ = 0;

        consec_count_ = 0;
        consec_min_hz_ = 0.f;
        consec_max_hz_ = 0.f;
        consec_start_us_ = last_edge_us_;

        seq_.store(0u);
        recomputeCareThreshold_();
    }

    // ---------------------------------------------------------------------
    // ISR hooks
    // ---------------------------------------------------------------------

    // Call from Timer UPDATE ISR if the counter is not 32-bit free-running.
    // Pass per_overflow_ticks = ARR+1 of the capture timer.
    inline void isrOnTimerUpdate(uint32_t per_overflow_ticks) {
        pending_overflow_ticks_ += per_overflow_ticks; // 32-bit op is atomic on M4F
    }

    // Call from Timer IC ISR with CCR value (capture latch).
    inline void isrOnCaptureCCR(uint32_t ccr) {
        // If it's our first edge, just latch and wait for the next one.
        if (!have_prev_) {
            last_ccr_ = ccr;
            have_prev_ = true;
            pending_overflow_ticks_ = 0;
            return;
        }

        // Delta in ticks (unsigned subtraction wraps naturally for any width).
        uint32_t dt_ticks = ccr - last_ccr_;

        // Add any full wraps (only nonzero if you also call isrOnTimerUpdate()).
        dt_ticks += pending_overflow_ticks_;
        pending_overflow_ticks_ = 0;
        last_ccr_ = ccr;

        if (dt_ticks == 0) return;
        float hz = (float)cfg_.timer_freq_hz / (float)dt_ticks;
        isrCommonUpdate_(hz);
    }

    // Compatibility helpers (not needed in your current wiring)
    inline void isrUpdateFromPeriod(uint32_t period_ticks) {
        if (period_ticks == 0) return;
        float hz = (float)cfg_.timer_freq_hz / (float)period_ticks;
        isrCommonUpdate_(hz);
    }
    inline void isrUpdateFromEdge(uint32_t now_ticks) {
        // Wrap-safe for arbitrary ARR (inclusive)
        uint32_t arr = cfg_.timer_max_tick;
        uint32_t dt_ticks = (now_ticks >= last_ticks_)
                          ? (now_ticks - last_ticks_)
                          : ((arr - last_ticks_) + 1u + now_ticks);
        last_ticks_ = now_ticks;
        if (dt_ticks == 0) return;
        float hz = (float)cfg_.timer_freq_hz / (float)dt_ticks;
        isrCommonUpdate_(hz);
    }

    // ---------------------------------------------------------------------
    // Main loop API
    // ---------------------------------------------------------------------

    Snapshot readSnapshot() const {
        Snapshot s{};
        for (int attempt = 0; attempt < 4; ++attempt) {
            uint32_t s1 = seq_.load();

            float hz_ema = hz_ema_;
            float hz_raw = hz_last_raw_;
            uint32_t last_edge = last_edge_us_;
            float val = value_;

            uint32_t s2 = seq_.load();
            if ((s1 == s2) && ((s1 & 1u) == 0u)) {
                uint32_t now = micros_ ? micros_() : last_edge;
                uint32_t age = now - last_edge;

                // Use derived timeout (falls back to cfg_.stale_timeout_us if no care-threshold)
                uint32_t to_us = derived_stale_timeout_us_;
                bool stale = (age > to_us);

                // Even if not stale, clamp reads below minimum care Hz to idle
                bool subthreshold = (!stale && (hz_ema < hz_min_care_));

                s.hz = (stale || subthreshold) ? 0.0f : hz_ema;
                s.value = (stale || subthreshold) ? cfg_.idle_value : val;
                s.hz_raw = hz_raw;
                s.last_edge_us = last_edge;
                s.stale = stale;
                s.quality = computeQuality_(age, hz_ema, hz_raw);
                return s;
            }
        }
        // Rare fallback if writer interleaved updates repeatedly
        s.hz = hz_ema_;
        s.value = value_;
        s.hz_raw = hz_last_raw_;
        s.last_edge_us = last_edge_us_;
        s.stale = false;
        s.quality = 50;
        return s;
    }

    // Optional: gently decay when stale (call from a slow tick)
    void serviceDecay() {
        uint32_t now = micros_ ? micros_() : last_edge_us_;
        uint32_t age = now - last_edge_us_;
        uint32_t to_us = derived_stale_timeout_us_;
        if (age <= to_us) return;

        beginWrite_();
        hz_ema_ *= 0.5f; // simple damping; tune as desired
        if (hz_ema_ < cfg_.min_hz) hz_ema_ = 0.0f;
        value_ = cfg_.K * hz_ema_ + cfg_.B;
        last_filt_us_ = now;
        last_value_filt_ = value_;
        endWrite_();
    }

    // Update mapping at runtime (e.g., calibration) and recompute care threshold
    inline void setMap(float K, float B) {
        beginWrite_();
        cfg_.K = K; cfg_.B = B;
        value_ = cfg_.K * hz_ema_ + cfg_.B;
        endWrite_();
        recomputeCareThreshold_();
    }

    // Set minimum value of interest (units) and optional new margin
    inline void setMinValueCare(float v, float margin = -1.f) {
        beginWrite_();
        cfg_.min_value_care = v;
        if (margin > 0.f) cfg_.care_timeout_margin = margin;
        endWrite_();
        recomputeCareThreshold_();
    }

    inline float lastRawHz() const { return hz_last_raw_; }

private:
    // --- Lock-free write bracketing (32-bit seq counter) ---
    inline void beginWrite_() { seq_.fetch_add(1u); } // odd during write
    inline void endWrite_()   { seq_.fetch_add(1u); } // back to even

    // --- Quality heuristic (simple freshness + outlier-ness) ---
    inline uint8_t computeQuality_(uint32_t age_us, float hz, float hz_raw) const {
        float age_ms = age_us * 0.001f;
        float age_score = (age_ms < 50.f) ? 100.f : (age_ms < 250.f ? 70.f : 30.f);
        float denom = (hz > hz_raw) ? hz : hz_raw;
        float dev = denom > 1e-6f ? std::fabs(hz - hz_raw) / denom : 0.0f;
        float outlier_score = (dev < cfg_.outlier_ratio) ? 100.f : 60.f;
        float q = 0.6f*age_score + 0.4f*outlier_score;
        if (q < 0.f) q = 0.f;
        if (q > 100.f) q = 100.f;
        return (uint8_t)q;
    }

    // --- Derived thresholds from config ---
    inline void recomputeCareThreshold_() {
        // minimum Hz we actually care to display
        if (std::isfinite(cfg_.min_hz_care_override)) {
            hz_min_care_ = (cfg_.min_hz_care_override > 0.f) ? cfg_.min_hz_care_override : 0.0f;
        } else if (cfg_.K > 0.f && cfg_.min_value_care > 0.f) {
            float hzv = (cfg_.min_value_care - cfg_.B) / cfg_.K;
            hz_min_care_ = (hzv > 0.f) ? hzv : 0.0f;
        } else {
            hz_min_care_ = 0.0f;
        }

        // derive timeout from period at hz_min_care_ with margin; else fall back
        if (hz_min_care_ > 0.f && std::isfinite(hz_min_care_)) {
            float period_s = 1.0f / hz_min_care_;
            float margin   = (cfg_.care_timeout_margin > 0.f) ? cfg_.care_timeout_margin : 1.0f;
            uint32_t t = (uint32_t)(period_s * margin * 1e6f);
            derived_stale_timeout_us_ = (t > 0u) ? t : cfg_.stale_timeout_us;
        } else {
            derived_stale_timeout_us_ = cfg_.stale_timeout_us; // explicit fallback
        }
    }

    inline float maxHzAllowed_() const {
        float h = cfg_.max_hz;
        if (cfg_.K > 0.f && std::isfinite(cfg_.max_value)) {
            float hv = (cfg_.max_value - cfg_.B) / cfg_.K;
            if (hv < h) h = hv;
        }
        if (h < 0.f) h = 0.f;
        return h;
    }

    inline bool accelLimitsActive_() const {
        return std::isfinite(cfg_.max_accel_up_value_per_s) ||
               std::isfinite(cfg_.max_accel_down_value_per_s);
    }

    inline void isrCommonUpdate_(float hz_inst) {
        uint32_t now_us = micros_ ? micros_() : last_edge_us_;

        // Reject impossible max-value samples
        float hmax = maxHzAllowed_();
        if (std::isfinite(hmax)) {
            float hguard = hmax * (1.f + cfg_.guard_margin_ratio);
            if (hz_inst > hguard) {
                return; // drop sample
            }
        }

        // ----- Consecutive-agreement gate -----
        {
            // Reset chain if too much time has elapsed between samples
            if ((now_us - consec_start_us_) > cfg_.consec_max_gap_us) {
                consec_count_ = 0;
                consec_min_hz_ = consec_max_hz_ = 0.f;
            }

            // Start or extend the chain
            if (consec_count_ == 0) {
                consec_min_hz_ = consec_max_hz_ = hz_inst;
                consec_start_us_ = now_us;
                consec_count_ = 1;
            } else {
                // Update spread
                if (hz_inst < consec_min_hz_) consec_min_hz_ = hz_inst;
                if (hz_inst > consec_max_hz_) consec_max_hz_ = hz_inst;

                float denom = (consec_max_hz_ > 1e-6f) ? consec_max_hz_ : 1.0f;
                float spread = (consec_max_hz_ - consec_min_hz_) / denom;

                // Check vs current filtered state (relaxed if EMA ~ 0)
                bool vs_state_ok = true;
                if (hz_ema_ > 1e-3f && std::isfinite(cfg_.consec_pct_vs_state)) {
                    float dden = (hz_inst > hz_ema_) ? hz_inst : hz_ema_;
                    float dev  = (dden > 1e-6f) ? std::fabs(hz_inst - hz_ema_) / dden : 0.f;
                    vs_state_ok = (dev <= cfg_.consec_pct_vs_state);
                }

                if ((spread <= cfg_.consec_pct_between) && vs_state_ok) {
                    if (consec_count_ < 0xFF) ++consec_count_;
                } else {
                    // Reset chain starting from this sample
                    consec_min_hz_ = consec_max_hz_ = hz_inst;
                    consec_start_us_ = now_us;
                    consec_count_ = 1;
                }
            }

            // If not enough consecutive agreement yet, just update last_raw/last_edge and bail.
            if (consec_count_ < cfg_.consec_required) {
                // Keep last_raw for diagnostics, but do NOT touch EMA/value
                hz_last_raw_ = hz_inst;
                last_edge_us_ = now_us;
                return;
            }

            // Optional: collapse to cluster center to reduce residual jitter
            // (You can just use hz_inst; using mid-spread is slightly calmer.)
            float hz_cluster = 0.5f * (consec_min_hz_ + consec_max_hz_);

            // Make it easy to keep accepting the stream without re-waiting N every time:
            // keep the chain "warm" at (required-1) so one bad tick won’t fully flush it.
            if (cfg_.consec_required > 0) {
                consec_count_ = (uint8_t)(cfg_.consec_required - 1);
                consec_min_hz_ = hz_cluster;
                consec_max_hz_ = hz_cluster;
            }

            // Replace hz_inst with the cluster value for the rest of the pipeline
            hz_inst = hz_cluster;
        }
        // ----- end consecutive-agreement gate -----


        // Clamp raw Hz to guards
        if (hz_inst < 0.f) hz_inst = 0.f;
        if (hz_inst > cfg_.max_hz) hz_inst = cfg_.max_hz;

        // Value for accel limits
        float value_inst = cfg_.K * hz_inst + cfg_.B;

        // dt since last filtered update (for accel limit math)
        float dt_s = 0.f;
        if (now_us != last_filt_us_) {
            uint32_t dtu = now_us - last_filt_us_;
            dt_s = (float)dtu * 1e-6f;
        }

        if (dt_s > 0.f && accelLimitsActive_() && std::fabs(cfg_.K) > 1e-12f) {
            float up_cap   = std::isfinite(cfg_.max_accel_up_value_per_s)
                           ? (last_value_filt_ + cfg_.max_accel_up_value_per_s * dt_s)
                           : INFINITY;
            float down_cap = std::isfinite(cfg_.max_accel_down_value_per_s)
                           ? (last_value_filt_ - cfg_.max_accel_down_value_per_s * dt_s)
                           : -INFINITY;

            float clamped = value_inst;
            if (clamped > up_cap)   clamped = up_cap;
            if (clamped < down_cap) clamped = down_cap;

            if (!cfg_.accel_clamp) {
                if (clamped != value_inst) return; // reject impossible jump
            } else {
                // Clamp then map back to Hz
                value_inst = clamped;
                hz_inst = (value_inst - cfg_.B) / cfg_.K;
                if (hz_inst < 0.f) hz_inst = 0.f;
                if (hz_inst > cfg_.max_hz) hz_inst = cfg_.max_hz;
            }
        }

        beginWrite_();

        hz_last_raw_ = hz_inst;
        last_edge_us_ = now_us;

        // EMA with outlier-aware alpha
        float denom = (hz_inst > hz_ema_) ? hz_inst : hz_ema_;
        float rel = (denom > 1e-6f) ? std::fabs(hz_inst - hz_ema_) / denom : 0.f;
        float a = (rel > cfg_.outlier_ratio) ? cfg_.alpha_outlier : cfg_.alpha_normal;

        hz_ema_ = a * hz_inst + (1.f - a) * hz_ema_;
        if (hz_ema_ < cfg_.min_hz) hz_ema_ = 0.f;
        if (hz_ema_ > cfg_.max_hz) hz_ema_ = cfg_.max_hz;

        value_ = cfg_.K * hz_ema_ + cfg_.B;

        // Track filtered timestamp & value for next accel step
        last_filt_us_ = now_us;
        last_value_filt_ = value_;

        endWrite_();
    }

    // --- Config & time source ---
    Config cfg_{};
    uint32_t (*micros_)() = nullptr;

    // --- Derived care threshold/timeout ---
    float    hz_min_care_ = 0.0f;
    uint32_t derived_stale_timeout_us_ = 250000;

    // --- Edge bookkeeping ---
    volatile uint32_t last_ccr_{0};
    volatile uint32_t pending_overflow_ticks_{0};
    volatile bool     have_prev_{false};

    // --- Shared state (32-bit each) ---
    volatile uint32_t last_edge_us_{0};
    volatile uint32_t last_filt_us_{0};
    volatile uint32_t last_ticks_{0}; // used by isrUpdateFromEdge()
    volatile float hz_last_raw_{0.0f};
    volatile float hz_ema_{0.0f};
    volatile float value_{0.0f};
    volatile float last_value_filt_{0.0f};

    // --- Consecutive-agreement state ---
    volatile uint8_t  consec_count_{0};
    volatile float    consec_min_hz_{0.f};
    volatile float    consec_max_hz_{0.f};
    volatile uint32_t consec_start_us_{0};


    // --- Lightweight 32-bit atomic seq counter ---
    struct Seq32 {
        volatile uint32_t v{0};
        inline uint32_t load() const { return v; }
        inline void store(uint32_t x){ v = x; }
        inline uint32_t fetch_add(uint32_t x){ uint32_t old=v; v=old+x; return old; }
    } seq_;
};
