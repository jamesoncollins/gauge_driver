#pragma once
#include <cstdint>
#include <cmath>

/*
 * HzSensorFilter
 * - ISR calls tick(dt_ticks) with the captured delta-ticks between edges.
 * - Getter does all work: staleness vs min-value, consecutive-agreement,
 *   average, units conversion, hard max, accel limit, and extrapolation.
 *
 * Notes:
 * - "Ticks" are the capture timer counts between edges.
 * - Hz = clock_hz / dt_ticks.
 * - Units = units_per_hz * Hz + units_bias.
 * - No EMA. No hidden margins. Everything explicit.
 */

template <int MAX_WIN=3>
class HzSensorFilter {
public:
    struct Config {
        // Conversion
        float units_per_hz = 1.0f;   // slope from Hz to user units (rpm, mph, etc.)
        float units_bias   = 0.0f;   // intercept in user units

        // Thresholds (in user units)
        float min_units = 0.0f;      // below this, report 0
        float max_units = INFINITY;  // hard ceiling

        // Acceleration limit (in user units / second)
        float max_accel_units_per_s = INFINITY; // applies both up and down

        // Timer properties
        uint32_t clock_hz = 1000000; // capture timer frequency (ticks/second)

        // Consecutive-agreement (pre-filter gate) over the ring buffer
        // Require that samples are mutually within common_pct (fraction of max in window).
        // Optionally also require agreement vs current state (set to NaN to ignore).
        //uint8_t  window_len = MAX_WIN;     // number of dt samples in the ring to check/average
        float    common_pct = 0.20f; // e.g., 0.20 means spread <= 20% of max in window
        float    common_vs_state_pct = NAN; // e.g., 0.30f; NaN disables this check

        // Max age logic: if no tick has arrived for longer than the period implied
        // by min_units, the getter outputs 0. (No extra "margin".)
    };

    struct Snapshot {
        float   units;           // filtered user units (or 0 below min/when stale)
        float   hz;              // average Hz used this update (0 if stale)
        bool    updated;         // true if a new ring-window passed the gate this call
        bool    stale;           // true if last tick age implies < min_units
        uint8_t used_samples;    // how many samples contributed (0 if stale)
        uint32_t last_tick_us;   // timestamp of last ISR tick
    };

    HzSensorFilter() = default;

    void init(const Config& cfg, uint32_t (*micros_fn)()) {
        cfg_ = cfg;
        micros_ = micros_fn;

        // Derived min Hz from min_units (if slope>0 and min_units>0)
        if (cfg_.units_per_hz > 0.0f && cfg_.min_units > 0.0f) {
            min_hz_ = (cfg_.min_units - cfg_.units_bias) / cfg_.units_per_hz;
            if (!std::isfinite(min_hz_) || min_hz_ < 0.0f) min_hz_ = 0.0f;
        } else {
            min_hz_ = 0.0f;
        }
        // Min period from min_hz
        min_period_us_ = (min_hz_ > 0.0f) ? static_cast<uint32_t>(1e6f / min_hz_) : 0u;

        // Reset state
        head_ = 0;
        count_ = 0;
        last_tick_us_ = micros_ ? micros_() : 0u;

        last_units_ = 0.0f;
        last_units_us_ = last_tick_us_;
        last_rate_units_per_s_ = 0.0f;
    }

    // --- ISR path: very short and predictable ---
    // dt_ticks = capture delta (CCR(n) - CCR(n-1) with wrap handling done by caller’s timer setup)
    inline void tick(uint32_t dt_ticks) {
        if (dt_ticks == 0) return; // ignore degenerate

        rb_[head_] = dt_ticks;
        head_ = (head_ + 1u) % MAX_WIN;
        if (count_ < MAX_WIN) ++count_;
        last_tick_us_ = micros_ ? micros_() : last_tick_us_;
    }

    // --- Getter: do all the work here ---
    Snapshot retrieveValue() {
        Snapshot out{};
        const uint32_t now_us = micros_ ? micros_() : last_tick_us_;
        const uint32_t age_us = now_us - last_tick_us_;

        // If we have a minimum displayable units, use its implied period to decide staleness.
        if (min_period_us_ > 0u && age_us > min_period_us_) {
            float units = 0.0f; // below min -> zero
            out = finalize_(units, 0.0f, /*updated=*/false, /*stale=*/true, /*used=*/0, last_tick_us_);
            // Do not change last_units_us_ here; keep extrapolation clocking from last update.
            return out;
        }

        // Gather the last `need` samples (most-recent-first)
        float hz_min = INFINITY, hz_max = 0.0f, hz_sum = 0.0f;
        for (uint8_t idx = 0; idx < MAX_WIN; idx++) {
            const uint32_t dt_ticks = rb_[idx];
            if (dt_ticks == 0)
            {
                out = finalize_(0, 0.0f, /*updated=*/false, /*stale=*/true, /*used=*/0, last_tick_us_);
                return out;
            }
            const float hz = static_cast<float>(cfg_.clock_hz) / static_cast<float>(dt_ticks);
            if (!std::isfinite(hz) || hz <= 0.0f)
            {
                out = finalize_(0, 0.0f, /*updated=*/false, /*stale=*/true, /*used=*/0, last_tick_us_);
                return out;
            }

            hz_min = (hz < hz_min) ? hz : hz_min;
            hz_max = (hz > hz_max) ? hz : hz_max;
            hz_sum += hz;
        }

//        if (used == 0) {
//            // No usable samples; extrapolate
//            float units = extrapolate_(now_us);
//            out = finalize_(units, 0.0f, /*updated=*/false, /*stale=*/false, /*used=*/0, last_tick_us_);
//            return out;
//        }

        // Consecutive-agreement: spread <= common_pct * max
        const float spread = (hz_max > 0.0f) ? ((hz_max - hz_min) / hz_max) : INFINITY;
        if (!(spread <= cfg_.common_pct)) {
            // Not consistent; hold/extrapolate
            float units = extrapolate_(now_us);
            out = finalize_(units, 0.0f, /*updated=*/false, /*stale=*/false, /*used=*/MAX_WIN, last_tick_us_);
            return out;
        }

        // Optional agreement vs current state (if configured)
        if (std::isfinite(cfg_.common_vs_state_pct) && last_units_us_ != 0u) {
            const float last_hz = (cfg_.units_per_hz > 0.0f)
                ? (last_units_ - cfg_.units_bias) / cfg_.units_per_hz
                : 0.0f;
            if (last_hz > 0.0f) {
                // Compare average hz vs last_hz
                const float hz_avg_tmp = hz_sum / static_cast<float>(MAX_WIN);
                const float denom = (hz_avg_tmp > last_hz) ? hz_avg_tmp : last_hz;
                const float dev = (denom > 0.0f) ? std::fabs(hz_avg_tmp - last_hz) / denom : 0.0f;
                if (dev > cfg_.common_vs_state_pct) {
                    float units = extrapolate_(now_us);
                    out = finalize_(units, 0.0f, /*updated=*/false, /*stale=*/false, /*used=*/MAX_WIN, last_tick_us_);
                    return out;
                }
            }
        }

        // Average accepted samples
        const float hz_avg = hz_sum / static_cast<float>(MAX_WIN);

        // Convert to user units
        float units = cfg_.units_per_hz * hz_avg + cfg_.units_bias;

        // Enforce min_units clamp (display intent)
        if (units < cfg_.min_units) {
            units = 0.0f; // below minimum: treat as zero
        }

        // Enforce max value
        if (std::isfinite(cfg_.max_units) && units > cfg_.max_units) {
            units = cfg_.max_units;
        }

        // Acceleration limiting vs last output
        if (std::isfinite(cfg_.max_accel_units_per_s) && last_units_us_ != 0u) {
            const float dt_s = (now_us - last_units_us_) * 1e-6f;
            if (dt_s > 0.0f) {
                const float max_delta = cfg_.max_accel_units_per_s * dt_s;
                const float lo = last_units_ - max_delta;
                const float hi = last_units_ + max_delta;
                if (units < lo) units = lo;
                if (units > hi) units = hi;
            }
        }

        out = finalize_(units, hz_avg, /*updated=*/true, /*stale=*/false, /*used=*/MAX_WIN, last_tick_us_);
        return out;
    }

private:
    Snapshot finalize_(float units, float hz_used, bool updated, bool stale,
                       uint8_t used_samples, uint32_t last_tick_us) {
        const uint32_t now_us = micros_ ? micros_() : last_units_us_;
        // Update rate model
        if (last_units_us_ != 0u) {
            const float dt_s = (now_us - last_units_us_) * 1e-6f;
            if (dt_s > 1e-9f) {
                last_rate_units_per_s_ = (units - last_units_) / dt_s;
            } else {
                last_rate_units_per_s_ = 0.0f;
            }
        }
        last_units_ = units;
        last_units_us_ = now_us;

        Snapshot s{};
        s.units = units;
        s.hz = hz_used;
        s.updated = updated;
        s.stale = stale;
        //s.used_samples = used_samples;
        s.last_tick_us = last_tick_us;
        return s;
    }

    float extrapolate_(uint32_t now_us) const {
        if (last_units_us_ == 0u) return 0.0f; // no history
        const float dt_s = (now_us - last_units_us_) * 1e-6f;
        if (dt_s <= 0.0f) return last_units_;
        // Constant-velocity extrapolation
        float u = last_units_ + last_rate_units_per_s_ * dt_s;

        // Respect min/max
        if (u < 0.0f) u = 0.0f;
        if (std::isfinite(cfg_.max_units) && u > cfg_.max_units) u = cfg_.max_units;

        return u;
    }

private:
    Config cfg_{};
    uint32_t (*micros_)() = nullptr;

    // Derived min period from min_units
    float    min_hz_ = 0.0f;
    uint32_t min_period_us_ = 0;

    // Ring buffer of dt_ticks (ISR writes)
    uint32_t rb_[MAX_WIN]{};
    uint8_t  head_ = 0;   // next write
    uint8_t  count_ = 0;  // number of valid entries (<= MAX_WIN)

    // ISR-updated timestamp of last tick
    volatile uint32_t last_tick_us_ = 0;

    // Output state for accel/extrapolation
    float    last_units_ = 0.0f;
    float    last_rate_units_per_s_ = 0.0f;
    uint32_t last_units_us_ = 0;
};
