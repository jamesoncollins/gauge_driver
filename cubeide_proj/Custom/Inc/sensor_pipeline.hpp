#pragma once
#include "HzSensorKalmanFilter.hpp"
#include "build_config.hpp"

// All units/calibration, capture arithmetic, filtering and needle targets live
// here so the simulator cannot silently bypass the production signal path.
namespace GaugeSensors {
enum Channel : unsigned { Speed = 0, Tach = 1 };
inline float bounded(float v, float maximum) {
    return !std::isfinite(v) || v < 0 ? 0 : (v > maximum ? maximum : v);
}
inline const GaugeChannelConfig &gauge_config(const VehicleConfig &vehicle, Channel channel) {
    return channel == Speed ? vehicle.speed_gauge : vehicle.tach_gauge;
}
inline int needle_ticks(const GaugeChannelConfig &gauge, float value) {
    value = bounded(value, gauge.max_units);
    const float angle = gauge.needle_zero_angle_deg +
        (value > gauge.needle_min_units ?
         (value-gauge.needle_min_units)*gauge.needle_degrees_per_unit : 0);
    return int(angle*gauge.needle_microsteps_per_degree + 0.5f);
}
struct Extrapolator {
    float last = 0, rate = 0;
    uint32_t reference_ms = 0;
    bool initialized = false;
    float value(float measurement, bool stale, uint32_t now_ms, float maximum) {
        if (stale) { *this = {}; return 0; }
        if (!initialized || measurement != last) {
            const uint32_t dt = now_ms-reference_ms;
            rate = initialized && dt ? (measurement-last)/float(dt) : 0;
            last = measurement; reference_ms = now_ms; initialized = true;
        }
        // Reference time is updated before computing the prediction horizon.
        return bounded(last + rate*float(now_ms-reference_ms), maximum);
    }
};
using Filter = HzSensorKalmanFilter<16>;
struct Reading {
    Filter::Snapshot filter{};
    float measured = 0;
    float extrapolated = 0;
    int target = 0;
};
struct Frame {
    Reading channel[2];
    uint32_t now_us = 0;
    uint32_t loop_us = 0;
    uint32_t worst_loop_us = 0;
    uint32_t event_flags = 0;
};
enum EventFlag : uint32_t { Startup=1, StaleTransition=2, ZeroTarget=4, Overflow=8, WorstLoop=16, DriverReset=32, RejectedTick=64 };
struct Event { Frame frame{}; uint32_t flags=0, reset_context=0; };
class Pipeline {
public:
    // Retained ring for debugger inspection. Sequence counts events, including
    // those overwritten; no formatting or printing occurs in interrupt context.
    Event history[16]{};
    uint32_t history_count = 0;
    void init(const VehicleConfig &vehicle, uint32_t now_us=0,
              uint32_t capture_arr=UINT32_MAX, uint32_t reset_context=0) {
        arr_ = capture_arr; reset_context_ = reset_context;
        for (unsigned i=0;i<2;++i) {
            gauges_[i] = gauge_config(vehicle,Channel(i));
            const auto &gauge=gauges_[i];
            Filter::Config c;
            c.units_per_hz = i == Speed ? vehicle.mph_per_hz : vehicle.rpm_per_hz;
            c.q_jerk = gauge.q_jerk;
            c.meas_var = gauge.meas_var;
            c.gate_sigma = gauge.gate_sigma;
            c.zero_speed_thresh_units = gauge.zero_speed_thresh_units;
            c.zero_periods_without_tick = gauge.zero_periods_without_tick;
            c.max_accel_units_per_s = gauge.max_accel_units_per_s;
            c.max_units = gauge.max_units;
            c.max_input_hz = gauge.max_input_hz;
            c.median_periods = gauge.median_periods;
            c.timestamp_resolution_us = 1000; // Current HAL_GetTick arrival clock.
            filters_[i].init(c, nullptr);
            seen_[i] = false; counters_[i] = 0; extrap_[i] = {};
        }
        frame_ = {}; frame_.now_us=now_us;
        last_update_ = now_us; have_update_ = false;
        extrap_time_ms_ = 0; extrap_last_us_ = now_us;
        history_count = 0; record(Startup);
    }
    // Called by the capture ISR. Main-loop state is never accessed here.
    void capture(Channel channel, uint32_t counter, uint32_t arrival_us) {
        const unsigned i = unsigned(channel);
        if (seen_[i]) {
            const uint32_t last = counters_[i];
            const uint32_t delta = counter >= last ? counter-last :
                uint32_t(uint64_t(arr_)-last+1+counter);
            filters_[i].tickAt(delta, arrival_us);
        }
        counters_[i] = counter; seen_[i] = true;
    }
    const Frame &update(uint32_t now_us, unsigned consume_mask=3) {
        const Frame previous = frame_;
        frame_.loop_us = have_update_ ? now_us-last_update_ : 0;
        last_update_ = now_us; have_update_ = true;
        frame_.now_us=now_us;
        uint32_t flags=0;
        if (frame_.loop_us > frame_.worst_loop_us) {
            frame_.worst_loop_us=frame_.loop_us; flags |= WorstLoop;
        }
        extrap_time_ms_ += (now_us-extrap_last_us_)/1000;
        for (unsigned i=0;i<2;++i) {
            if (!(consume_mask & (1u<<i))) continue;
            Reading &r=frame_.channel[i];
            r.filter=filters_[i].retrieveValueAt(now_us);
            r.measured=r.filter.stale ? 0 : r.filter.units;
            // Derive milliseconds from elapsed microseconds instead of the wrapped
            // absolute clock; extrapolation uses its own wrapping millisecond clock.
            r.extrapolated=extrap_[i].value(r.measured,r.filter.stale,
                extrap_time_ms_, gauges_[i].max_units);
            r.target=needle_ticks(gauges_[i], r.extrapolated);
            if (r.filter.stale != previous.channel[i].filter.stale) flags |= StaleTransition;
            if (r.target == needle_ticks(gauges_[i],0) &&
                previous.channel[i].target > r.target) flags |= ZeroTarget;
            if (r.filter.dropped_measurements != previous.channel[i].filter.dropped_measurements) flags |= Overflow;
            if (r.filter.rejected_ticks != previous.channel[i].filter.rejected_ticks) flags |= RejectedTick;
        }
        // Preserve fractional milliseconds through irregular sub-ms scheduling.
        extrap_last_us_ += ((now_us-extrap_last_us_)/1000)*1000;
        frame_.event_flags=flags;
        if (flags) record(flags);
        return frame_;
    }
    const Frame &frame() const { return frame_; }
    void driver_reset() { record(DriverReset); }
private:
    void record(uint32_t flags) {
        history[history_count++ % 16] = {frame_,flags,reset_context_};
    }
    GaugeChannelConfig gauges_[2]{};
    Filter filters_[2];
    uint32_t counters_[2]{};
    bool seen_[2]{};
    uint32_t arr_=UINT32_MAX, reset_context_=0, last_update_=0;
    uint32_t extrap_time_ms_=0, extrap_last_us_=0;
    bool have_update_=false;
    Extrapolator extrap_[2];
    Frame frame_{};
};
}
