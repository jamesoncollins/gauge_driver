#ifdef SENSOR_LEGACY
#include "legacy_filter.hpp"
#else
#include "HzSensorKalmanFilter.hpp"
#include "sensor_pipeline.hpp"
#endif
#include <cstdio>
static uint32_t clock_us;
static uint32_t clock_now() { return clock_us; }
int main() {
    using Filter = HzSensorKalmanFilter<16>;
    Filter speed, tach;
    Filter::Config s, t;
    s.units_per_hz = 0.8425872f * 1.015625f;
    s.zero_speed_thresh_units = 5;
    t.units_per_hz = 20;
    t.zero_speed_thresh_units = 400;
    speed.init(s, clock_now); tach.init(t, clock_now);
    std::puts("case,now_us,speed_stale,tach_stale,speed_units,tach_units,speed_tick_age_us,tach_tick_age_us");
    for (clock_us = 10000; clock_us <= 2000000; clock_us += 10000) {
        tach.tick(10000);
        if (clock_us % 20000 == 0) speed.tick(20000);
        if (clock_us <= 1000000 || clock_us == 2000000) {
            auto a = speed.retrieveValue(); auto b = tach.retrieveValue();
            if (clock_us >= 1000000)
                std::printf("stall,%u,%d,%d,%.3f,%.3f,%u,%u\n", clock_us,
                    a.stale, b.stale, a.units, b.units,
                    clock_us-a.last_tick_us, clock_us-b.last_tick_us);
        }
    }
    speed.init(s, clock_now);
    clock_us = 0xffff0000U; speed.tick(20000); speed.retrieveValue();
    clock_us = 0xffff8000U; speed.tick(20000);
    clock_us = 0x1000U;
    auto a = speed.retrieveValue();
    std::printf("rollover,%u,%d,0,%.3f,0,%u,0\n", clock_us, a.stale, a.units,
                clock_us-a.last_tick_us);
    const float last = 1000, measurement = 400;
    const unsigned elapsed = 20;
    const float rate = (measurement-last)/elapsed;
    std::printf("# historical_extrapolation measurement=400 target=%.0f\n", measurement+rate*elapsed);
    std::printf("# rollover_updated=%d\n",int(a.updated));
#ifndef SENSOR_LEGACY
    GaugeSensors::Extrapolator e;
    e.value(1000,false,10,10000);
    std::printf("# fixed_extrapolation measurement=400 target=%.0f\n",e.value(400,false,30,10000));
#endif
}
