#pragma once
#include "sensor_pipeline.hpp"
#include <cstdio>

namespace GaugeSensors {
enum class Scenario { Clean, Missing, Dropout, Extra, Jitter, Stall };
struct Truth { float speed=0, rpm=0; };
using Profile = Truth (*)(uint64_t elapsed_us);
struct SensorTrace {
    Frame frame{};
    uint32_t now_us=0;
    Truth truth{};
    uint32_t missing[2]{}, extra[2]{};
};
// Fixed 1 ms integration grid, independent of render/consumer scheduling.
class SensorSimulator {
public:
    Pipeline pipeline;
    void init(const VehicleConfig &, uint32_t start_us=0);
    void scenario(Scenario s, unsigned channel_mask=3);
    void advance(uint64_t elapsed_us, Profile profile);
    void reset();
    void dump(FILE *out=stdout) const;
    void stream(bool enabled) { streaming_=enabled; }
    const SensorTrace &latest() const { return latest_; }
    uint64_t elapsed() const { return elapsed_; }
private:
    VehicleConfig vehicle_{};
    uint32_t start_=0;
    uint64_t elapsed_=0, grid_=0, last_trace_=0;
    double phase_[2]{};
    uint64_t pending_extra_[2]{};
    uint32_t edges_[2]{}, missing_[2]{}, extra_[2]{};
    Scenario scenario_=Scenario::Clean;
    unsigned mask_=3;
    bool streaming_=false;
    SensorTrace latest_{};
    SensorTrace traces_[256]{};
    uint32_t trace_count_=0;
    void trace(FILE *, const SensorTrace &) const;
};
}
