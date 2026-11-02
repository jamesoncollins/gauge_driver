#include "sim_sensors.hpp"
#include <algorithm>
#include <cmath>

namespace GaugeSensors {
void SensorSimulator::init(const VehicleConfig &vehicle, uint32_t start_us) {
    vehicle_=vehicle; start_=start_us; reset();
}
void SensorSimulator::reset() {
    elapsed_=grid_=last_trace_=0; trace_count_=0;
    for (unsigned i=0;i<2;++i) { phase_[i]=0; pending_extra_[i]=0; edges_[i]=missing_[i]=extra_[i]=0; }
    latest_={}; pipeline.init(vehicle_,start_);
}
void SensorSimulator::scenario(Scenario s,unsigned mask) {
    scenario_=s; mask_=mask; reset();
}
void SensorSimulator::advance(uint64_t elapsed_us,Profile profile) {
    if (elapsed_us < elapsed_) return;
    struct Edge { uint64_t time; unsigned channel; bool extra; };
    while (grid_+1000 <= elapsed_us) {
        const Truth truth=profile(grid_+500);
        Edge events[8]; unsigned count=0;
        for (unsigned i=0;i<2;++i) {
            if (pending_extra_[i] && pending_extra_[i]<=grid_+1000) {
                events[count++]={pending_extra_[i],i,true}; pending_extra_[i]=0;
            }
            const double units=i==Speed ? truth.speed : truth.rpm;
            double hz=bounded(float(units),gauge_config(vehicle_,Channel(i)).max_units) /
                (i==Speed?vehicle_.mph_per_hz:vehicle_.rpm_per_hz);
            const bool affected=(mask_ & (1u<<i)) != 0;
            if (affected && scenario_==Scenario::Jitter)
                hz *= 1+0.08*std::sin(double(grid_)*0.00071+double(i));
            const double add=hz*0.001;
            const double old_phase=phase_[i];
            phase_[i]+=add;
            while (phase_[i]>=1-1e-12 && count<8) {
                const double fraction=(1-old_phase)/add;
                // At supported sensor frequencies there is at most one normal
                // edge per channel per millisecond.
                const uint64_t t=grid_+uint64_t(std::llround(std::max(0.0,std::min(1000.0,fraction*1000))));
                phase_[i]=std::max(0.0,phase_[i]-1); ++edges_[i];
                bool drop=affected && ((scenario_==Scenario::Missing && edges_[i]%11==0) ||
                    (scenario_==Scenario::Dropout && t%8000000>=3000000 && t%8000000<4000000));
                if (drop) ++missing_[i];
                else events[count++]={t,i,false};
                if (!drop && affected && scenario_==Scenario::Extra && edges_[i]%11==0) {
                    const uint64_t extra_t=t+100;
                    if(extra_t<=grid_+1000) events[count++]={extra_t,i,true};
                    else pending_extra_[i]=extra_t;
                }
            }
        }
        std::sort(events,events+count,[](const Edge &a,const Edge &b){
            return a.time<b.time || (a.time==b.time && a.channel<b.channel);
        });
        for (unsigned j=0;j<count;++j) {
            const auto &e=events[j];
            const uint64_t absolute=uint64_t(start_)+e.time;
            pipeline.capture(Channel(e.channel),uint32_t(absolute),
                             uint32_t((absolute/1000)*1000));
            if (e.extra) ++extra_[e.channel];
        }
        grid_+=1000;
    }
    elapsed_=elapsed_us;
    const bool stall=scenario_==Scenario::Stall && elapsed_us%8000000>=3000000 && elapsed_us%8000000<4000000;
    const uint32_t now=uint32_t(((uint64_t(start_)+elapsed_us)/1000)*1000);
    if (!stall || mask_!=3) pipeline.update(now,stall ? (3u & ~mask_) : 3u);
    latest_.now_us=now; latest_.frame=pipeline.frame(); latest_.truth=profile(elapsed_us);
    for(unsigned i=0;i<2;++i) { latest_.missing[i]=missing_[i]; latest_.extra[i]=extra_[i]; }
    if (elapsed_us-last_trace_>=50000) {
        last_trace_=elapsed_us; traces_[trace_count_++%256]=latest_;
        if (streaming_) trace(stdout,latest_);
    }
}
void SensorSimulator::trace(FILE *out,const SensorTrace &t) const {
    for (unsigned i=0;i<2;++i) {
        const auto &r=t.frame.channel[i]; const auto &f=r.filter;
        std::fprintf(out,"%u,%u,%s,%.4f,%u,%u,%u,%u,%u,%u,%.4f,%d,%.4f,%d,%u,%u,%u,%u,%u,%u\n",
            t.now_us,t.frame.now_us,i==Speed?"speed":"tach",i==Speed?t.truth.speed:t.truth.rpm,
            f.last_period_ticks,f.tick_age_us,f.measurement_age_us,f.dropped_measurements,
            t.missing[i],t.extra[i],f.units,int(f.stale),r.extrapolated,r.target,
            t.frame.loop_us,t.frame.worst_loop_us,t.frame.event_flags,
            f.rejected_ticks,f.last_raw_period_ticks,f.raw_tick_age_us);
    }
}
void SensorSimulator::dump(FILE *out) const {
    std::fputs("now_us,frame_us,channel,truth,period_ticks,edge_age_us,estimate_age_us,queue_drops,missing_edges,extra_edges,filtered,stale,extrapolated,target,loop_us,worst_loop_us,event_flags,rejected_ticks,raw_period_ticks,raw_edge_age_us\n",out);
    const uint32_t first=trace_count_>256 ? trace_count_-256 : 0;
    for (uint32_t n=first;n<trace_count_;++n) trace(out,traces_[n%256]);
    std::fflush(out);
}
}
