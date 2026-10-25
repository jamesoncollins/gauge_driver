// Deterministic queue-saturating noise bursts with a strictly 25 ms consumer.
// SENSOR_BURST_LEGACY adds the historical filter and extrapolator side by side.
#ifdef SENSOR_BURST_LEGACY
#define HzSensorKalmanFilter LegacyHzSensorKalmanFilter
#include "legacy_filter.hpp"
#undef HzSensorKalmanFilter
#undef KF_LOCK
#endif
#include "sensor_pipeline.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
using namespace GaugeSensors;
namespace {
constexpr uint32_t loop_us=25000;
#ifdef SENSOR_BURST_LEGACY
uint32_t clock_us;
uint32_t now_fn() { return clock_us; }
#endif
unsigned errors=0;
#define CHECK(x) do { if(!(x)) { std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); ++errors; } } while(0)
struct Spec {
    unsigned id=0, pulses=0, spacing=0, mask=3, repeat_ms=0;
    bool cold=false, precise=false;
    float speed=0, rpm=0;
};
struct Edge { uint32_t time; Channel channel; bool noise; };
struct Stats {
    float min_filtered=1e30f, max_filtered=0, min_extrapolated=1e30f;
    uint32_t stale=0, zero=0, first_zero=0, drops=0,rejected=0;
    float final_filtered=0;
    bool finite=true;
};
struct Sample {
    uint32_t time=0;
    std::array<Reading,2> current{};
#ifdef SENSOR_BURST_LEGACY
    std::array<Reading,2> legacy{};
#endif
};
#ifdef SENSOR_BURST_LEGACY
// Exact pre-fix extrapolation behavior, including stale reset and clipping.
struct HistoricalExtrapolator {
    float last=0,rate=0; uint32_t last_ms=0;
    float value(float measurement,bool stale,uint32_t now,float maximum) {
        const uint32_t dt=now-last_ms;
        if(stale) { last=0;last_ms=now;rate=0;return 0; }
        if(measurement!=last) {
            rate=last_ms && dt ? (measurement-last)/float(dt) : 0;
            last=measurement; last_ms=now;
        }
        return bounded(last+rate*float(dt),maximum);
    }
};
struct HistoricalPipeline {
    LegacyHzSensorKalmanFilter<16> filter[2];
    HistoricalExtrapolator extrap[2];
    uint32_t counter[2]{};
    VehicleConfig vehicle;
    explicit HistoricalPipeline(const VehicleConfig &v):vehicle(v) {
        for(unsigned i=0;i<2;++i) {
            const auto &g=gauge_config(v,Channel(i));
            LegacyHzSensorKalmanFilter<16>::Config c;
            c.units_per_hz=i==Speed?v.mph_per_hz:v.rpm_per_hz;
            c.q_jerk=g.q_jerk;c.meas_var=g.meas_var;c.gate_sigma=g.gate_sigma;
            c.zero_speed_thresh_units=g.zero_speed_thresh_units;
            c.zero_periods_without_tick=g.zero_periods_without_tick;
            c.max_accel_units_per_s=g.max_accel_units_per_s;c.max_units=g.max_units;
            filter[i].init(c,now_fn);
        }
    }
    void capture(const Edge &e,uint32_t timestamp) {
        clock_us=timestamp;
        const auto period=e.time-counter[e.channel]; counter[e.channel]=e.time;
        filter[e.channel].tick(period);
    }
    std::array<Reading,2> update(uint32_t now) {
        clock_us=now;
        std::array<Reading,2> result;
        for(unsigned i=0;i<2;++i) {
            auto f=filter[i].retrieveValue(); auto &r=result[i];
            r.filter.units=f.units;r.filter.accel_units_s2=f.accel_units_s2;
            r.filter.stale=f.stale;r.filter.have_state=f.have_state;r.filter.updated=f.updated;
            r.filter.last_tick_us=f.last_tick_us;r.filter.tick_age_us=now-f.last_tick_us;
            r.filter.last_period_ticks=filter[i].last_dt_ticks();
            r.measured=f.stale?0:f.units;
            r.extrapolated=extrap[i].value(r.measured,f.stale,now/1000,gauge_config(vehicle,Channel(i)).max_units);
            r.target=needle_ticks(gauge_config(vehicle,Channel(i)),r.extrapolated);
        }
        return result;
    }
};
#endif
void update_stats(Stats &s,const Reading &r,const GaugeChannelConfig &g,uint32_t t) {
    s.finite &= std::isfinite(r.filter.units) && std::isfinite(r.filter.accel_units_s2) && std::isfinite(r.extrapolated);
    s.min_filtered=std::min(s.min_filtered,r.filter.units);
    s.max_filtered=std::max(s.max_filtered,r.filter.units);
    s.min_extrapolated=std::min(s.min_extrapolated,r.extrapolated);
    s.stale+=r.filter.stale;
    if(r.target==needle_ticks(g,0)) { if(!s.zero) s.first_zero=t; ++s.zero; }
    s.drops=r.filter.dropped_measurements;
    s.rejected=r.filter.rejected_ticks; s.final_filtered=r.filter.units;
}
std::vector<Edge> edges_for(const Spec &s,const VehicleConfig &v,uint32_t end) {
    std::vector<Edge> events;
    for(unsigned i=0;i<2;++i) {
        const double hz=(i==Speed?s.speed:s.rpm)/(i==Speed?v.mph_per_hz:v.rpm_per_hz);
        events.push_back({0,Channel(i),false}); // prime both capture adapters identically
        for(unsigned n=1;;++n) {
            const uint64_t t=uint64_t(std::llround(n*1000000.0/hz));
            if(t>end) break;
            events.push_back({uint32_t(t),Channel(i),false});
        }
    }
    // Each burst occupies one millisecond, immediately before a consumer call.
    // With the 2000 RPM / 50 Hz speed profile there are no genuine edges later
    // in this cell, so >=16 injected edges replace the entire current queue.
    const uint32_t first=s.cold?24000:5024000;
    const uint32_t burst_end=first+(s.repeat_ms?1000000:0);
    for(uint32_t start=first;start<=burst_end;start+=s.repeat_ms?s.repeat_ms*1000:2000000) {
        for(unsigned i=0;i<2;++i) if(s.mask&(1u<<i))
            for(unsigned n=0;n<s.pulses;++n)
                events.push_back({start+20+n*s.spacing,Channel(i),true});
    }
    std::sort(events.begin(),events.end(),[](const Edge &a,const Edge &b) {
        return a.time<b.time || (a.time==b.time && a.channel<b.channel);
    });
    // Coincident ideal/noise edges are one physical edge, not two zero-period captures.
    events.erase(std::unique(events.begin(),events.end(),[](const Edge &a,const Edge &b){
        return a.time==b.time && a.channel==b.channel;
    }),events.end());
    return events;
}
void row(FILE *out,const Spec &s,const char *version,unsigned i,const Stats &r) {
    if(!out)return;
    std::fprintf(out,"%u,%s,%s,%u,%u,%u,%u,%d,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%u,%lld,%d,%u,%.6f\n",
        s.id,version,i==Speed?"speed":"tach",s.pulses,s.spacing,s.mask,s.repeat_ms,int(s.cold),
        s.precise?"1us":"1ms",s.speed,s.rpm,r.min_filtered,r.max_filtered,r.min_extrapolated,
        r.zero,r.first_zero,r.stale,version[0]=='l'?-1LL:static_cast<long long>(r.drops),int(r.finite),r.rejected,r.final_filtered);
}
void trace_row(FILE *out,const Spec &s,const char *version,const Sample &sample,unsigned i,const Reading &r) {
    std::fprintf(out,"%u,%s,%u,%s,%.6f,%u,%u,%u,%d,%d,%.6f,%.6f,%.6f,%d,%u,%u,%u\n",s.id,version,
        sample.time,i==Speed?"speed":"tach",i==Speed?s.speed:s.rpm,
        r.filter.last_period_ticks,r.filter.last_tick_us,r.filter.tick_age_us,
        int(r.filter.updated),int(r.filter.stale),r.filter.units,r.filter.accel_units_s2,r.extrapolated,r.target,
        r.filter.rejected_ticks,r.filter.last_raw_period_ticks,r.filter.raw_tick_age_us);
}
struct Result { std::array<Stats,2> current;
    unsigned simultaneous_current=0,simultaneous_legacy=0;
#ifdef SENSOR_BURST_LEGACY
    std::array<Stats,2> legacy;
#endif
};
Result run(const Spec &s,const VehicleConfig &v,FILE *summary,const std::string &directory,bool &saved_current,bool &saved_legacy) {
    Pipeline p;p.init(v);
#ifdef SENSOR_BURST_LEGACY
    clock_us=0;HistoricalPipeline old(v);
#endif
    const uint32_t end=s.cold?4000000:9000000;
    const auto edges=edges_for(s,v,end);
    std::vector<Sample> samples;
    size_t next=0;Result result;
    for(uint32_t t=0;t<=end;t+=loop_us) {
        while(next<edges.size() && edges[next].time<=t) {
            const auto &e=edges[next++];
            const uint32_t timestamp=s.precise?e.time:(e.time/1000)*1000;
            p.capture(e.channel,e.time,timestamp);
#ifdef SENSOR_BURST_LEGACY
            old.capture(e,timestamp);
#endif
        }
        const auto &frame=p.update(t);
        CHECK(frame.loop_us==0 || frame.loop_us==loop_us);
        Sample sample;sample.time=t;
        for(unsigned i=0;i<2;++i) sample.current[i]=frame.channel[i];
#ifdef SENSOR_BURST_LEGACY
        sample.legacy=old.update(t);
#endif
        samples.push_back(sample);
        // Exclude normal initial sensor acquisition, but include every update
        // around a warm-state burst and all cold-start recovery after 200 ms.
        if(t>=(s.cold?200000:4900000)) {
            if(sample.current[Speed].target==needle_ticks(v.speed_gauge,0) &&
               sample.current[Tach].target==needle_ticks(v.tach_gauge,0)) ++result.simultaneous_current;
#ifdef SENSOR_BURST_LEGACY
            if(sample.legacy[Speed].target==needle_ticks(v.speed_gauge,0) &&
               sample.legacy[Tach].target==needle_ticks(v.tach_gauge,0)) ++result.simultaneous_legacy;
#endif
        }
        if(!s.cold && s.rpm==2000 && s.mask==3 && s.pulses==32 && s.spacing==10 && !s.repeat_ms && t==5025000) {
            for(unsigned i=0;i<2;++i) {
                // Noise that used to overwrite all 16 entries must now be
                // rejected before queue admission, not merely fail to move it.
                CHECK(sample.current[i].filter.dropped_measurements==0);
                CHECK(sample.current[i].filter.rejected_ticks>=32);
                CHECK(sample.current[i].filter.last_raw_period_ticks==10);
                CHECK(sample.current[i].filter.last_raw_tick_us==(s.precise?5024330u:5024000u));
                CHECK(sample.current[i].filter.last_period_ticks>=10000);
            }
        }
        if(t>=(s.cold?200000:4900000)) for(unsigned i=0;i<2;++i) {
            const auto &g=gauge_config(v,Channel(i));
            update_stats(result.current[i],sample.current[i],g,t);
#ifdef SENSOR_BURST_LEGACY
            update_stats(result.legacy[i],sample.legacy[i],g,t);
#endif
        }
    }
    bool zero_current=false,zero_legacy=false;
    for(unsigned i=0;i<2;++i) {
        row(summary,s,"current",i,result.current[i]);
        CHECK(result.current[i].finite);
        CHECK(!result.current[i].stale);
        const float truth=i==Speed?s.speed:s.rpm;
        CHECK(std::fabs(result.current[i].final_filtered-truth)<std::max(0.1f,truth*0.01f));
        zero_current |= result.current[i].zero!=0;
#ifdef SENSOR_BURST_LEGACY
        row(summary,s,"legacy",i,result.legacy[i]);
        CHECK(result.legacy[i].finite);
        zero_legacy |= result.legacy[i].zero!=0;
#endif
    }
    // Preserve the first complete failing trace for each version. Also keep a
    // representative warm-state simultaneous 32-edge burst even if it passes.
    const bool representative=!s.precise && !s.cold && s.mask==3 && s.pulses==32 && s.spacing==10 && !s.repeat_ms && s.rpm==2000;
    const bool tach_example=!s.precise && !s.cold && s.mask==2 && s.pulses==17 && s.spacing==50 && s.repeat_ms==25 && s.rpm==1000;
    const bool speed_example=!s.precise && !s.cold && s.mask==1 && s.pulses==16 && s.spacing==50 && s.repeat_ms==25 && s.speed==12;
    const bool save_current=zero_current && !saved_current;
    const bool save_legacy=zero_legacy && !saved_legacy;
    static bool saved_warm_current=false,saved_warm_legacy=false,saved_both_current=false,saved_both_legacy=false;
    const bool save_warm_current=!s.cold && zero_current && !saved_warm_current;
    const bool save_warm_legacy=!s.cold && zero_legacy && !saved_warm_legacy;
    const bool save_both_current=result.simultaneous_current && !saved_both_current;
    const bool save_both_legacy=result.simultaneous_legacy && !saved_both_legacy;
    if(!directory.empty() && (save_current || save_legacy || save_warm_current || save_warm_legacy || save_both_current || save_both_legacy || representative || tach_example || speed_example)) {
        const char *name=tach_example?"burst-tach-1000rpm.csv":speed_example?"burst-speed-12mph.csv":save_both_current?"burst-simultaneous-current-zero.csv":
            save_both_legacy?"burst-simultaneous-legacy-zero.csv":
            save_warm_current?"burst-warm-current-zero.csv":save_warm_legacy?"burst-warm-legacy-zero.csv":
            save_current?"burst-first-current-zero.csv":save_legacy?"burst-first-legacy-zero.csv":"burst-representative.csv";
        FILE *out=std::fopen((directory+"/"+name).c_str(),"w");CHECK(out);
        if(out) {
            std::fputs("case,version,now_us,channel,truth,last_period_ticks,last_edge_us,edge_age_us,updated,stale,filtered,rate_units_per_s,extrapolated,target,rejected_ticks,raw_period_ticks,raw_edge_age_us\n",out);
            for(const auto &sample:samples) for(unsigned i=0;i<2;++i) {
                trace_row(out,s,"current",sample,i,sample.current[i]);
#ifdef SENSOR_BURST_LEGACY
                trace_row(out,s,"legacy",sample,i,sample.legacy[i]);
#endif
            }
            std::fclose(out);
        }
        if(save_current || save_legacy || save_warm_current || save_warm_legacy || save_both_current || save_both_legacy || tach_example || speed_example) {
            const std::string edge_name=std::string(name).substr(0,std::string(name).size()-4)+"-edges.csv";
            out=std::fopen((directory+"/"+edge_name).c_str(),"w");CHECK(out);
            if(out) {
                std::fputs("case,capture_us,arrival_us,channel,noise\n",out);
                for(const auto &e:edges) std::fprintf(out,"%u,%u,%u,%s,%d\n",s.id,e.time,
                    s.precise?e.time:(e.time/1000)*1000,e.channel==Speed?"speed":"tach",int(e.noise));
                std::fclose(out);
            }
        }
    }
    saved_warm_current |= !s.cold && zero_current; saved_warm_legacy |= !s.cold && zero_legacy;
    saved_both_current |= result.simultaneous_current!=0; saved_both_legacy |= result.simultaneous_legacy!=0;
    saved_current |= zero_current;saved_legacy |= zero_legacy;
    return result;
}
}
int main(int argc,char **argv) {
    const std::string directory=argc>1?argv[1]:"";
    FILE *out=directory.empty()?nullptr:std::fopen((directory+"/burst-summary.csv").c_str(),"w");
    if(!directory.empty() && !out)return 2;
    if(out)std::fputs("case,version,channel,pulses,spacing_us,mask,repeat_ms,cold_start,timestamp_resolution,truth_speed,truth_rpm,min_filtered,max_filtered,min_extrapolated,zero_frames,first_zero_us,stale_frames,queue_drops,finite,rejected_ticks,final_filtered\n",out);
    const auto &v=get_build_config().vehicle;
    const float speeds[]={12,15,25,42.78763125f,60,80,160};
    const float rpms[]={750,1000,1500,2000,3000,6000,10000};
    unsigned cases=0,current_zero=0,legacy_zero=0,overflows=0,simultaneous_current=0,simultaneous_legacy=0;
    bool saved_current=false,saved_legacy=false;
    // Controls have identical sensor input and scheduling, with no noise.
    for(unsigned profile=0;profile<7;++profile) for(bool precise:{false,true}) {
        Spec s;s.id=cases++;s.speed=speeds[profile];s.rpm=rpms[profile];s.pulses=0;s.precise=precise;
        auto r=run(s,v,out,directory,saved_current,saved_legacy);
        for(auto &c:r.current) {CHECK(c.zero==0);CHECK(c.drops==0);}
#ifdef SENSOR_BURST_LEGACY
        for(auto &c:r.legacy) CHECK(c.zero==0);
#endif
    }
    for(unsigned profile=0;profile<7;++profile)
    for(unsigned pulses:{16u,17u,32u,64u,128u,256u,512u})
    for(unsigned spacing:{1u,2u,5u,10u,25u,50u}) {
        if((pulses-1)*spacing>900)continue;
        for(unsigned mask:{1u,2u,3u})
        for(unsigned repeat:{0u,100u,25u})
        for(bool cold:{false,true})
        for(bool precise:{false,true}) {
            Spec s;s.id=cases++;s.speed=speeds[profile];s.rpm=rpms[profile];s.pulses=pulses;
            s.spacing=spacing;s.mask=mask;s.repeat_ms=repeat;s.cold=cold;s.precise=precise;
            const auto r=run(s,v,out,directory,saved_current,saved_legacy);
            current_zero+=r.current[0].zero || r.current[1].zero;
            overflows+=r.current[0].drops || r.current[1].drops;
            simultaneous_current+=r.simultaneous_current!=0;
#ifdef SENSOR_BURST_LEGACY
            legacy_zero+=r.legacy[0].zero || r.legacy[1].zero;
            simultaneous_legacy+=r.simultaneous_legacy!=0;
#endif
        }
    }
    if(out)std::fclose(out);
    std::printf("25ms burst sweep: cases=%u overflow_cases=%u current_zero_cases=%u simultaneous_current_cases=%u",
        cases,overflows,current_zero,simultaneous_current);
#ifdef SENSOR_BURST_LEGACY
    std::printf(" legacy_zero_cases=%u simultaneous_legacy_cases=%u",legacy_zero,simultaneous_legacy);
#else
    (void)legacy_zero; (void)simultaneous_legacy;
#endif
    std::printf(" invariant_failures=%u\n",errors);
    // An observed current zero target is a reproduced behavioral failure, even
    // if all numerical, freshness and cadence invariants hold.
    return errors || current_zero ? 1 : 0;
}
