#include "sensor_pipeline.hpp"
#include "sim_sensors.hpp"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <thread>
#include <atomic>
#include <initializer_list>
#include <algorithm>
using namespace GaugeSensors;
static unsigned failures=0;
#define CHECK(x) do { if(!(x)) { std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); ++failures; } } while(0)
static VehicleConfig vehicle() {
    return get_build_config().vehicle;
}
static Truth constant(uint64_t) { return {42.788f,2000}; }
static Truth ramps(uint64_t t) {
    if(t<1000000 || t>=6000000) return {};
    const float u=t<3500000 ? float(t-1000000)/2500000 : float(6000000-t)/2500000;
    return {80*u,5000*u};
}
static void check_frame(const Frame &f) {
    for(unsigned i=0;i<2;++i) {
        const auto &r=f.channel[i];
        CHECK(std::isfinite(r.filter.units)); CHECK(std::isfinite(r.filter.accel_units_s2)); CHECK(std::isfinite(r.extrapolated));
        CHECK(r.extrapolated>=0 && r.extrapolated<=gauge_config(vehicle(),Channel(i)).max_units);
        CHECK(r.target>=needle_ticks(gauge_config(vehicle(),Channel(i)),0));
    }
}
static void test_queue_and_stop() {
    Pipeline p; p.init(vehicle());
    CHECK(p.update(0).channel[0].filter.stale);
    for(uint32_t t=10000;t<=2000000;t+=10000) {
        p.capture(Tach,t,t);
        if(t%20000==0) p.capture(Speed,t,t);
        if(t<=1000000 || t==2000000) {
            const auto &f=p.update(t); check_frame(f);
            if(t>=1000000) for(auto &r:f.channel) {
                CHECK(!r.filter.stale); CHECK(r.measured>0);
                if(t==2000000) {
                    CHECK(r.filter.dropped_measurements>0);
                    CHECK(r.filter.measurement_age_us<=20000); CHECK(r.filter.tick_age_us<=20000);
                }
            }
        }
    }
    CHECK(!p.update(2149000).channel[Tach].filter.stale);
    CHECK(p.update(2151000).channel[Tach].filter.stale);
    CHECK(!p.update(2513000).channel[Speed].filter.stale);
    const auto &stopped=p.update(2514000);
    CHECK(stopped.channel[Speed].filter.stale);
    for(auto &r:stopped.channel) { CHECK(r.measured==0); CHECK(r.extrapolated==0); }
    for(uint32_t t=2600000;t<3100000;t+=10000) {
        p.capture(Tach,t,t); if(t%20000==0) p.capture(Speed,t,t); p.update(t);
    }
    for(auto &r:p.frame().channel) { CHECK(!r.filter.stale); CHECK(r.measured>0); }
}
static void test_rollover() {
    Filter f; Filter::Config c; f.init(c,nullptr);
    f.tickAt(20000,0xffff0000); f.retrieveValueAt(0xffff0000);
    f.tickAt(20000,0xffff8000); f.tickAt(20000,0);
    auto s=f.retrieveValueAt(0x1000);
    CHECK(s.updated); CHECK(!s.stale); CHECK(s.measurement_age_us==0x1000);
    CHECK(std::fabs(s.units-50)<0.01f);
    // Timestamp zero is valid, not an uninitialized sentinel.
    f.init(c,nullptr); f.tickAt(20000,0); CHECK(!f.retrieveValueAt(0).stale);
    f.tickAt(0,20000); CHECK(!f.retrieveValueAt(1000).updated);
    CHECK(f.retrieveValueAt(1000000).stale);
    CHECK(f.retrieveValueAt(0x80001000U).stale);
    f.init(c,nullptr);
    f.tickAt(20000,1000);
    CHECK(!f.retrieveValueAt(999).updated);
    CHECK(f.retrieveValueAt(1000).updated);
    Pipeline p; p.init(vehicle(),0xfffff000,0xfffffffe);
    p.capture(Speed,0xfffff000,0xfffff000);
    p.capture(Speed,0x1000,0x1000);
    const auto &r=p.update(0x2000).channel[Speed];
    CHECK(r.filter.last_period_ticks==8191); CHECK(!r.filter.stale);
}
static void test_extrapolation() {
    Extrapolator e;
    CHECK(e.value(1000,false,10,10000)==1000);
    CHECK(e.value(400,false,30,10000)==400);
    CHECK(e.value(400,false,31,10000)==370);
    CHECK(e.value(400,true,32,10000)==0);
    CHECK(e.value(900,false,33,10000)==900);
    e={}; e.value(1000,false,0xfffffff0,10000);
    CHECK(e.value(900,false,0x10,10000)==900);
    CHECK(needle_ticks(vehicle().speed_gauge,10)==36); CHECK(needle_ticks(vehicle().tach_gauge,500)==60);
    CHECK(needle_ticks(vehicle().speed_gauge,NAN)==36);
}
static void test_scenarios(FILE *trace) {
    SensorSimulator sim;
    for(unsigned scenario=0;scenario<=5;++scenario) for(unsigned mask=1;mask<=3;++mask) {
        sim.init(vehicle()); sim.scenario(Scenario(scenario),mask);
        for(uint64_t t=0;t<=6500000;t+=10000) {
            sim.advance(t,constant); const auto &f=sim.pipeline.frame(); check_frame(f);
            for(unsigned i=0;i<2;++i) {
                // Genuine 1 s sensor loss intentionally zeros the output;
                // allow 100 ms for measurement/filter reacquisition afterwards.
                const bool reacquiring=scenario==2 && (mask&(1u<<i)) && t>=3000000 && t<4100000;
                if(t>100000 && !reacquiring) {
                    CHECK(!f.channel[i].filter.stale);
                    CHECK(f.channel[i].measured>0);
                    CHECK(f.channel[i].target>needle_ticks(gauge_config(vehicle(),Channel(i)),0));
                }
            }
            if(scenario==2 && t==3800000) for(unsigned i=0;i<2;++i)
                CHECK(f.channel[i].filter.stale==bool(mask&(1u<<i)));
            if(scenario==5 && t==4000000) for(unsigned i=0;i<2;++i) {
                CHECK(!f.channel[i].filter.stale);
                CHECK(f.channel[i].filter.dropped_measurements>0 || !(mask&(1u<<i)));
            }
        }
        const auto &tr=sim.latest();
        for(unsigned i=0;i<2;++i) {
            if((scenario==1 || scenario==2) && (mask&(1u<<i))) CHECK(tr.missing[i]>0);
            if(scenario==3 && (mask&(1u<<i))) CHECK(tr.extra[i]>0);
        }
        if(trace) { std::fprintf(trace,"# scenario=%u mask=%u\n",scenario,mask); sim.dump(trace); }
    }
    sim.init(vehicle());
    for(uint64_t t=0;t<=8000000;t+=10000) { sim.advance(t,ramps); check_frame(sim.pipeline.frame()); }
    for(auto &r:sim.pipeline.frame().channel) CHECK(r.filter.stale && r.measured==0);
}
static void test_cadence_and_shared_path() {
    SensorSimulator a,b; a.init(vehicle()); b.init(vehicle());
    // Different rendering/consumer cadences must generate the same edge stream.
    for(uint64_t t=0;t<2000000;t+=7000) a.advance(t,constant);
    a.advance(2000000,constant); b.advance(2000000,constant);
    for(unsigned i=0;i<2;++i) {
        const auto &x=a.pipeline.frame().channel[i].filter;
        const auto &y=b.pipeline.frame().channel[i].filter;
        CHECK(x.last_tick_us==y.last_tick_us); CHECK(x.last_period_ticks==y.last_period_ticks);
        CHECK(!y.stale);
    }
    // Manual hardware capture adapter and simulator use exactly the same pipeline.
    Pipeline p; p.init(vehicle()); SensorSimulator s; s.init(vehicle());
    // 2000 RPM => exact 100 Hz; speed zero avoids rounding calibration concerns.
    auto tach_only=[](uint64_t)->Truth { return {0,2000}; };
    for(uint32_t t=1000;t<=1000000;t+=1000) {
        if(t%10000==0) p.capture(Tach,t,t);
        if(t%10000==0) {
            const auto &f=p.update(t); s.advance(t,tach_only);
            const auto &r=s.pipeline.frame();
            CHECK(f.channel[Tach].measured==r.channel[Tach].measured);
            CHECK(f.channel[Tach].target==r.channel[Tach].target);
        }
    }
    // Full deterministic replay, including virtual sensor/consumer faults.
    a.scenario(Scenario::Extra,3); b.scenario(Scenario::Extra,3);
    for(uint64_t t=0;t<=2000000;t+=17000) {
        a.advance(t,ramps); b.advance(t,ramps);
        for(unsigned i=0;i<2;++i) CHECK(a.pipeline.frame().channel[i].target==b.pipeline.frame().channel[i].target);
    }
}
static void test_vehicle_profile() {
    auto custom=vehicle();
    custom.speed_gauge.max_units=220;
    custom.tach_gauge.max_units=12000;
    custom.tach_gauge.needle_min_units=800;
    custom.tach_gauge.needle_zero_angle_deg=7;
    custom.tach_gauge.needle_degrees_per_unit=0.01f;
    custom.tach_gauge.needle_microsteps_per_degree=10;
    CHECK(needle_ticks(custom.tach_gauge,0)==70);
    CHECK(needle_ticks(custom.tach_gauge,800)==70);
    CHECK(needle_ticks(custom.tach_gauge,12000)==1190);
    CHECK(needle_ticks(custom.tach_gauge,20000)==1190);
    // A fresh filter's first valid period exposes its configured ceiling without
    // settling/transient effects. Exercise ceilings above and below 3000GT values.
    for(float ceiling : {3000.0f,12000.0f}) {
        custom.tach_gauge.max_units=ceiling;
        Pipeline p; p.init(custom);
        p.capture(Tach,0,0); p.capture(Tach,1600,1000);
        const auto &r=p.update(3000).channel[Tach]; // 12,500 RPM input, edge qualified
        CHECK(r.filter.units==ceiling); CHECK(r.extrapolated==ceiling);
        CHECK(r.target==needle_ticks(custom.tach_gauge,ceiling));
    }
    {
        Pipeline p; p.init(custom);
        p.capture(Speed,0,0); p.capture(Speed,2000,2000);
        const auto &r=p.update(5000).channel[Speed];
        CHECK(r.filter.units==220); CHECK(r.extrapolated==220);
        CHECK(r.target==needle_ticks(custom.speed_gauge,220));
    }
    custom.speed_gauge.max_units=25;
    custom.tach_gauge.max_units=1000;
    SensorSimulator sim; sim.init(custom);
    sim.advance(200000,constant); // ground truth exceeds both configured ceilings
    CHECK(sim.pipeline.frame().channel[Speed].filter.last_period_ticks>30000);
    CHECK(sim.pipeline.frame().channel[Tach].filter.last_period_ticks==20000);
    for(unsigned i=0;i<2;++i) {
        const auto &r=sim.pipeline.frame().channel[i];
        const auto &gauge=gauge_config(custom,Channel(i));
        CHECK(r.filter.units<=gauge.max_units); CHECK(r.extrapolated<=gauge.max_units);
        CHECK(r.target<=needle_ticks(gauge,gauge.max_units));
    }
    // Zero detection is also selected from the vehicle profile.
    custom.tach_gauge.zero_speed_thresh_units=200;
    Pipeline p; p.init(custom);
    p.capture(Tach,0,0); p.capture(Tach,20000,20000); p.update(20000);
    CHECK(!p.update(319000).channel[Tach].filter.stale);
    CHECK(p.update(321000).channel[Tach].filter.stale);
}
static void test_edge_qualification() {
    Filter f; Filter::Config c;
    c.units_per_hz=20; c.max_units=10000; c.max_input_hz=1000;
    c.timestamp_resolution_us=1000; c.zero_speed_thresh_units=400;
    f.init(c,nullptr);
    f.tickAt(20000,20000);
    CHECK(!f.retrieveValueAt(21000).have_state);
    auto r=f.retrieveValueAt(22000);
    CHECK(r.updated && r.have_state && !r.stale); CHECK(r.units==1000);
    // A plausible first noise interval must be cancelled as part of the burst.
    f.tickAt(4020,24000);
    for(unsigned n=0;n<16;++n)f.tickAt(50,24000);
    r=f.retrieveValueAt(25000);
    CHECK(!r.updated); CHECK(r.units==1000); CHECK(r.rejected_ticks==17);
    CHECK(r.last_tick_us==20000); CHECK(r.last_raw_tick_us==24000);
    CHECK(r.raw_tick_age_us==1000); CHECK(r.dropped_measurements==0);
    f.tickAt(15180,40000); // Actual time 40000 - final spurious edge at 24820.
    r=f.retrieveValueAt(42000);
    CHECK(r.updated); CHECK(r.last_period_ticks==20000); CHECK(r.units==1000);
    CHECK(r.last_raw_period_ticks==15180);
    // Noise alone cannot refresh a qualified signal forever.
    uint32_t last=40000;
    for(uint32_t start=60000;start<=210000;start+=25000) {
        for(unsigned n=0;n<17;++n) {
            const uint32_t edge=start+n*50;
            f.tickAt(edge-last,(edge/1000)*1000); last=edge;
        }
        r=f.retrieveValueAt(start+5000);
    }
    CHECK(r.stale); CHECK(r.last_tick_us==40000); CHECK(r.raw_tick_age_us<25000);
    // Quantized times must not release a candidate prematurely.
    f.init(c,nullptr); f.tickAt(20000,20000); f.retrieveValueAt(22000);
    f.tickAt(4990,24000); CHECK(!f.retrieveValueAt(25000).updated);
    f.tickAt(50,25000); f.tickAt(14960,40000);
    r=f.retrieveValueAt(42000); CHECK(r.updated); CHECK(r.last_period_ticks==20000);
    CHECK(r.units==1000);
    // Candidate confirmation, including timestamp zero, works across rollover.
    f.init(c,nullptr); f.tickAt(20000,0xffffff00);
    r=f.retrieveValueAt(0x800); CHECK(r.updated); CHECK(r.units==1000);
    CHECK(r.last_tick_us==0xffffff00); CHECK(!r.stale);
}
static void test_missed_edge_period_median() {
    Filter f;Filter::Config c;
    c.units_per_hz=20;c.max_input_hz=1000;c.timestamp_resolution_us=1000;
    c.zero_speed_thresh_units=400;c.median_periods=true;
    f.init(c,nullptr);
    for(uint32_t t=20000;t<=60000;t+=20000) { f.tickAt(20000,t); f.retrieveValueAt(t+2000); }
    // One missed pulse produces a doubled raw period, not an actual RPM drop.
    f.tickAt(40000,100000);
    auto r=f.retrieveValueAt(102000);
    CHECK(r.last_raw_period_ticks==40000);CHECK(r.last_period_ticks==20000);
    CHECK(std::fabs(r.units-1000)<0.1f);
    f.tickAt(20000,120000);r=f.retrieveValueAt(122000);CHECK(std::fabs(r.units-1000)<0.1f);
    // A sustained real frequency change must replace the median, not be hidden.
    f.tickAt(10000,130000);f.retrieveValueAt(132000);
    f.tickAt(10000,140000);r=f.retrieveValueAt(142000);CHECK(r.last_period_ticks==10000);
    CHECK(r.units>1000);
    // History cannot conceal the new frequency after a genuine stopped interval.
    CHECK(f.retrieveValueAt(500000).stale);
    f.tickAt(360000,500000);f.retrieveValueAt(502000);
    f.tickAt(40000,540000);r=f.retrieveValueAt(542000);
    CHECK(r.last_period_ticks==40000);
}
static void test_clean_response_after_qualification() {
    auto profile=vehicle();auto reference=profile;
    reference.speed_gauge.max_input_hz=reference.tach_gauge.max_input_hz=0;
    reference.speed_gauge.median_periods=reference.tach_gauge.median_periods=false;
    SensorSimulator guarded,plain;guarded.init(profile);plain.init(reference);
    float maximum_rpm_difference=0,maximum_speed_difference=0;
    for(uint64_t t=0;t<=8000000;t+=25000) {
        guarded.advance(t,ramps);plain.advance(t,ramps);
        const auto &g=guarded.pipeline.frame();const auto &p=plain.pipeline.frame();
        check_frame(g);
        for(const auto &r:g.channel)CHECK(r.filter.rejected_ticks==0);
        if(t>=2000000 && t<=5000000) {
            maximum_rpm_difference=std::max(maximum_rpm_difference,std::fabs(g.channel[Tach].measured-p.channel[Tach].measured));
            maximum_speed_difference=std::max(maximum_speed_difference,std::fabs(g.channel[Speed].measured-p.channel[Speed].measured));
            for(const auto &r:g.channel) CHECK(!r.filter.stale && r.measured>0);
        }
    }
    // A pending edge can delay incorporation by one 25 ms main-loop iteration;
    // allow that timing difference without permitting a material response loss.
    std::printf("clean response versus unqualified input: max RPM difference %.3f, max MPH difference %.3f\n",maximum_rpm_difference,maximum_speed_difference);
    CHECK(maximum_rpm_difference<200); CHECK(maximum_speed_difference<5);
    for(const auto &r:guarded.pipeline.frame().channel) CHECK(r.filter.stale && r.measured==0);
}
static void test_concurrent_queue() {
    for(bool qualification:{false,true}) {
        Filter f; Filter::Config c; c.max_units=100;
        if(qualification) { c.max_input_hz=1000;c.timestamp_resolution_us=1000;c.median_periods=true; }
        f.init(c,nullptr);
        std::atomic<uint32_t> now{0}; std::atomic<bool> done{false};
        std::thread producer([&]{for(uint32_t t=1000;t<=2000000;t+=1000) {
            f.tickAt(20000,t); now.store(t);
        } done.store(true);});
        while(!done.load()) { auto s=f.retrieveValueAt(now.load()); CHECK(std::isfinite(s.units)); }
        producer.join(); auto s=f.retrieveValueAt(now.load()); CHECK(!s.stale); CHECK(std::isfinite(s.units));
    }
}
int main(int argc,char **argv) {
    FILE *trace=argc>1 ? std::fopen(argv[1],"w") : nullptr;
    if(argc>1 && !trace) return 2;
    test_queue_and_stop(); test_rollover(); test_extrapolation();
    test_scenarios(trace); test_cadence_and_shared_path(); test_vehicle_profile(); test_edge_qualification(); test_missed_edge_period_median(); test_clean_response_after_qualification(); test_concurrent_queue();
    if(trace) std::fclose(trace);
    std::printf("sensor_pipeline_test: %u failures\n",failures);
    return failures ? 1 : 0;
}
