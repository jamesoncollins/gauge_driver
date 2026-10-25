#!/usr/bin/env python3
"""Compare historical/current filters on queue-filling bursts at a 25 ms cadence."""
import argparse
import csv
import os
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BASELINE='fc1981c16411bc86549692bf2fd8406d9b804a4a'

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler',default=shutil.which('g++') or 'C:/msys64/ucrt64/bin/g++.exe')
    args=parser.parse_args()
    out=ROOT/'build/sensor-analysis'
    out.mkdir(parents=True,exist_ok=True)
    os.environ['PATH']=str(Path(args.compiler).resolve().parent)+os.pathsep+os.environ.get('PATH','')
    header=subprocess.check_output(['git','show',BASELINE+':./Custom/Inc/HzSensorKalmanFilter.hpp'],cwd=ROOT)
    (out/'legacy_filter.hpp').write_bytes(header)
    final_status=0
    for policy in ('keep-new','keep-old'):
        destination=out if policy=='keep-new' else out/'keep-old-control'
        destination.mkdir(parents=True,exist_ok=True)
        includes=['-I'+str(out),'-I'+str(ROOT/'Custom/Inc')]
        if policy=='keep-old':
            # Experimental copy: alter only queue overflow retention. Production
            # sources remain untouched, including freshness and extrapolation.
            header=(ROOT/'Custom/Inc/HzSensorKalmanFilter.hpp').read_text(encoding='utf-8')
            old_block="""        if (count_ == MAX_MEAS_Q) {
            tail_ = (tail_ + 1u) % MAX_MEAS_Q;
            --count_;
            ++dropped_; // discard oldest, retain the most recent periods
        }"""
            new_block="""        if (count_ == MAX_MEAS_Q) {
            ++dropped_;
            return; // experimental keep-oldest control, build artifact only
        }"""
            if header.count(old_block)!=1: raise RuntimeError('Queue control patch no longer matches production')
            (destination/'HzSensorKalmanFilter.hpp').write_text(header.replace(old_block,new_block),encoding='utf-8')
            (destination/'sensor_pipeline.hpp').write_bytes((ROOT/'Custom/Inc/sensor_pipeline.hpp').read_bytes())
            includes.insert(0,'-I'+str(destination))
        exe=destination/'sensor_burst_comparison.exe'
        subprocess.run([args.compiler,'-std=c++14','-O2','-Wall','-Wextra',
            '-DSENSOR_BURST_LEGACY','-DGAUGE_PLATFORM_SIMULATOR','-DGAUGE_DISPLAY_WIN32','-DGAUGE_SIM_PROFILE_3000GT_SOFT',
            *includes,str(ROOT/'Custom/Tests/sensor_burst_test.cpp'),
            str(ROOT/'Custom/Src/build_config.cpp'),'-o',str(exe)],cwd=ROOT,check=True)
        result=subprocess.run([str(exe),str(destination)],cwd=ROOT,text=True,capture_output=True)
        (destination/'burst-result.txt').write_text(result.stdout+result.stderr,encoding='utf-8')
        print('Queue policy:',policy)
        print(result.stdout,end='')
        if result.stderr: print(result.stderr,end='')
        final_status=max(final_status,result.returncode)
        with (destination/'burst-summary.csv').open(newline='',encoding='utf-8') as file:
            rows=list(csv.DictReader(file))
        for version in ('current','legacy'):
            part=[r for r in rows if r['version']==version]
            zero=[r for r in part if int(r['zero_frames'])]
            stale=[r for r in part if int(r['stale_frames'])]
            print(f'{version}: channel_cases={len(part)}, zero_target_channel_cases={len(zero)}, stale_channel_cases={len(stale)}')
            warm=[r for r in zero if r['cold_start']=='0' and r['timestamp_resolution']=='1ms']
            print('Warm 1 ms zero channel cases:',len(warm))
            tach_example=[r for r in part if r['channel']=='tach' and r['truth_rpm']=='1000.000000'
                and r['pulses']=='17' and r['spacing_us']=='50' and r['mask']=='2'
                and r['repeat_ms']=='25' and r['cold_start']=='0' and r['timestamp_resolution']=='1ms']
            if tach_example: print('1000 RPM example:',tach_example[0])
        # No-noise controls and numerical/freshness invariants are asserted by C++.
        # Exit 1 means a calibrated-zero command was reproduced, not a crash.
    print('Traces:',out)
    return final_status

if __name__=='__main__': raise SystemExit(main())
