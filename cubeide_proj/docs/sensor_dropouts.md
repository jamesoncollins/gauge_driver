# Needle dropout audit and sensor simulation

## Findings

The physical root cause is **not yet confirmed**. A deterministic reproduction
of the historical code does demonstrate a software path that commands both
needles to zero while both sensor inputs continue normally.

`fc1981c16411bc86549692bf2fd8406d9b804a4a` is the first parent of merge
`b8289b7` (PR #20), and is a useful pre-refactor comparison point. Its commit
message is "setup cmake for the multiplatform build"; that is not proof of a
hardware-qualified release. The later `good-working-copy-2026-08-02` tag also
cannot establish when this intermittent behavior started.

| Area | Baseline to pre-fix HEAD (`8eb9ad4`) | Classification |
| --- | --- | --- |
| TIM2 setup | Same 1 MHz counter, falling-edge CH3/CH4, input filter 15, ARR `0xfffffffe`, priority 0 | No capture configuration regression found |
| Capture callback | Same period subtraction; vehicle configuration replaces odometer constants | No substantial measurement algorithm change |
| Kalman header | Byte-for-byte identical | Queue and rollover defects are inherited |
| Filter initialization | Baseline enables capture before filter initialization; current bringup initializes first | Refactor improved initialization ordering |
| Generic refactor (`625a5a2`, `cdc6300`) | Used default filter tuning, DWT `get_us_32`, and direct filtered needle targets | Introduced response/timing differences |
| `af994f4` | Restored explicit speed/tach tuning and outer extrapolation, initially using the `updated` flag | Partial restoration |
| `42e396d` | Restored `HAL_GetTick()*1000`, value-change extrapolation, and the pre-update elapsed interval | Closely restored baseline behavior, including an inherited extrapolation defect |
| Needle mapping and driver | Calibration and Switec driver code unchanged | No driver algorithm regression found in this comparison |
| Scheduling | Rendering, BLE and shared-loop organization changed substantially | May expose inherited queue overflow; not evidence of an observed one-second hardware stall |

### Reproduced defects

1. **False staleness after consumer stalls.** At 50 Hz VSS / 100 Hz tach, pause
   consumption from 1 s to 2 s while continuing every edge. The old 16-slot
   queues retain their oldest measurements and discard recent ones. Both
   snapshots become stale despite a last-edge age of zero. The caller's stale
   handling therefore requests zero RPM and zero speed. The unchanged timeout
   thresholds are approximately 513.45 ms for speed and 150 ms for tach.
2. **Extrapolation overshoot.** An estimate changing from 1000 to 400 over 20 ms
   sets the reference to 400, then incorrectly applies that same preceding
   interval again: `400 + (400-1000) = -200`. Current hardware clamps that to
   zero. The fix returns 400 at the new reference time.
3. **Rollover queue blockage.** A queued pre-wrap timestamp is compared to a
   post-wrap `now` using ordinary unsigned ordering. The old code leaves that
   measurement pending (`updated=0`); the fixed version consumes it
   (`updated=1`). This occurs around 71.58 minutes and does not explain the
   user's observed drop within minutes of power-on.

Run from the MSYS2 UCRT environment:

```sh
python scripts/reproduce_sensor_baseline.py
cmake --build --preset x86-debug
ctest --test-dir build/x86-debug --output-on-failure
build/x86-debug/sensor_pipeline_test.exe build/sensor-analysis/scenarios.csv
```

The comparison writes `legacy.txt`, `fixed.txt` and corresponding CSV files to
`build/sensor-analysis/`. Representative stall rows at 2,000,000 us:

| Version | Speed stale | Tach stale | Speed | RPM | Last-edge ages |
| --- | --- | --- | --- | --- | --- |
| Baseline | true | true | 42.788 | 2000 | 0 / 0 us |
| Fixed | false | false | 42.788 | 2000 | 0 / 0 us |

## Implementation and compatibility

`GaugeSensors::Pipeline` owns capture arithmetic, vehicle-dependent filter
configuration, stale handling, extrapolation and calibrated motor targets.
Hardware and both simulator backends use this component. `VehicleConfig`
contains `speed_gauge` and `tach_gauge` profiles for output ceilings, Kalman
parameters, zero detection, acceleration limits and needle calibration. The
3000GT values are defined in `Custom/Src/build_config.cpp`; generic pipeline and
sensor-simulator code use those profiles rather than fixed vehicle constants.
The output ceiling is independent of the engine's RPM warning settings. Hardware reads each
capture register once; odometer counting remains in the ISR adapter. The first
edge primes the counter instead of treating time since timer startup as a
sensor period. Counter wrap uses the configured ARR modulus.

The production filter now offers `tickAt(period, arrival_us)` and
`retrieveValueAt(now_us)` alongside its callback-based API. Snapshots add edge
age, incorporated-measurement age, last period and discarded-measurement count.
Queue overflow discards the oldest measurement. Queue operations and bounded
batch copies use a short PRIMASK-preserving critical section on ARM and a
mutex on hosts; Kalman matrix operations execute with interrupts enabled.
There is one capture producer and one main-loop consumer. Initialization must
precede capture enablement. No filter tuning or vehicle calibration changed.
Virtual-zero updates are not inserted before real measurements in a backlog.

Timestamp ordering assumes consumption gaps shorter than half the uint32
microsecond range (about 35.8 minutes); normal clock rollover is supported.
A stopped input remains stale across long idle periods until a new edge arrives.
Unsigned diagnostic ages are modulo the clock range. Timing remains quantized
to milliseconds, matching the existing hardware implementation.

## Simulator controls and traces

The existing drive profile supplies ground truth and independent ECU data.
Sensor edges are derived from accumulated phase on a fixed 1 ms grid, with
microsecond edge locations and millisecond-quantized arrival timestamps. Draw
cadence does not change the generated edge stream. Consumer cadence can affect
the estimator, as on hardware. Scenario changes reset the virtual sensor/profile
clock and retained traces. Native console access is synchronized with updates.

Native examples:

```text
sensor clean
sensor missing speed
sensor dropout tach
sensor extra both
sensor jitter both
sensor stall both
sensor reset
sensor trace
sensor trace on
sensor trace off
```

Scenarios default to `both`. Missing drops every eleventh edge; extra inserts an
edge 100 us after every eleventh edge; jitter modulates frequency by up to 8%.
Dropout and stall last from 3 s through 4 s of every 8 s cycle. A stall with
`both` freezes pipeline consumption while edges continue; a single-channel
stall freezes only that channel's consumer. Unaffected channels continue normally.
The rest of the visual simulation continues, allowing fault inspection.

Browser developer-console equivalents (scenario IDs clean=0, missing=1,
dropout=2, extra=3, jitter=4, stall=5; masks speed=1, tach=2, both=3):

```js
Module._sim_control_sensor_scenario(5, 3);
Module._sim_control_sensor_reset();
Module._sim_control_sensor_trace(-1); // dump CSV
Module._sim_control_sensor_trace(1);  // stream on; 0 turns it off
```

The retained ring holds 256 samples at no more than 20 Hz; streaming is off by
default. CSV includes scheduled time, last consumed frame time, ground truth,
period, edge/estimate age, queue drops, injected missing/extra edge counts,
filtered estimate, stale state, extrapolated value, motor target, loop timing
and event flags. Stale transitions can be identified from the stale column.
During a stall, `frame_us` and estimator diagnostics remain at their last
consumed values; `now_us` and injected-edge counts continue advancing.

```sh
python scripts/smoke_sensor_simulators.py
```

This tests native text controls, all browser scenario/mask exports, invalid
selection, reset, CSV and stall recovery. It writes native/browser logs and a
browser stall CSV under `build/sensor-analysis/`, plus twelve exact 240x320
frames and a contact sheet under `build/web-debug/analysis/`. It records the
existing SDL main-loop timing startup diagnostic separately and rejects other
browser errors. The smoke script uses hidden native startup and headless Chrome.

## Hardware verification

1. Build both hardware debug presets. Load the appropriate firmware using the
   normal local flashing procedure. This work does not automatically flash it.
2. Inspect `g_sensor_pipeline.history` and `history_count` in the debugger.
   Sixteen event snapshots are retained, overwritten oldest first; the sequence
   count reveals overwritten entries. Each snapshot includes both channels.
   Flags: startup=1, stale transition=2, transition to zero target=4, queue
   overflow=8, new worst loop duration=16, software driver reset=32, rejected
   capture/cancelled candidate=64.
   Every entry carries the RCC CSR value captured during initialization.
3. Optionally enable the existing `PRINT_TO_USB` compile define. The main loop
   drains event records at the existing print interval, retries USB busy, and
   formats outside ISR context. A sequence gap indicates overwritten records.
   Channel fields are
   `measured/target/stale/qualified_edge_age/estimate_age/queue_drops/rejected_ticks/raw_edge_age`;
   time/age values are microseconds. This is a bounded diagnostic stream, not
   an interrupt trace or retained history across MCU resets.
4. Repeat the conditions that previously caused drops. On a bench, exercise
   steady pulses, independent signal interruption, both signals interrupted,
   and stop/restart. Expected signal-loss timeouts are unchanged. The synthetic
   regression suite requires recovery within 100 ms after its one-second loss
   at 50/100 Hz; this is not a universal recovery guarantee at every frequency.
5. At an observed drop, correlate the zero-target event with stale flags, edge
   ages, queue drops and loop duration. If the physical needles fall while
   software targets remain nonzero, investigate motor-driver reset, supply,
   pulse output and shared stepping infrastructure. If edge ages grow on both
   channels, inspect capture/interrupt delivery and the electrical inputs.
   A software backlog reproduction alone does not prove the physical cause.

Automated coverage includes startup, steady inputs, acceleration/deceleration,
stop/restart, all six scenarios on all three channel masks, overflow recovery,
future-edge deferral, timestamp zero and rollover, the hardware timer modulus,
variable cadence, identical manual-capture/simulator scheduling, deterministic
replay and concurrent producer/consumer queue access. Genuine input loss still
zeros the output; continuing normal pulses after a consumer stall must not.

## Validation results

- All four debug presets built successfully: x86, web, ST7789VI and S6E63D6.
- Native CTest: both sensor pipeline and existing GUI layout tests passed.
- All 18 scenario/channel combinations passed the deterministic sensor suite.
- Alternate vehicle profiles passed ceiling, simulator pulse-rate, needle
  calibration and zero-timeout tests, including a 12,000 RPM output ceiling.
- Native/browser command, export, trace and simultaneous-stall smoke checks passed.
- The optional `PRINT_TO_USB` branch passed an ARM syntax check.
- Twelve 240x320 web frames were captured and visually reviewed. Shift-circle
  coverage and the lower plastic mask are intentional; no new persistent
  element overlap or aperture clipping was identified in these frames.
- Existing ARM RWX linker and SDL startup timing diagnostics remain. Firmware
  was built but not flashed, and no physical hardware verification was performed.

## Follow-up: same-timestamp queue-filling bursts

The later [burst experiment](sensor_bursts.md) reproduces calibrated-zero
commands with a strict 25 ms loop. It also identifies a warmed-up tach noise
regression introduced by the keep-newest queue policy. The burst regression initially failed. The subsequent edge-qualification and
period-median correction now passes that regression and the earlier sensor
pipeline and GUI tests. This
follow-up supersedes any implication that the initial passing suite established
robustness against queue-filling noise bursts. The linked report records both the initial reproduction and the subsequent
production correction, including remaining hardware uncertainty.

The corrected CSV appends `rejected_ticks`, `raw_period_ticks` and
`raw_edge_age_us`. Existing `edge_age_us` now describes the latest qualified
edge; `period_ticks` is the period supplied to the filter after reconstruction
and optional median selection. Input-frequency ceilings and median selection
are configured in the vehicle's gauge profiles. See the burst report for the
root-cause trace, qualification contract, and response-delay tradeoff.
