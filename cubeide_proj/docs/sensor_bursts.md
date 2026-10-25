# Same-timestamp sensor burst test

## Current status: fixed and verified

The production input path now rejects impossible close-edge clusters before
queue admission and applies a median of three qualified periods. The current
pipeline passes all 6,818 scenarios: **zero calibrated-zero failures, zero false
stale frames, zero numerical/cadence/control failures**. Every case also verifies
recovery to within 1% of the true input (or 0.1 unit) after the noise stops.
The historical implementation still produces 540 failing cases on the same
inputs, preserving a useful differential reproduction.

The root cause is more specific than queue overflow: a burst supplies many
correlated, implausible measurements. Soft Kalman gating reduces each one's
influence but never rejects it outright. When genuine measurements are displaced,
uncertainty grows and the repeated corrections can drive the internal estimate
far outside the display range. On recovery, the model carries a large negative
rate while downweighting valid sensor measurements as deviations from its now
wrong prediction. The predicted estimate can then cross the needle minimum.
Identical millisecond timestamps are not necessary for this failure.

An instrumented pre-fix 1,000 RPM run recorded 400,000 RPM burst measurements.
At 6,025,000 us the internal estimate reached about 18,626 RPM even though the
reported output was capped at 10,000. After normal ticks resumed, the model's
rate reached -4,000 RPM/s. At the last correction before 6,475,000 us, a genuine
1,000 RPM measurement only raised the estimate from about 487 to 537 RPM;
prediction to the main-loop time then lowered it to about 496 RPM, mapping to
60 motor steps. The burst-specific trace is preserved at
`build/sensor-analysis/root-cause/filter-updates-before.csv`.

### Input-path correction

- `max_input_hz` in each vehicle gauge profile defines a generous capture ceiling,
  separate from the output ceiling: 500 Hz for speed, 1,000 Hz for tach in the
  3000GT profile. These allow about 428 MPH / 20,000 RPM, respectively, so normal
  operation and the gauge's full range have headroom.
- Hold a candidate edge until either a following edge has sufficient spacing or
  its quiet interval has elapsed. A too-close edge cancels the preceding
  candidate as well as rejecting the new edge; otherwise the plausible first
  interval of a noise burst could leak through.
- Sum actual capture ticks across rejected edges. The next qualified period is
  measured from the last qualified edge rather than from the final noise edge.
- A genuine edge may itself be lost inside a rejected cluster. The profile's
  `median_periods` option uses three qualified periods so a single doubled
  interval is not interpreted as a sudden slowdown. A sustained frequency
  change still replaces the median, and history resets after a stopped interval.
- Qualification, reconstruction and median selection are bounded integer work
  inside the existing short queue guard. Kalman matrix work remains in the main
  loop. The 1 ms arrival clock is retained; a conservative resolution margin
  prevents it from prematurely certifying a candidate.
- Staleness uses qualified edges. Raw edge age and `rejected_ticks` are reported
  separately, so a noise-only stream does not keep an old nonzero reading fresh.
  Rejected counts include cancelled candidates, which can include genuine edges
  overlapping the cluster; they are not an exact count of electrical noise edges.

The formerly failing warmed-up 1,000 RPM case now remains at 1,000 RPM throughout,
with no queue drops and no zero command. Both keep-newest and keep-oldest queue
controls pass the entire burst sweep once qualification is enabled. The normal
long-stall backlog test still passes, so the current keep-newest policy is retained.

Clean acceleration/deceleration, stop/restart, missed edges, real frequency
changes, qualification timing, rollover and concurrent capture/consumption are
covered separately. On the test ramp, the maximum difference versus the old
unqualified input path was 30.364 RPM and 1.132 MPH. The median can add roughly
one sensor period of response delay and quiet qualification can defer the latest
sample to the next main-loop iteration; no Kalman gains were retuned.

All three native tests pass; all four debug presets build; optional USB diagnostic
syntax and native/browser smoke tests pass. Twelve simulator frames were captured
and reviewed. Firmware has not been flashed, and the physical simultaneous-drop
root cause is still unconfirmed. Arbitrary electrical interference can produce
other edge patterns that this sweep does not model.

Pre-fix traces and summaries remain under `build/sensor-analysis/before-noise-fix/`;
current corrected traces are at the original paths listed below.

## Pre-fix result (historical record)

**Reproduced calibrated-zero commands at a strictly 25 ms consumer cadence.**
The current pipeline and the historical implementation both fail some noise
cases. Neither needs a slow main loop, timestamp rollover, or stale detection
to produce these commands. No simultaneous speed-and-tach zero command was
observed in this sweep.

This is a software reproduction under injected captured edges. It does not
establish that the car produces these edge patterns, or prove the cause of its
simultaneous needle drop. The hardware input filter and electrical waveform are
not simulated; very short spacing cases are deliberately adversarial.

## Experiment

`Custom/Tests/sensor_burst_test.cpp` sends actual edge times through the shared
capture adapter. It calculates periods from successive edges, rather than
inventing RPM measurements. The historical comparison uses the filter header
from `fc1981c` with its original queue retention and pre-fix extrapolation.

- Genuine input remains constant, covering seven profiles from 12 MPH / 750 RPM
  through 160 MPH / 10,000 RPM.
- The main loop consumes every 25,000 us; this is asserted on every update.
- Noise bursts contain 16, 17, 32, 64, 128, 256, or 512 additional edges, spaced
  1, 2, 5, 10, 25, or 50 us apart. Only combinations fitting within one
  millisecond are included. Spurious captures have nonzero periods.
- Test speed, tach, and both; single bursts and bursts every 100 ms or 25 ms
  for one second; both initial acquisition and five-second stabilized operation.
- In the millisecond-timestamp cases, every noise edge in a burst receives the
  exact same quantized arrival timestamp. Actual capture periods remain precise.
- Repeat the same scenarios with precise microsecond arrival timestamps as a
  control. Add fourteen clean controls across profiles and timestamp resolutions.
- Each queue-policy variant runs 6,818 cases. The calibrated-zero test uses the
  configured needle minimum target (36 speed steps / 60 tach steps), rather than
  requiring the numerical RPM/MPH estimate to be exactly zero.

The current queue overflowed in 6,720 cases. All numerical, cadence and clean
control assertions passed; no false stale frames occurred. The current
keep-newest policy produced a zero command in 276 cases; the historical path
produced one in 540 cases. These are counts from deliberately chosen scenarios,
including cold starts and both timestamp resolutions, not failure probabilities.
No paired scenario changed from zero to nonzero, or vice versa, solely by
switching between 1 ms and 1 us timestamps. Filter values can still differ.

## Warmed-up 1,000 RPM reproduction

1. Genuine tach remains at 1,000 RPM; genuine speed remains at 15 MPH.
2. Run normally for five seconds.
3. Inject 17 extra tach edges spaced 50 us apart, occupying 800 us. The first
   burst is at 5,024,020 through 5,024,820 us, all stamped 5,024,000 us.
4. Repeat every 25 ms through 6,024,820 us. Continue genuine pulses throughout,
   including after the bursts stop. Main-loop cadence remains 25 ms.
5. Observe the command at 6,475,000 us, about 450 ms after the last burst.

| Path | Filtered RPM range | Zero-point command | Stale |
| --- | --- | --- | --- |
| Current, keep newest | About 496.23 to 10,000 | Yes: target 60 for one 25 ms frame | No |
| Historical | About 991.14 to 1,065.61 | No | No |
| Current with only queue policy changed to keep oldest | About 991.14 to 1,065.61 | No | No |

The current filter estimate itself crosses below the 500 RPM needle minimum.
The zero command in this case is not introduced by the outer extrapolator.
This table reports the bounded `Snapshot.units`; the private filter state is
not logged by this harness.

**The queue-policy-only control confirms a regression from the recent
keep-newest change for this tach scenario.** It is not sufficient to claim that
all current failures are caused by that policy: the historical implementation
also fails other cases, and retaining old samples has its own backlog drawbacks.
No production queue or filter changes were made during the initial diagnostic
experiment; the subsequent correction is documented at the top of this report.
The experimental keep-old control is generated only under the build directory.

## Existing historical failure

At a constant 12 MPH / 750 RPM, sixteen extra speed edges spaced 50 us apart,
repeated every 25 ms for one second after five seconds of normal operation,
produce a speed zero-point command in both historical and current paths.
At 7,100,000 us, the historical filtered estimate is about 10.06 MPH and its
old extrapolation lowers the command below the needle minimum; the current
estimate itself is about 9.52 MPH. See the CSV for exact samples. This is a
low-speed reproduction, not a demonstrated simultaneous needle drop.

## Run and inspect

From MSYS2 UCRT in the repository:

```sh
source scripts/env-msys2-ucrt.sh
python scripts/test_sensor_bursts.py
```

The script builds the historical/current comparison, then repeats it with only
the current overflow policy changed in generated build-artifact headers.
**The command now exits with status 0 for the corrected pipeline.** It still
runs the historical failing comparison. Results go to `build/sensor-analysis/`:

- `burst-summary.csv`: all current/historical scenario summaries.
- `burst-tach-1000rpm.csv` and its `-edges.csv`: detailed tach reproduction and
  the exact captured edge stream.
- `burst-speed-12mph.csv` and its `-edges.csv`: the low-speed reproduction.
- `burst-representative.csv`: a warmed-up 32-edge burst filling both queues with
  same-timestamp noise; direct assertions check overflow and last capture data.
- `burst-result.txt`: current run summary.
- `keep-old-control/`: equivalent results with the queue-policy-only control.

Also available through the native build:

```sh
cmake --build --preset x86-debug
ctest --test-dir build/x86-debug --output-on-failure
```

The `sensor_burst_test` fails if it observes a calibrated-zero command under
these nonzero constant inputs. With the correction, it and the sensor pipeline
and GUI tests all pass. Raw/qualified edge diagnostics are appended to trace CSVs.

## Follow-up: minutes at the zero point

The user reported the real needles could remain at their calibrated zero points
for minutes, a materially different duration from the burst-recovery undershoot.
A follow-up virtual-time experiment held the consumer at exactly 25 ms and ran
six steady input profiles with single bursts, one second of repeated bursts, or
three minutes of repeated bursts. Every case then ran for another 30 seconds
with clean captured edges. The historical, pre-noise-fix and corrected paths
were compared. Results are under `build/sensor-analysis/long-duration/`, including
the standalone `test.cpp` and `before.csv` / `after.csv`.

The longest uninterrupted zero command was 800 ms in the pre-fix pipeline and
700 ms in the historical implementation, even after three minutes of this noise
pattern. The corrected pipeline had no zero commands in those noise cases.
This does not reproduce the duration of the car symptom and is not proof that
other noise patterns cannot produce it.

A separate control deliberately withheld capture callbacks for three minutes
while the main loop continued at 25 ms and the simulated true inputs remained
nonzero. That produced about 179.925 seconds continuously at a zero point,
including in the corrected code: staleness intentionally parks a needle when
usable measurements stop arriving. This demonstrates how sustained loss of the
capture stream fits the reported duration, not that such a loss occurred in the
car. Raw edge age, qualified edge age, stale state, filter values and targets
must be correlated on hardware before claiming the physical root cause.
