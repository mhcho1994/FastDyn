# Waypoint attack integration validation

Validated locally on 2026-10-09, branch `feature/physics_analysis_demo`, based
on `feature/reconfig-modelica` at `3e9123c`. This is one run per case on the
existing ArduCopter 4.6.2 firmware; it is not statistical evidence of attack
resistance or a reproduction of the original study's compensation results.

## Actual flight results

All five runs followed the same ten-waypoint zigzag mission at 15 m and
completed FC waypoint acceptance, RTL descent and disarming. All four attack
injections were observed in FMU truth logs over 140.0026–199.9526 s, and every
recording extends beyond the complete 140–200 s schedule.

The following statistics use each case's own valid waypoint-flight window.
“Truth dwell” additionally requires the true vehicle to remain inside the
2 m horizontal / ±1 m vertical target volume for 1 s (0.051 s sampling tolerance).
FC completion and true navigation performance are intentionally separate.

| Case | FC mission | Truth dwell | Segment RMS (m) | Heading abs P95 (°) | Position estimate RMS (m) | RTL home error (m) |
|---|---|---|---|---|---|---|
| none | 10/10 + RTL | 9/10 | 0.511 | 5.83 | 0.869 | 0.014 |
| magnetic | 10/10 + RTL | 9/10 | 0.517 | 6.31 | 0.873 | 0.009 |
| wind | 10/10 + RTL | 10/10 | 0.538 | 6.97 | 0.848 | 0.003 |
| gps | 10/10 + RTL | 4/10 | 8.871 | 13.73 | 10.347 | 0.013 |
| gyro | 10/10 + RTL | 7/10 | 1.366 | 32.22 | 0.551 | 0.101 |

The baseline physically entered all ten target regions, but its final target
had only 0.80 s of recorded truth dwell when the FC advanced to RTL, so the
stricter continuous-dwell audit reports 9/10. This is not changed to 10/10 to
match the FC. In the GPS case the FC reports all ten targets while only four
satisfy the truth dwell audit: the false GPS solution steers the vehicle away
from the actual target coordinates. Recovery before landing allows a small
final home error despite large errors during the attack.

All-case common valid window: **112.502610688–178.352610688 s**. Common-window
segment RMS: baseline 0.509 m, magnetic 0.525 m, wind 0.519 m, GPS 8.498 m,
gyro 1.304 m. Common-window heading absolute P95: 4.49°, 4.59°, 5.99°, 13.76°,
32.49°, respectively. No per-case time shifting is used. The full attack
schedule extends into RTL; tracking metrics exclude RTL.

Recorded injection peak norms:

- Magnetic offset: 0.2011 Gauss (current-dependent, FRD).
- Wind: 14.9999 m/s.
- GPS offset: 14.4222 m (N +12, E −8).
- Gyro bias: 0.05236 rad/s (yaw 3°/s); false-rate vector: 0.14392 rad/s
  (roll 2°/s and yaw 8°/s peaks together).

## Checks performed

- Compiled pristine and observed FMI3 FMUs with the pinned local Rumoca compiler.
- **17 Python tests passed**, including actual compiled FMU calls: recording
  preserves integration states; sensor injections preserve open-loop physical
  states; wind changes motion; magnetic/gyro additions use the documented units;
  GPS offset and velocity derivative agree, including vertical sign; the GPS
  yaw output is unavailable; unaffected accelerometer/barometer channels match.
  Tests also cover spoofed FC arrival versus truth, contiguous dwell, missing
  telemetry, wrapped heading, CSV imports, campaign sampling and read-only WS.
- **4 Node tests passed**: NWU display axes, SLERP, gap/out-of-range behavior,
  and separation of estimated and true positions.
- Chromium/Playwright passed the real Rumoca viewer, live route overlay,
  five-case replay, baseline/GPS selection, attack indicator, time/camera/playback
  controls and local-only asset loading. A comparison screenshot was inspected.
- The attack campaign planner prepared all five cases. Seed-42 Monte Carlo
  prepared baseline + three wind cases and was checked for repeatable parameters
  and identical routes. Monte Carlo flight trials were not run.
- Video export captured the new baseline/GPS replay over 160–162 s at 12 fps;
  see the artifact below. The earlier demo's watchdog/process-cleanup check
  remains applicable; it was not repeated for this model update.

## Evidence

Paths are relative to the FastDyn root; generated recordings/builds are ignored
by Git. None are original-study recordings.

| Artifact | Path |
|---|---|
| FMUs and build provenance | `out/physics_analysis/waypoint-attacks/build/` |
| Five complete run directories | `out/physics_analysis/waypoint-attacks/{baseline,magnetic,wind,gps-spoof,gyro}/` |
| Combined replay | `out/physics_analysis/waypoint-attacks/replay.json` |
| Full per-case/common-window metrics | `out/physics_analysis/waypoint-attacks/replay.metrics.json` |
| Spreadsheet metrics | `out/physics_analysis/waypoint-attacks/replay.metrics.csv` |
| Browser screenshot | `out/physics_analysis/waypoint-attacks/replay-check.png` |
| Video smoke check | `out/physics_analysis/waypoint-attacks/replay-check.mp4` |
| Prepared campaigns | `out/physics_analysis/waypoint-attacks/{attack-plan,mc-plan}/` |

For this validation, optional Python/browser dependencies were placed in ignored
`out/physics_analysis/test-deps` and `out/physics_analysis/browsers`. Commands used
`PYTHONPATH=out/physics_analysis/test-deps` and, for browser tools,
`PLAYWRIGHT_BROWSERS_PATH=out/physics_analysis/browsers`. Core environment package
requirements were not changed. README installation instructions are the normal
setup path for a new checkout.

Core Python/C/Modelica sources, baseline config and submodule files remain
unchanged. The pre-existing dirty state of
`third_party/courbet_deps/fire_modelica_models` is preserved. Changes are confined
to this demo and the pre-existing documentation link added to the root README.
