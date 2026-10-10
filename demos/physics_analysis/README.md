# Waypoint flight and physics attack analysis

English | [한국어](README_ko.md)

This optional demo runs the existing **ArduCopter 4.6.2 / QEMU / Rumoca FMU**
setup through ten zigzag waypoints and RTL. A baseline and four disturbance or
sensor attack cases share the same mission. Recording, attack models, scenario
control, analysis, and Three.js visualization all live under this directory;
FastDyn core, its shared Modelica models, and submodules are unchanged.

## Mission and cases

`scenarios/_mission.toml` defines the common mission: take off to 15 m, follow
`(N,E) = (10,-10), (20,10), …, (90,-10), (100,10)` meters from home, then RTL.
The helper streams **GUIDED position targets**; this is waypoint following,
not an uploaded AUTO mission. Each target requires the FC's estimated position
to stay within 2 m horizontally and ±1 m vertically for 1 simulation second.
The waypoint phase has a 360 simulation-second timeout. RTL includes descent,
a sustained low-altitude check, and ordinary disarming. The original
`DISARM_DELAY=0` and initialization parameter file are preserved.

| CLI scenario | Injection, in addition to the common mission |
|---|---|
| `baseline` | No disturbance or sensor injection |
| `magnetic` | Body-FRD magnetic field offset proportional to modeled propulsion current; default scale 1 |
| `wind` | Wind from 270°: 5 m/s background + up to 10 m/s gusts, peak 15 m/s |
| `gps-spoof` | Smooth GPS displacement: N +12 m, E −8 m, U 0 m; consistent velocity displacement |
| `gyro` | Yaw bias +3°/s, false sinusoidal yaw rate ±8°/s and roll rate ±2°/s at 0.25 Hz |

The four injections use **absolute FMU time 140–200 s**, with 10 s smoothstep
ramps at both ends. The plateau is 150–190 s. They affect later waypoints and
part of RTL in the validated runs. These are not seconds since takeoff.
The helper rejects an attack run if takeoff finishes after the attack starts.
The baseline retains the same nominal window for before/during/after analysis.
The old hover, altitude-excitation and rectangular-route scenarios have been
replaced. Monte Carlo cases also use the common zigzag mission.

Edit an individual scenario's `[attack]` table or pass a custom TOML path. A
custom file inherits the common mission and can override its fields. Gyro bias
and false-rate amplitudes are separately configurable; setting one group to
zero permits separate experiments. Angles/rates in configuration are degrees
and degrees/second; gyro outputs to the controller are radians/second.

## Build and run

Run from the FastDyn root. The existing FastDyn environment, QEMU/plugin,
firmware, Rumoca compiler and FMI3 headers must already be built. Install only
the optional demo package when you need these tools:

```bash
fastdyn-env/bin/python -m pip install -e 'demos/physics_analysis[test]'
fastdyn-env/bin/python demos/physics_analysis/demo.py build
fastdyn-env/bin/python demos/physics_analysis/demo.py run baseline \
  --output out/physics_analysis/my-baseline
fastdyn-env/bin/python demos/physics_analysis/demo.py campaign attacks \
  --output out/physics_analysis/my-attacks
```

`campaign attacks` runs baseline, magnetic, wind, GPS and gyro cases in that
order, retaining failures and exporting a combined replay. `run gps-spoof`
executes just that case. `run --duration` overrides the waypoint timeout;
`--timeout` is the whole-run wall-clock watchdog (default 900 s).

Default build: `out/physics_analysis/waypoint-attacks/build/`, containing
`pristine/`, `observed/` and `build.json`. The demo compiles with the local
`third_party/common/rumoca/target/debug/rumoca`; use `build --compiler PATH`
for another compiler. Only the observed FMU has the recording wrapper.
Legacy demo builds are rejected. Build and run directories are never reused;
choose fresh `--output` and `--build-dir` paths for rebuilds and reruns.
No original-study firmware or recorded results are copied.

```bash
# Baseline plus three repeatable wind parameter samples (four runs).
fastdyn-env/bin/python demos/physics_analysis/demo.py campaign monte-carlo \
  --seed 42 --count 3 --output out/physics_analysis/my-mc
# Prepare scenarios/configuration without starting QEMU.
fastdyn-env/bin/python demos/physics_analysis/demo.py campaign attacks \
  --prepare-only --output out/physics_analysis/my-plan
```

All runs have isolated RAM, QMP sockets, ports and logs. For concurrent runs,
use different `--port-base` values, e.g. 15550 and 15650; each reserves four
consecutive monitor/firmware/GCS/GPS ports. Keep output paths short enough for
Unix sockets. Fixed seeds reproduce sampled parameters, not bit-identical
asynchronous QEMU/MAVLink runs. Small campaigns do not establish reliability.

## Model and injection boundary

`models/PhysicsAnalysis/Copter.mo` composes the existing `FastDyn.Copter` as
`vehicle`. `AttackInputs.mo` generates the time-varying signals. Wind replaces
still-air drag with relative-air drag through an external force difference;
zero wind adds zero force. Sensor injections affect only the exported `mag`,
`gps`, `vel_ned` and `gyro` channels. They do not directly overwrite plant
position, attitude or angular velocity. GPS altitude and vertical velocity use
consistent signs; the barometer remains independent. No GPS truth heading is
sent to the controller: demo `yaw_deg=0` means unavailable in `GPS_INPUT`.
Quaternion-derived truth heading is retained exclusively for analysis.

Magnetic slopes are adapted from the sibling study, in body-FRD Gauss/A:
`[0.00489157, 0.00473074, -0.00704143]`. The illustrative propulsion-current proxy
is `max(0, 0.42 + 42.5908 * mean(omega_i² / omega_max²)) A`. This is **not an
electrical/battery model or measured current**. It is logged as `model_current_a`;
`current_a` and `voltage_v` remain blank (`null` in replay). These parameters
are experiment inputs, not calibrated claims about a physical aircraft.

The cases represent model-level sensor corruption and environmental stress;
no radio attack or external GPS transmission is performed. This does not
reproduce the original study's current compensation or CompassMot results.

## Live Rumoca viewer and offline replay

“Online” here means a live local viewer of the same simulation, not an upload
to a hosted Rumoca service. The server serves the actual checked-out Rumoca
viewer and quadrotor scene, with demo-only trajectory/waypoint overlays.

```bash
# May start before the run; the bridge waits for the truth file.
fastdyn-env/bin/python demos/physics_analysis/demo.py serve \
  --truth out/physics_analysis/my-baseline/truth.csv
# Or view the recorded five-case campaign:
fastdyn-env/bin/python demos/physics_analysis/demo.py serve \
  --replay out/physics_analysis/my-attacks/replay.json
```

Live: <http://127.0.0.1:8080/>. Replay: <http://127.0.0.1:8080/replay/>.
`--port` and `--ws-port` change HTTP/WS ports (defaults 8080/8081). Both `--truth`
and `--replay` can be supplied together. The server binds to localhost and
ignores all viewer flight-control messages; only the mission helper controls
the simulation. The FMU writes a CSV and never waits for a browser.

Replay supports two-case comparison, play/pause, speed, time seeking, camera
modes and trail length. Numbered rings show all ten planned targets, including
unreached targets; the gray leg is RTL. The aircraft and solid colored line
show **plant truth**. Yellow wireframe marker/track show the **FC estimate**.
Charts show altitude, heading error, position-estimate error and segment error;
shading denotes the scheduled injection window. Readouts include injected
signals and the distinctly labeled modeled current proxy.

After changing `--replay` or replacing the selected file, reload any already-open
replay tab. The page loads the recording when opened; restarting a server alone
does not replace data already loaded in a browser. The startup log shows the
absolute replay path, and the page's `Source:` label shows its folder/file.
Replay responses bypass the browser cache. A missing replay file is rejected
at startup; only a live `--truth` file may be created later.

All Three.js, mesh and texture assets are local; replay needs no internet/CDN,
but requires the HTTP server rather than `file://`. The mesh's illustrative
scale and propeller rotation do not represent collision geometry.

```bash
fastdyn-env/bin/python demos/physics_analysis/demo.py export \
  out/physics_analysis/run-a out/physics_analysis/run-b \
  --output out/physics_analysis/comparison.json
# Legacy study recordings remain importable.
fastdyn-env/bin/python demos/physics_analysis/demo.py export /path/to/truth.csv \
  --output out/physics_analysis/imported.json
```

Imported CSVs need `sim_time_s,north_m,west_m,alt_m,qw,qx,qy,qz`. Other channels
are optional; absent telemetry, targets and outcomes remain unavailable.

## Metrics and reproducibility

Every recorded run, including a failed run, exports `replay.json`,
`replay.metrics.json` and `replay.metrics.csv`.

- `completed` means the **FC mission and RTL landing** finished. It does not
  certify that all real-world target positions were reached under spoofing.
- `waypoints` audits each currently commanded target against plant truth.
  `truth_waypoints_reached` requires continuous dwell within the same 2 m / ±1 m
  target volume for 1 s, allowing one 20 Hz sample interval (0.051 s) for boundary
  uncertainty. It also records minimum distance and true distance at the FC's
  reported arrival. Passing near a future target does not count it early.
- Segment RMS is distance to the current commanded leg, not the closest leg
  of the whole planned route. Target distance and altitude errors are separate.
- Heading error is wrapped FC yaw minus quaternion-derived compass heading.
  Position-estimate error is horizontal FC position minus true position.
  Reports give RMS, absolute P95 and absolute maximum.
- Phase metrics cover before/during/after the scheduled injection during
  **waypoint following**. RTL is excluded from these tracking statistics and
  has a separate final true horizontal home error. Injection peaks and recorded
  injection windows cover the full recording, including RTL.
- Truth altitude ≤0.3 m after exceeding 2 m during waypoint flight marks possible
  contact; later samples are excluded. This is not a contact-force crash detector.
- Recording is approximately 20 Hz. Positions use NWU; quaternions are wxyz,
  body FLU → world NWU. Display coordinates are `[-W,U,N]`. Attitude interpolation
  uses shortest-path SLERP; position interpolation is linear. There is no
  extrapolation or interpolation through gaps longer than 0.5 s.
- Clock mapping is `truth_sim_s = MAVLink boot_s + boot_offset_s` (default zero).
  Verify `--boot-offset-s` when changing firmware. Comparisons use absolute
  simulation time and the intersection of valid windows, without per-case time
  shifts. Separate before/during/after statistics use the shared attack schedule.
- Statistics are sample-weighted; irregular imported CSVs are not time-weighted.
  Missing values remain unavailable, never zero. An observed injection is
  reported separately from flight completion and from full schedule coverage.

Runs retain scenario, initialization parameters, isolated FastDyn config, raw
truth/telemetry/command CSVs, event log, mission/result JSONs, MAVLink tlog and
execution logs. `run.json` records firmware/FMU/source hashes and coordinate
conventions. Exported data records the analysis source hash used for that export.

## Validation and optional video

```bash
PHYSICS_DEMO_TEST_BUILD=out/physics_analysis/waypoint-attacks/build \
  fastdyn-env/bin/python -m pytest demos/physics_analysis/tests -q
node --test demos/physics_analysis/web/math.test.js
# With optional video dependencies and a running server:
fastdyn-env/bin/python -m pip install -e 'demos/physics_analysis[video]'
fastdyn-env/bin/python -m playwright install chromium
fastdyn-env/bin/python demos/physics_analysis/tests/browser_check.py \
  --output out/physics_analysis/browser-check.png
# ffmpeg must also be installed.
fastdyn-env/bin/python demos/physics_analysis/demo.py video \
  --start 160 --end 165 --fps 24 --output out/physics_analysis/flight.mp4
```

See [the actual five-flight validation results](VALIDATION.md) and
[sources/licenses](CREDITS.md). Passing the demo's execution checks does not
mean the controller rejected an attack or retained accurate navigation.
