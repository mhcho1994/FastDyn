# PX4 v1.17.0-rc2: 23–27 kHz acoustic sweep

This experiment uses the current FIRE P2 quadrotor, the actual FMUv6C ARM ELF
running NuttX, and QEMU/FastDyn. Confirm the PX4 version using `ver all`: the
commit is `0b6e4687defb353a34201951809efd3f0040a9ba`, matching the
[official v1.17.0-rc2 tag](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0-rc2).
Follow the [PX4 profile](../virtuals/physics/flight_controllers/courbet/px4/fmu_v6c/README.md)
for the base runtime environment.

## Running the experiment

Run the commands from the FastDyn root. The dependencies are the same as for the
base P2 profile: patched QEMU, `fastdyn-env`, C build tools, Rust/Rumoca, and the
portable Modelica library. Use new output directory names, since FastDyn replaces
an existing output directory when it is reused.

```bash
source fastdyn-env/bin/activate
ninja -C build
python utils/build_fmi3_fmu.py --config configs/px4_fmu_v6c_acoustic_sweep.toml --skip-submodules
mkdir -p out/acoustic-example-logs

fastdyn run -c configs/px4_fmu_v6c_acoustic_baseline.toml --no-build-fmu \
  -o out/acoustic-example-control > out/acoustic-example-logs/control.log 2>&1
cp out/acoustic-example-logs/control.log out/acoustic-example-control/console.log

fastdyn run -c configs/px4_fmu_v6c_acoustic_sweep.toml --no-build-fmu \
  -o out/acoustic-example-sweep > out/acoustic-example-logs/sweep.log 2>&1
cp out/acoustic-example-logs/sweep.log out/acoustic-example-sweep/console.log

python virtuals/gyro_acoustic/tools/analyze.py \
  out/acoustic-example-control out/acoustic-example-sweep --output out/acoustic-example-report
```

Both runs use the same newly built FMU. Run them sequentially; each is observed
for 120 seconds of host time after the mission starts. With
`--no-exit-on-complete`, recording continues until the observation period ends,
including after ground impact or an abnormal disarm. **A process exit code of 0
alone does not establish normal flight.** The analyzer checks that the FMU
binaries match, verifies the PX4 commit, and checks arming and mission start,
completion of the observation period, finite state values, and sweep/recovery
records.

The mission is normal arming → takeoff to 15 m → loiter for 50 seconds → landing.
The viewer is available at `http://127.0.0.1:5000/mavcesium/` by default, but its
attitude display shows PX4 estimates. This experiment uses the FMU traces
described below to assess the actual simulated vehicle state. The environment
variables `FASTDYN_MISSION_FILE`/`FASTDYN_PARAM_FILE`/`FASTDYN_BRIDGE_BOOT` take
precedence over the configuration, so clear any overrides left from other
experiments.

## TOML configuration and device assumptions

Complete run configurations:

- [sweep](../configs/px4_fmu_v6c_acoustic_sweep.toml)
- [0 Pa control](../configs/px4_fmu_v6c_acoustic_baseline.toml)

The two files differ only in `rms_pressure_pa`. The key settings are:

```toml
[CPU.cpu0.plugins.gyro_acoustic]
enabled = true
rms_pressure_pa = 20.0
start_frequency_hz = 23000.0
end_frequency_hz = 27000.0
duration_s = 20.0
airborne_altitude_m = 5.0
delay_after_airborne_s = 3.0
ramp_s = 0.25
axis_coupling = [1.0, 0.0, 0.0]
reference_frequency_hz = 25000.0
sense_frequency_hz = 25000.0
integration_step_s = 1e-5
```

- A single linear sweep starts 3 **simulation seconds** after the FMU altitude
  first reaches 5 m. The frequency schedule is independent of host execution
  speed. No injection occurs if the vehicle does not take off.
- `rms_pressure_pa` is the RMS acoustic pressure at the sensor location, not a
  speaker power or distance command. The pressure follows a raised-cosine ramp
  over `ramp_s` at the start and end.
- `axis_coupling` contains signed coupling coefficients for the FRD roll/pitch/yaw
  sensor axes. This example couples only to roll. It applies to all gyro data
  published by the PX4 bridge; it does not select a particular physical IMU or
  reproduce redundant IMU voting.
- `effective_area_m2`, `mems_mass_kg`, `drive_amplitude_m`, `sense_q`,
  `afe_tau_s`, and `demod_tau_s` are all explicitly specified in the TOML.
  **The 25 kHz resonance and coupling coefficients are illustrative assumptions,
  not measured or identified parameters of a commercial gyro.** These results
  therefore cannot establish vulnerable frequencies, required acoustic pressure,
  or crash thresholds for a real vehicle.
- The plugin is inactive when `enabled=false` or its table is omitted. The
  control run uses `enabled=true` and `rms_pressure_pa=0` to retain the same
  recording and trigger paths.

## Model and integration point

```text
RMS acoustic pressure + frequency schedule from TOML
  → mechanical transfer function for a single sense mode + analog front end
  → phase-continuous demodulation + 2-pole LPF
  → false rate [rad/s]
  → added to clean FMU gyro → PX4 sensor_gyro/uORB → EKF/controller
  → actual PX4 motor outputs → P2 FMU dynamics
```

The equations match the direct sense-force component of FIRE's
`Vulnerabilities.Acoustic.EquivalentGyroResponse` with `driveCoupling=0` and
`crossStiffness=0`. The mechanical/AFE response, including resonance, is evaluated
as a quasi-steady response at the instantaneous frequency. The filters after
demodulation are integrated in the time domain.

```text
H(w) = 1 / (m*(ws²-w²) + j*m*ws*w/Qs) / (1+j*w*afeTau)
scale = 2*m*Xd*wr*abs(H(wr))
phase(t) = 2*pi*((f0-fr)*t + 0.5*((f1-f0)/T)*t²) + phase0 - arg(H(wr))
raw = -sqrt(2)*pRms*Aeff*real(H(w)*exp(j*phase(t))) / scale
falseRate = LPF2(raw) * axis_coupling
```

`phase(t)` integrates frequency rather than simply using `f(t)*t`. The 23–27 kHz
carrier is not sampled directly at the 1 ms PX4 period. Instead, the demodulated
beat, which reaches at most ±2 kHz, is processed at a default internal interval
of 10 µs. The filter uses an exact two-pole update for each midpoint input.
Configuration validation requires at least 40 internal steps per beat period
and at least 20 per filter time constant. Convergence between 10 µs and 5 µs
steps was tested separately.

This is an **external reduced sensor model**, not a direct integration of the
25 kHz MEMS mechanical motion in the current FIRE I3 model. It does not model
mechanical ring-up during fast sweeps, AGC/PLL transients, ADC aliasing,
saturation/quantization, package/room acoustics, or physical damage. Do not
combine it with another acoustic IMU model in a way that duplicates the effect.
These TOML configurations select P2 with ideal I0 sensors, so the gyro, altitude,
and velocity values recorded before injection in `truth_*` correspond to the
physical state. When this plugin is used with an FMU whose sensors include bias
or noise, those fields represent clean sensor values before acoustic injection
and must not be interpreted as independent ground truth. SIH `*_groundtruth`
is not used.

## Outputs and assessment

Each run produces the following files in `run-artifacts/gyro_acoustic/`:

- `settings.json`: the coefficients and exposure conditions actually applied.
- `response.txt`: a versioned input artifact for the native model.
- `trace.csv`: FMU time, sweep elapsed time/frequency/pressure, false rate on
  three axes, gyro values before and after injection, clean FMU altitude and
  NED velocity, and the four actual PX4 PWM channels.

The default CSV recording rate is now up to 1 kHz, with some 2 ms intervals
during early boot. This rate is intended for observing flight state, not for
reconstructing the full ±2 kHz beat. The numerical model's internal integration
runs separately at 100 kHz.

The analyzer compares the runs from sweep start until 10 seconds after sweep
end. This window precedes the planned landing in the example mission. If the
FMU altitude of a vehicle that has already taken off falls to 0.3 m or below
within this window, the analyzer records `early_ground_contact=true`. Ground
contact is evidence of abnormal descent within the simulation; the landing-gear
model does not predict physical damage. If the mission timing or groundZ is
changed, review the assessment window and altitude threshold as well.

`summary.json` and `comparison.png` compare the two runs' maximum false rate,
altitude range, descent speed, horizontal displacement, and early ground
contact. Horizontal displacement is computed by integrating FMU NED velocity.
Use `console.log` and `mav.tlog` to inspect PX4 STATUSTEXT messages, modes, and
motor outputs in more detail.

## Validation

```bash
python -m pytest -q tests/unit/test_gyro_acoustic.py tests/unit/test_fmu_runtime.py
```

The tests cover zero acoustic pressure, the analytical value for a resonant
tone, chirp phase, both frequency endpoints, internal step convergence, filter
decay after injection ends, rejection of invalid settings, and injection into
the FMU sensor batch. A fresh P2 FMI3 FMU was built from the current sources,
and both runs used the same binary. The earlier P2/Rumoca build limitations
still described in the FIRE documentation were not reproduced in this
environment.


## Results from this workspace (2026-10-26)

Both runs used the newly built P2 FMU, and normal arming and mission start were
confirmed. These results come from a single pair of runs using the settings
above, which assume a 25 kHz resonance and coupling to roll.

| Metric | 0 Pa control | 20 Pa sweep |
|---|---:|---:|
| Maximum recorded false rate | 0 rad/s | 18.0004 rad/s |
| Maximum descent speed in the comparison window | 0.176 m/s | 8.831 m/s |
| Maximum horizontal displacement in the comparison window | 2.118 m | 120.656 m |
| Ground contact before planned landing | None | 13.368 s after sweep start |
| PX4 roll attitude failure message | Absent | Present |

The control run landed and disarmed normally after the 50-second loiter. The
exposure run showed abnormal descent and ground contact after a large false
rate near 25 kHz. Both runs maintained finite state values until the end of
the observation period and completed without FMU integration failures. These
results do not establish calibration of the physical coefficients against
measurements or a statistical crash probability.

Local run artifacts are in `out/acoustic-sweep/{baseline,exposure,report}/`.
`report/summary.json` stores the metrics and the hash of the FMU used in the
runs. `report/provenance.json` stores hashes of the ELF, configurations, and
response source code, along with repository commits.


## Full 3D / IMU / flight-state comparison (reruns with additional instrumentation)

In `configs/px4_fmu_v6c_acoustic_*.toml`, `trace_period_s=0.001` specifies the
recording interval. The physical and sensor exposure conditions are unchanged.
The current FMU's `truth.p_w`, `truth.v_w`, `truth.q_wb`, and `truth.omega_b` are
read using their actual value references from the packaged XML. Older FMUs
without these fields can still run, but report `truth_valid=0`. The full-state
analyzer requires a new recording rather than estimating the missing actual
state.

```bash
python virtuals/gyro_acoustic/tools/plot_flight.py \
  out/acoustic-sweep-full/baseline out/acoustic-sweep-full/exposure \
  --output out/acoustic-sweep-full/report
```

The two run directories contain separate reruns completed for this work. The
earlier 100 Hz results are preserved in `out/acoustic-sweep/`. Differences in
host-time startup timing for the mission helper and MAVLink mean that
trajectories and contact times are not identical across runs with the same
settings. In these reruns, contact occurred **19.035 s** after sweep start.
This does not replace the original result of 13.368 s and must not be treated
as a reanalysis of the same run.

The output files are listed below. Each static plot is provided in both PNG
and PDF formats.

| File | Contents |
|---|---|
| `trajectory_3d.html` | Full 3D trajectories with offline rotation and zoom. Supports mouse dragging, the scroll wheel, and Fit to the selected run. |
| `trajectory_3d.png` | Full 3D trajectories and projections onto three planes, using directly recorded CG positions. Coordinates are North/East/Up, with equal physical scale on all axes. |
| `imu_acceleration.png` | Body-FRD specific force x/y/z [m/s²] at the FMU→PX4 boundary. |
| `imu_angular_velocity.png` | Gyro x/y/z [deg/s] after injection, compared with clean gyro values before injection shown as dashed lines. |
| `velocity.png` | World-NED velocity [m/s]: actual FMU values and PX4 estimates. Down is positive. |
| `attitude.png` | Body-FRD→world-NED roll/pitch/yaw [deg]: converted FMU quaternions and PX4 estimates. |
| `body_rate.png` | Actual vehicle p/q/r [deg/s] and filtered rates from PX4 ATTITUDE. These are not Euler angle derivatives. |
| `*_signals.csv` | All FMU samples. Position, velocity, and IMU values use SI units; Euler angles are in deg. |
| `*_px4_*.csv` | PX4 estimates and filtered outputs. Units are specified in each CSV header. |

Time plots align each run's sweep start to zero. The left panels show the full
recording. The right panels show either the 25 kHz crossing for IMU/rate signals
(8–12 s) or the flight departure interval for velocity/attitude (-3–25 s).
Shading marks the exposure period, gray dashed lines mark the 25 kHz crossing,
and magenta dashed lines mark the first ground contact before the planned
landing.

Blue denotes the control run and orange denotes the exposure run. In the
velocity/attitude/body-rate plots, solid lines show FMU values and dashed lines
show PX4 values. The time_boot_ms timestamps in PX4 ATTITUDE and
LOCAL_POSITION_NED come from the same QEMU virtual clock as the FMU via
`hrt_micros64`. Reception wall time is not used as state time. The MAVLink
estimates arrive at low rates of approximately 4 Hz/1 Hz, respectively, and do
not represent high-rate sensor output or capture every instantaneous peak.
SIH HIL_STATE_QUATERNION messages are excluded because they do not represent
this FMU's actual state.

IMU acceleration is specific force, not gravity-removed world acceleration.
A stationary, level FRD vehicle reads approximately [0,0,-9.81] m/s². Recording
takes place at the FMU sensor boundary immediately before PX4 uORB publication;
it excludes the bridge's small anti-stale dither (acceleration ±0.0001 m/s²,
gyro ±0.00001 rad/s) and PX4's internal filters. Injection affects only the
gyro, so acceleration changes are physical consequences of attitude, thrust,
and ground contact. The current I0 sensors do not apply full-scale clipping,
so very large accelerations and angular velocities occur at impact. These
values must not be interpreted as the output range of a real IMU or as
predictions of structural impact loads.

Euler angles are displayed within ±180°, with lines broken at wrap boundaries.
Near pitch ±90°, the Euler representation is not unique, so also inspect the
original quaternions and body rates. Differences between actual and estimated
yaw are observed in the control run as well, so not all attitude errors should
be attributed to acoustic injection. This comparison records behavior under
the existing controller and sensor assumptions.

## Rumoca acoustic replay

The acoustic replay uses Rumoca's actual quadrotor scene and local assets to
animate the two recorded flights side by side. It supports play/pause, speed,
scrubbing, synchronized camera orbit/zoom, trajectory history, full/common
recording ranges and jumps to exposure onset, model resonance, exposure end
and early ground contact. It displays FMU truth poses, acoustic frequency and
pressure, injected rates, PWM and flight-state plots. PX4 attitude and filtered
rate overlays are optional and are visually separate from FMU truth.

From the FastDyn root, using the existing environment:

```bash
fastdyn-env/bin/python virtuals/gyro_acoustic/tools/replay.py export \
  out/acoustic-sweep-full/baseline out/acoustic-sweep-full/exposure \
  --output out/acoustic-sweep-full/replay/replay.json

fastdyn-env/bin/python virtuals/gyro_acoustic/tools/replay.py serve \
  --replay out/acoustic-sweep-full/replay/replay.json --port 8080
```

Open <http://127.0.0.1:8080/>. Export refuses to overwrite an existing file;
choose a new output name when regenerating a recording. The server only binds
to localhost and exposes the selected replay and viewer assets. Stop it with
Ctrl-C. No QEMU run, FMU rebuild, new Rumoca simulation or CDN is needed.
NumPy is required for export; pymavlink is optional for reading `mav.tlog`.
Use `export --no-telemetry` to omit PX4 overlays. Missing telemetry is reported
in the page, and the actual flight remains viewable. The server itself uses
Python's standard library and the installed FastDyn path helper. An alternate
Rumoca checkout can be selected with `serve --rumoca-root PATH`.

The converter requires the full-state `trace.csv` described above. It reuses
the flight plotting loader, rejects invalid truth and mismatched exposure
settings, and checks equal FMU hashes when both run-local binaries exist.
The original CSV and settings are not modified. Each export includes their
hashes, the converter hash, sample counts and full-resolution summary metrics.

Replay time is relative to each run's recorded sweep onset. Each pane also
shows the original FMU time: `sim_time_s = replay_time + sweep_start_sim_s`.
The default comparison interval is -3 s through 10 s after exposure ends;
the full view uses the intersection of the two recordings after onset alignment.
This aligns an event, not firmware boot or mission scheduling, and does not
stretch time. Position is converted from NED to NWU; body-FRD → world-NED
quaternions are converted to body-FLU → world-NWU as `[w,x,-y,-z]`.
IMU, Euler-angle and body-rate plots retain their documented FRD/NED conventions.

Poses are selected at 50 Hz by default (`export --hz 10..200`), retaining
endpoints, exposure events, false-rate extrema and gap boundaries. Position
interpolation is linear and orientation uses shortest-path SLERP. No pose is
extrapolated or interpolated across a gap longer than 0.25 s. Sensor readouts
use recorded samples rather than interpolated acoustic signals. Playback plots
are a subsampled overview, not a reconstruction of high-frequency gyro signals;
all summary metrics are computed from the original samples before subsampling.
PX4 dashed plots break at gaps longer than 0.75 s and Euler wrap boundaries.
Metrics cover exposure onset through 10 s after exposure ends; contact uses
the existing 0.3 m altitude threshold and is not a structural damage estimate.

The scene is reused without changing the Rumoca submodule. The display mesh
is scaled 3x, while trajectory coordinates remain in meters. PWM is displayed
as PWM, not converted into unmeasured rotor RPM or arming state. Source and
asset attribution is in `tools/replay_web/CREDITS.md` under `virtuals/gyro_acoustic`.

```bash
fastdyn-env/bin/python -m pytest -q tests/unit/test_acoustic_replay.py \
  tests/unit/test_acoustic_plotting.py
node --test virtuals/gyro_acoustic/tools/replay_web/math.test.js
```

An optional Chromium check exercises the real scene, playback, seeking, camera
and range controls, checks that all assets load locally, and saves a screenshot:

```bash
# Optional test environment only; not required to serve or view the replay.
fastdyn-env/bin/python -m pip install playwright
fastdyn-env/bin/python -m playwright install chromium
fastdyn-env/bin/python tests/integration/acoustic_replay_browser.py \
  --url http://127.0.0.1:8080/ \
  --output out/acoustic-sweep-full/replay/comparison.png
```

This browser check targets the documented 23–27 kHz paired example and its
recorded early-contact event. Unit tests cover quaternion frame conversion,
off-grid injection peaks, onset alignment, missing-data interpolation,
invalid/mismatched recordings and server asset path boundaries.
