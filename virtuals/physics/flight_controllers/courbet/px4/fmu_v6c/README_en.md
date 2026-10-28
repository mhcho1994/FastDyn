# PX4 FMUv6C + QEMU + FastDyn P2

**English** | [한국어](README_ko.md)

This ARM/NuttX profile runs the user-provided
`courbet/bin/px4_fmu-v6c_default.elf` on a Cortex-M7. It does not substitute a
host PX4 SITL executable. The inspected ELF contains PX4 commit
`0b6e4687defb353a34201951809efd3f0040a9ba` (`v1.17.0-rc2`), which is also
reported by `ver all` at runtime.

## Running the profile

See [AcousticSweep.md](../../../../../../docs/AcousticSweep.md) for configurations
that compare false rates and flight effects during a 23–27 kHz acoustic sweep.
The guide includes a 0 Pa control, an illustrative 20 Pa exposure, FMU state
traces, and analysis commands.

From the FastDyn directory, run with the prepared ELF address configuration
and plugin:

```bash
fastdyn run -c configs/px4_fmu_v6c_fire_p2.toml
```

The `boot.nsh` file specified by `[Run].bridge_boot` in the TOML is executed
automatically; there is no need to export `FASTDYN_BRIDGE_BOOT` beforehand.
Commands are sent one at a time at each NSH prompt to start the sensor
publication modules, EKF2, PWM output, commander, and multicopter controllers.
MAVProxy/MAVCesium and the PX4 mission helper then start automatically, upload
the default mission, and perform **normal arming → takeoff → waypoint flight →
landing → disarm**. Open `http://127.0.0.1:5000/mavcesium/` in a browser to view
position and attitude. The run session and helpers exit when the mission
completes. After `FASTDYN_PX4_READY`, commands such as `ver all`,
`listener sensor_accel -n 1`, and `ekf2 status` can be entered in the terminal.
Press Ctrl-C to stop the run. The `boot.nsh` file itself does not request
arming; the separate host mission helper handles automatic arming.
`FASTDYN_PX4_READY` indicates that boot command delivery has completed, not
that the EKF is ready or arming has succeeded.

Use the existing script to inspect ELF symbols, DWARF ABI information, and
instructions, regenerate `unlabeled_conf`, build the plugin, and run:

```bash
./virtuals/physics/flight_controllers/courbet/px4/fmu_v6c/run.sh -o out/px4-fmu-v6c
```

This script runs the shared configuration generator and `ninja -C build`.
It uses Python and the CLI from `fastdyn-env/bin` by default. Override their
paths with `FASTDYN_PYTHON` and `FASTDYN_CLI` if needed. GDB with Python support
and pyelftools are required. If the FMU is already built, add `--no-build-fmu`
to the run arguments.

To open only the NuttX console without starting modules automatically, override
the default with an empty environment variable:

```bash
FASTDYN_BRIDGE_BOOT= fastdyn run -c configs/px4_fmu_v6c_fire_p2.toml --no-run-processes
# Or use the existing option in the build/configuration generation script:
./virtuals/physics/flight_controllers/courbet/px4/fmu_v6c/run.sh --shell -o out/px4-shell
```

To use a different NSH command file, set
`FASTDYN_BRIDGE_BOOT=/absolute/path/custom.nsh`. The environment variable takes
precedence over the TOML, and a relative path in the environment variable is
resolved from the directory where the command is run. Relative paths in the
TOML are resolved from the FastDyn repository root. The boot file setting
applies only to the current run session and does not persist into subsequent
profiles. `--no-run-processes` disables only the host helpers; automatic PX4
initialization remains enabled. Commands must be no longer than 126 characters
per line and must end with a newline.

## Automatic mission and arming

The default run performs the following steps automatically:

1. Start RAM mission storage with `dataman start -r` and the mission executor
   with `navigator`. The boot file also starts the sensors, EKF, commander,
   multicopter controllers, and TELEM1 MAVLink.
2. MAVProxy sends GCS heartbeats to the UART bridge and forwards PX4 responses
   to the helper and viewer.
3. `mav_command_control_px4.py` verifies the PX4 heartbeat and uploads the
   parameters and mission. It removes the home row from the default QGC file
   and sends the 5 actual flight items.
4. Wait for a GPS fix, global position, and EKF validity messages.
5. Request AUTO.MISSION with `MAV_CMD_DO_SET_MODE` and verify the PX4
   main/submode in the heartbeat.
6. Request normal arming with `MAV_CMD_COMPONENT_ARM_DISARM(1, 0)` and verify
   the armed bit in the heartbeat.
7. Verify an accepted ACK for `MAV_CMD_MISSION_START`. The run exits
   successfully only after flight, the final mission item, landing, and
   disarm have all been confirmed.

**Switching to AUTO.MISSION alone does not arm the vehicle.** The PX4 Commander
in this ELF also attempts a mode switch and normal arming while handling
`MAV_CMD_MISSION_START` for a valid mission. The host helper explicitly requests
arming first to verify the state transitions clearly. It does not use the
force-arm value `21196` or bypass arming checks. Temporary rejections are
retried within the timeout. Failures in readiness, upload, arming, or mission
completion are reported as errors and terminate the run session. Specific
rejection reasons appear in the console and MAVLink STATUSTEXT messages.

To start only the modules and UART for diagnostics, without an automatic mission:

```bash
fastdyn run -c configs/px4_fmu_v6c_fire_p2.toml --no-run-processes
```

This also leaves MAVProxy and the viewer disabled. From another terminal, you
can connect directly to the bridge to test only the upload. `--no-arm` does
not request a mode change, arming, or mission start.

```bash
fastdyn-env/bin/python virtuals/physics/flight_controllers/courbet/mavlink/mav_command_control_px4.py \
  --connect udpout:127.0.0.1:14551 --no-arm \
  virtuals/physics/flight_controllers/courbet/mavlink/px4_init.param \
  virtuals/physics/flight_controllers/courbet/mavlink/copter_mission.waypoints
```

Override the default mission and parameter files with `FASTDYN_MISSION_FILE`
and `FASTDYN_PARAM_FILE`. Since this automatic mission setup has no RC input,
`COM_RC_IN_MODE=4` is set during boot. SD card checks and logging settings are
also adjusted for this virtual board, but the actual PX4 code still determines
EKF validity and normal arming eligibility.

## TELEM1 UART and host UDP

```text
PX4 commander / navigator / uORB
             ↕ actual PX4 mavlink module
NuttX TELEM1: /dev/ttyS5 (UART7, g_uart7priv)
             ↕ FastDyn UART adapter, original MAVLink bytes
Host UDP 127.0.0.1:14551
             ↕ MAVProxy --master=udpout:127.0.0.1:14551
             ├─ --out=udpout:127.0.0.1:14552 ↔ PX4 mission helper (udpin)
             └─ fastdyn_cesium → HTTP/WebSocket :5000 → browser
```

| Environment variable | Default port | Role |
|---|---:|---|
| `FASTDYN_MAVLINK_FIRMWARE_PORT` | UDP 14551 | QEMU host adapter binds to loopback |
| `FASTDYN_MAVLINK_GCS_PORT` | UDP 14552 | Mission helper binds; MAVProxy forwards in both directions |
| `FASTDYN_MAVCESIUM_PORT` | TCP 5000 | MAVCesium web page and WebSocket |

The boot commands are `mavlink start -d /dev/ttyS5 -b 921600 -m 0 -r 100000 -Z`
and `mavlink boot_complete`. The current ELF does not accept the string
`normal` for Normal mode, so the numeric value `0` is used. `-Z` disables
hardware flow control, which is not modeled. The 921600 baud setting applies
to the guest UART; the adapter does not reproduce actual serial transmission
timing.

On transmit, the file → inode → device pointer chain in `uart_write`
distinguishes the NSH console from TELEM1. Console output (USART3) continues
to go to the terminal, while only TELEM1 bytes are forwarded over UDP. On
receive, the nonblocking UDP buffer is fed into the HRT UART polling path,
which executes the actual NuttX `uart_recvchars`. The guest UART receive
queue, semaphore, and poll mechanism wake the PX4 MAVLink receive thread, so
PX4's parameter, mission, and command handling code actually runs. The host
does not generate synthetic heartbeats or acceptance ACKs.

The UDP socket is created when TELEM1 is opened. Replies go to the source
address of the first received packet, so a heartbeat from MAVProxy or a
directly connected `udpout` client is required. UART transmissions before
the first reception are discarded. The bridge uses one current UDP peer;
use MAVProxy to distribute traffic to multiple clients. Port conflicts and
invalid ports cause explicit failures, and the socket closes when the QEMU
process exits. The default QEMU monitor TCP port is 4444. The mission helper's
shutdown watcher uses this port to locate the QEMU process and shut it down
cleanly. MAVProxy's `mav.tlog`, `mav.tlog.raw`, and `mav.parm` are saved in the
run output directory.

The viewer's `GLOBAL_POSITION_INT` and `ATTITUDE` messages contain PX4 estimates
derived from FMU sensor feedback. Motor outputs therefore drive the FMU,
and the results pass through the sensors and EKF before appearing in the
viewer. Loading map and Cesium assets in the browser may require external
network access.

## Bridge path

```
PX4 ARM ELF / NuttX
  ├─ QEMU virtual time → hrt_absolute_time, NuttX SysTick
  ├─ TIM8 HRT IRQ 62 (1 ms) ← advance the FMU to the same virtual time
  ├─ FMU accel/gyro/mag/baro/GPS → uORB → sensor processing / EKF2 / controllers
  └─ actuator_outputs_sim → 4-channel PWM → FMU P2
```

The implementation resides in the existing
`flight_controllers/ardupilot/ardupilot.c`; no `px4.c` was added. Timekeeping,
the `phy_*` physics API, and FMI execution are shared, while adapters handle
firmware-specific calling conventions and data structures. The existing
ArduPilot hooks' `R0=this` convention, C++ object offsets, and SPI data
representations cannot be applied unchanged to PX4 functions.

This profile starts the ELF's `simulator_sih` and `sensor_*_sim` modules as
sensor publishers. At entry to `uORB::DeviceNode::publish`, the published
data is replaced with FMU values. The actual uORB distribution, NuttX
scheduler, sensor processing, EKF2, and controller code still execute.
SIH's own dynamics also run, but IMU, barometric, magnetic-field, and GPS
feedback is replaced with FMU values. **SIH's `*_groundtruth` topics do not
represent FMU state and must not be used for validation.** The SIH distance
sensor is disabled because it is not connected to the FMU.

Sensor units are FRD acceleration in m/s², angular velocity in rad/s,
magnetic field in gauss, pressure in Pa, temperature in °C, GPS latitude and
longitude in deg, and altitude in m. Because the FMU has no geoid model,
GPS MSL and ellipsoid altitudes are reported with the same value. Dither
from a deterministic hash sequence is added to prevent stationary ideal
sensor outputs from being marked STALE by PX4's repeated-value detection:
up to ±0.0001 m/s² for acceleration, ±0.00001 rad/s for angular velocity,
±0.00001 gauss for magnetic field, and ±0.1 Pa for pressure. This is not a
calibrated physical sensor noise model. A simple alternating +/- sequence
is not used because VehicleIMU integration cancels it out, causing gyro
STALE detection to recur. With this issue fixed, EKF inputs remain active
even during prolonged stationary operation.

The motor order is PX4 Quad-X: front right / rear left / front left / rear
right. This matches the channel order in the current `quad_p2.toml`.
Normalized outputs of 0–1 are mapped to this FMU's PWM range of 1100–1900.
NaN or missing channels are set to 1000 to stop the motor. If an FMU uses a
different motor order or PWM range, the adapter must be adjusted accordingly.

## Configuration files and hardware scope

- `labeled_conf`: the source list of hooks and register modifications,
  expressed using symbol names.
- `profile.toml`: PX4 rules for the shared generator, the ELF path, the
  FMUv6C vector address, and HRT/SysTick instruction checks. Paths are
  resolved relative to this file.
- `../../prepare_execution_configuration.py`: the shared CLI. ELF address
  conversion and output are handled by `../../execution_configuration`;
  NuttX/uORB ABI checks are handled by `../../profiles/px4`. If a required
  symbol, instruction, or structure check fails, generation stops and
  preserves the existing generated files.
- `unlabeled_conf`: generated address configuration, ELF SHA-256, and
  diagnostic `symbols.txt`.
- `boot.nsh`: NSH commands that start modules included in the ARM firmware.
- `configs/px4_fmu_v6c_fire_p2.toml`: M7, vector `0x08020000`, H743 RAM banks,
  and the P2 FMU.

The `create_new_conf.sh` script is a template generator that copies Rover
configuration; it does not translate PX4 addresses or ABI information.
The `run.sh` script passes this directory's `profile.toml` to the shared
generator. To generate only the configuration, run from the FastDyn root:

```bash
fastdyn-env/bin/python virtuals/physics/flight_controllers/courbet/prepare_execution_configuration.py \
  --profile virtuals/physics/flight_controllers/courbet/px4/fmu_v6c/profile.toml
```

The former `prepare.py` has been consolidated into the shared tool. Always
rerun generation after changing the ELF. Other firmware versions or boards
require a review of the hooks and ABI. Use `--elf` to select the ELF to
inspect and `--output-dir` to choose a separate generation directory.
Both option paths are resolved from the current working directory. These
options do not change the binary path in the FastDyn run TOML.

This configuration is **not a complete Pixhawk6C/STM32H743 peripheral
emulator**. On the generic `cortexm` machine, hooks bypass PLL setup, physical
UART/SPI/I2C sensor initialization, USB power readiness, and SD/MMC/MTD waits.
Testing sensor drivers at the register level requires additional peripheral
models. The NSH console and TELEM1 UART are supported; other UARTs are not
yet connected. UART device and uORB structure offsets are validated against
the ELF's DWARF information. See the
[shared README](../../README.md#flight-stack-specific-mission-helpers) for
helper usage and differences from the ArduPilot helper.

NuttX SysTick runs at 1 ms. Since the current QEMU `cortexm` system clock is
168 MHz, the ELF's reload value for 480 MHz is patched to 167999. TIM8 HRT
is serviced at 1 ms intervals. `icount` provides a virtual time base; it
does not guarantee cycle accuracy for a real H743. If the QEMU clock changes,
the SysTick patch must be updated as well.

There is no persistent parameter store, so automatic save requests are
skipped. Settings are applied from `boot.nsh` on every run. SD logging is
not used, and the SD card check is disabled. Early hardware ID read failure
messages occur because the ID peripheral is absent. Mission storage is also
in RAM, so parameters and missions must be reapplied after a restart.

## Validation

```bash
fastdyn-env/bin/python virtuals/physics/flight_controllers/courbet/tests/test_execution_configuration.py
fastdyn-env/bin/python virtuals/physics/flight_controllers/courbet/tests/smoke_px4_fmu_v6c.py --auto-boot
fastdyn-env/bin/python virtuals/physics/flight_controllers/courbet/tests/smoke_px4_fmu_v6c.py
fastdyn-env/bin/python virtuals/physics/flight_controllers/courbet/tests/smoke_px4_mission.py
```

The first test checks ArduPlane/PX4 address generation, path resolution,
missing or duplicate symbols, ABI and instruction changes, ELF hash
validation, CFI flags, and preservation of existing output on failure.
The `--auto-boot` test starts actual QEMU through a direct `fastdyn run`
without setting the boot environment variable, then verifies the NuttX/PX4
versions, completion of automatic initialization, and six types of bridge
connections. The FMUv6C smoke test without that option uses a custom boot
file to check the GPS position and the motor test's 1100 → 1180 → 1100
transition. Motor tests run only inside the emulator. Logs are written to
`out/px4-smoke-*/console.log`. The integration tests require a built P2 FMU.

The `smoke_px4_mission.py` test runs the default TOML directly with the
default boot file. Through the actual UART↔UDP↔MAVProxy path, it verifies
INT32/REAL32 parameter round trips, upload of the 5 mission items, normal
arming, takeoff, movement, landing, and disarm. It also checks the MAVCesium
HTTP response and receives position and attitude streams over WebSocket,
verifying an altitude of at least 10 m and horizontal movement of at least
5 m. The default ports are reserved for testing at 19551/19552/19553 and
can be changed with `--base-port`. Console logs, received viewer data
(`viewer.jsonl`), and measurement summaries are saved in `out/px4-mission-*/`.
These tests do not replace checks of map rendering in a browser or flight
validation on a real vehicle.
