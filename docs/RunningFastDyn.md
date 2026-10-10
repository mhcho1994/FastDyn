# Running FastDyn

## Slide: Run a firmware from one configuration

**FastDyn turns a firmware ELF and one TOML configuration into a QEMU run.**

```bash
fastdyn run -c configs/target.toml -o fastdyn_work
```

- `-c` / `--config` is required: it selects the board, CPU, memory, firmware,
  device routing, virtuals, and enabled run-wide plugins.
- `-o` / `--work-dir` selects where FastDyn writes run artifacts. It defaults to
  `./fastdyn_work`; give each experiment its own directory.
- FastDyn prepares enabled plugins, resolves symbols and trigger addresses,
  writes generated virtual/modifier rules, prepares RAM backing, builds a
  configured FMU when applicable, then launches QEMU.

## Slide: A practical first run

```bash
# Optional: derive a reviewed starter from an ELF.
./fastdyn-env/bin/python tools/elf2config/elf_to_config.py firmware.elf \
  --output configs/firmware.toml

# Browse or refine the configuration.
fastdyn help

# Run it and keep all artifacts together.
fastdyn run -c configs/firmware.toml -o fastdyn_work
```

Before launching, make sure the TOML has the correct QEMU path, CPU target,
RAM map, initial vector address for Cortex-M firmware, and peripheral routing.
The generated configuration uses `classic` for the conventional Cortex-M MMIO
window as a starting point; it must be reviewed for the actual board.

## Slide: Useful run options

```bash
# Retain a prior work directory rather than resetting it.
fastdyn run -c configs/target.toml -o fastdyn_work --persist-work-dir

# Use an explicit SVD file or SVD catalog when resolving [Machine].platform.
fastdyn run -c configs/target.toml -s third_party/common/cmsis-svd-data

# Override the active FMU, or skip its automatic build.
fastdyn run -c configs/target.toml --fmu quadrotor
fastdyn run -c configs/target.toml --no-build-fmu
```

Use `fastdyn run --help` for every option. `fastdyn help run` provides the
same primary workflow in the interactive configuration-help menu.

## Slide: What to inspect after a run

The work directory is the run record. Depending on the configuration, it
contains:

- `virtuals/virtuals.txt` and `virtuals/modifiers.txt`: resolved runtime rules;
- `run-artifacts/`: generated plugin data such as introspection schemas;
- logs, QEMU output, RAM backing, and timing data;
- plugin-specific results such as FunctionCounter counts or VariableWatch logs.

If the run does not boot, begin with the CPU/machine selection, memory map,
ELF/vector address, QEMU path, and peripheral routing. Use `fastdyn help` to
find valid configuration values without memorizing them.

## Copter trajectory and flight-state plots

Run from the FastDyn directory with `fastdyn-env` activated:

```bash
fastdyn run -c configs/copter462.toml -o out/fmi3
python utils/plot_copter_trajectory.py \
  --log out/fmi3/mission.tlog \
  --output out/fmi3/plots/copter
```

The Copter configuration records MAVLink telemetry in the run directory. Plot
only after the mission has finished recording. The script saves two figures,
each in PNG, PDF, and SVG formats:

- `copter_trajectory_3d`: measured North/East/altitude trajectory, numbered mission
  waypoints, and green/red quadrotors at the first/last plotted samples. The
  enlarged quadrotor geometry follows the recorded roll, pitch, and yaw.
- `copter_dynamics`: three subplots for NED velocity and 3D speed (m/s),
  roll/pitch/yaw (degrees), and body angular rates p/q/r (degrees/s).

Time starts at the first position sample after an armed AUTO heartbeat. Altitude
is relative to the firmware home. Yaw is unwrapped to avoid jumps at ±180 degrees;
its original wrapped values are also saved in `copter_attitude.csv`. The angular
rates are body-axis gyro rates, not Euler-angle derivatives. Position and attitude
CSV files preserve their individual telemetry timestamps; the summary JSON records
units, frames, the actual plotted time range, and whether a disarm was observed.
The final icon marks the last recorded position, which may precede touchdown.

Select a shorter recorded interval with `--start 30 --end 60`. To plot a different
mission, pass its QGC waypoint file with `--mission path/to/mission.waypoints`.
Use the waypoint file belonging to the recording. A requested end time beyond
the available telemetry is rejected rather than extrapolated.
