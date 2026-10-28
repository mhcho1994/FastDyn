"""Configure a reduced acoustic gyro response at the FMU sensor boundary."""

from __future__ import annotations

import json
import math

from fastdyn.virtual_preprocessing import (
    ConfigurationHelp, RunContext, RunDefinition, RunPrepareResult,
    VirtualPreparationError, register_run_preprocessor,
)

# Ordered wire format shared with the native model. Timing uses the FMU clock.
DEFAULTS = {
    "rms_pressure_pa": 20.0,
    "start_frequency_hz": 23000.0,
    "end_frequency_hz": 27000.0,
    "duration_s": 20.0,
    "airborne_altitude_m": 5.0,
    "delay_after_airborne_s": 3.0,
    "ramp_s": 0.25,
    "phase_rad": 0.0,
    "effective_area_m2": 1e-10,
    "mems_mass_kg": 2.5e-9,
    "drive_amplitude_m": 2e-7,
    "reference_frequency_hz": 25000.0,
    "sense_frequency_hz": 25000.0,
    "sense_q": 30.0,
    "afe_tau_s": 1e-5,
    "demod_tau_s": 0.003,
    "integration_step_s": 1e-5,
    "trace_period_s": 0.01,
}
NONNEGATIVE = {"rms_pressure_pa", "delay_after_airborne_s", "ramp_s"}


def resolve_settings(settings: dict[str, object]) -> dict[str, object]:
    """Validate the exposure and illustrative single-sense-mode device profile."""
    label = "[CPU.cpu0.plugins.gyro_acoustic]"
    unknown = set(settings) - set(DEFAULTS) - {"enabled", "axis_coupling"}
    if unknown:
        raise VirtualPreparationError(f"{label}: unknown keys: {sorted(unknown)}")
    if not isinstance(settings.get("enabled", False), bool):
        raise VirtualPreparationError(f"{label}.enabled must be boolean")
    result = {}
    for name, default in DEFAULTS.items():
        value = settings.get(name, default)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise VirtualPreparationError(f"{label}.{name} must be finite numeric")
        if name != "phase_rad" and (value < 0 if name in NONNEGATIVE else value <= 0):
            raise VirtualPreparationError(f"{label}.{name} is outside its positive/nonnegative range")
        result[name] = float(value)
    axes = settings.get("axis_coupling", [1.0, 0.0, 0.0])
    if not isinstance(axes, list) or len(axes) != 3 or any(
        isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in axes
    ):
        raise VirtualPreparationError(f"{label}.axis_coupling must contain three finite FRD gains")
    result["axis_coupling"] = [float(v) for v in axes]
    if result["ramp_s"] * 2 > result["duration_s"]:
        raise VirtualPreparationError(f"{label}.ramp_s must be <= duration_s/2")
    beat = max(abs(result[k] - result["reference_frequency_hz"])
               for k in ("start_frequency_hz", "end_frequency_hz"))
    max_step = min(result["demod_tau_s"] / 20, 1 / (40 * max(1, beat)))
    if not 1e-7 <= result["integration_step_s"] <= max_step:
        raise VirtualPreparationError(
            f"{label}.integration_step_s must be in [1e-7, {max_step:g}] to resolve baseband"
        )
    return result


class AcousticPreprocessor:
    """Serialize validated settings into this plugin's run artifacts."""

    def prepare(self, ctx: RunContext) -> RunPrepareResult:
        settings = resolve_settings(dict(ctx.settings))
        config = ctx.plugin_artifact_path("response.txt")
        values = [settings[key] for key in DEFAULTS] + settings["axis_coupling"]
        config.write_text("1\n" + "\n".join(format(v, ".17g") for v in values) + "\n",
                          encoding="utf-8")
        manifest = ctx.plugin_artifact_path("settings.json")
        manifest.write_text(json.dumps(settings, indent=2) + "\n", encoding="utf-8")
        return RunPrepareResult(artifacts=[config, manifest])


register_run_preprocessor(RunDefinition(
    name="gyro_acoustic", prepare=AcousticPreprocessor(),
    enabled=lambda ctx: ctx.settings.get("enabled", False) is not False,
    help=ConfigurationHelp(
        "Sweep acoustic exposure through a reduced gyro response; record FMU truth and false rate.",
        '[CPU.cpu0.plugins.gyro_acoustic]\nenabled = true\nrms_pressure_pa = 20.0\n'
        'start_frequency_hz = 23000.0\nend_frequency_hz = 27000.0\nduration_s = 20.0',
        "docs/AcousticSweep.md"),
))
