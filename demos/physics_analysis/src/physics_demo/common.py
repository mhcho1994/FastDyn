from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import subprocess
import tomllib

DEMO = Path(__file__).resolve().parents[2]
ROOT = DEMO.parents[1]
OUT = ROOT / "out/physics_analysis"
RUMOCA = ROOT / "third_party/common/rumoca"


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def revision(path):
    return subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()


def scenario(name):
    path = Path(name)
    if not path.is_file():
        path = DEMO / "scenarios" / f"{name}.toml"
    with (DEMO / "scenarios/_mission.toml").open("rb") as f:
        value = tomllib.load(f)
    with path.open("rb") as f:
        value.update(tomllib.load(f))
    validate_scenario(value)
    return value


ATTACK_PARAMETERS = {
    "none": set(),
    "magnetic": {"magnetic_scale"},
    "wind": {"wind_mean_mps", "wind_gust_mps", "wind_from_deg", "wind_frequency_hz", "wind_phase_rad"},
    "gps": {"gps_north_m", "gps_east_m", "gps_up_m"},
    "gyro": {f"gyro_{mode}_{axis}_dps" for mode in ("bias", "false") for axis in ("roll", "pitch", "yaw")} | {"gyro_frequency_hz"},
}


def validate_scenario(value):
    if value.get("kind") != "waypoints":
        raise ValueError("all demo missions must use kind=waypoints")
    for key in ("altitude_m", "duration_s", "hold_s", "acceptance_radius_m", "altitude_tolerance_m"):
        if not math.isfinite(value[key]) or value[key] <= 0:
            raise ValueError(f"{key} must be finite and positive")
    points = value.get("waypoints_ne_m")
    if not points or any(len(p) != 2 or not all(math.isfinite(x) for x in p) for p in points):
        raise ValueError("waypoints_ne_m needs finite [north,east] pairs")
    attack = value.get("attack", {})
    kind = attack.get("kind")
    if kind not in ATTACK_PARAMETERS:
        raise ValueError("attack kind must be none, magnetic, wind, gps or gyro")
    unknown = set(attack) - ATTACK_PARAMETERS[kind] - {"kind", "start_s", "duration_s", "ramp_s"}
    if unknown:
        raise ValueError(f"unexpected parameters for {kind}: {sorted(unknown)}")
    for key, val in attack.items():
        if key != "kind" and (not isinstance(val, (int, float)) or not math.isfinite(val)):
            raise ValueError(f"attack {key} must be finite")
    if attack['start_s'] < 0 or attack['ramp_s'] <= 0 or attack['duration_s'] < 2*attack['ramp_s']:
        raise ValueError("attack needs start >= 0, ramp > 0 and duration >= 2*ramp")
    strengths = {
        'none': [1], 'magnetic': [attack.get('magnetic_scale', 0)],
        'wind': [attack.get('wind_mean_mps', 0), attack.get('wind_gust_mps', 0)],
        'gps': [attack.get(f'gps_{a}_m', 0) for a in ('north','east','up')],
        'gyro': [attack.get(f'gyro_{m}_{a}_dps', 0) for m in ('bias','false') for a in ('roll','pitch','yaw')],
    }
    if not any(strengths[kind]):
        raise ValueError(f'{kind} needs a nonzero injection amplitude')
    for key in ('wind_mean_mps', 'wind_gust_mps', 'wind_frequency_hz', 'gyro_frequency_hz', 'magnetic_scale'):
        if attack.get(key, 0) < 0:
            raise ValueError(f"{key} must be nonnegative")


def attack_parameters(spec):
    validate_scenario(spec)
    return {('attack_' + key if key in {'start_s', 'duration_s', 'ramp_s'} else key): float(val)
            for key, val in spec['attack'].items() if key != 'kind'}


def heading(q):
    """Body FLU to world NWU quaternion wxyz -> compass heading in degrees."""
    w, x, y, z = q
    return -math.degrees(math.atan2(2 * (w*z+x*y), 1-2*(y*y+z*z)))


def wrap_deg(value):
    return (value + 180) % 360 - 180
