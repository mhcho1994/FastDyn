"""Compile and instrument a demo-owned FMU; never patch the core or its FMUs."""
from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

from .common import DEMO, ROOT, OUT, RUMOCA, sha256, revision, write_json

SIGNALS = {
    "vehicle.plant.p": ["north_m", "west_m", "alt_m"],
    "vehicle.plant.q": ["qw", "qx", "qy", "qz"],
    "vehicle.plant.v_w": ["vn_mps", "vw_mps", "vu_mps"],
    "vehicle.plant.omega_m": ["omega1", "omega2", "omega3", "omega4"],
    "pwm": ["pwm1", "pwm2", "pwm3", "pwm4"],
    "attack.wind_nwu": ["wind_n_ms", "wind_w_ms", "wind_u_ms"],
    "attack.gate": ["attack_gate"],
    "attack.model_current_a": ["model_current_a"],
    "attack.mag_delta_frd": ["mag_delta_x_gauss", "mag_delta_y_gauss", "mag_delta_z_gauss"],
    "attack.gps_offset_neu": ["gps_offset_n_m", "gps_offset_e_m", "gps_offset_u_m"],
    "attack.gps_velocity_neu": ["gps_velocity_n_mps", "gps_velocity_e_mps", "gps_velocity_u_mps"],
    "attack.gyro_bias_frd": ["gyro_bias_x_rps", "gyro_bias_y_rps", "gyro_bias_z_rps"],
    "attack.gyro_false_frd": ["gyro_false_x_rps", "gyro_false_y_rps", "gyro_false_z_rps"],
    "vehicle.gyro": ["true_gyro_x_rps", "true_gyro_y_rps", "true_gyro_z_rps"],
    "gyro": ["sensor_gyro_x_rps", "sensor_gyro_y_rps", "sensor_gyro_z_rps"],
}


def instrument(source: Path, metadata: Path):
    original = source.read_text()
    if "PHYSICS_DEMO_OBSERVER" in original:
        raise ValueError("source is already instrumented")
    variables = {v.get("name"): v for v in ET.parse(metadata).findall("ModelVariables/Float64")}
    reads, columns = [], []
    for name, fields in SIGNALS.items():
        v = variables.get(name)
        if v is None:
            raise ValueError(f"truth observer requires FMU variable {name}")
        dims = v.findall("Dimension")
        if not ((not dims and len(fields) == 1) or (len(dims) == 1 and dims[0].get("start") == str(len(fields)))):
            raise ValueError(f"unexpected shape of {name}")
        reads.append(f"vr = {int(v.get('valueReference'))}; "
                     f"if (fmi3GetFloat64(i, &vr, 1, values + {len(columns)}, {len(fields)}) > fmi3Warning) return 0;")
        columns.extend(fields)
    template = (DEMO / "observer.c.in").read_text()
    code = template.replace("@READS@", "\n    ".join(reads))
    code = code.replace("@COUNT@", str(len(columns)))
    code = code.replace("@HEADER@", "sim_time_s," + ",".join(columns) + ",current_a,voltage_v")
    prefix = '/* PHYSICS_DEMO_OBSERVER */\n#include "model.h"\n' + "\n".join(
        f"#undef {name}\n#define {name} demo_original_{name}" for name in ("fmi3DoStep", "fmi3FreeInstance", "fmi3Terminate", "fmi3Reset"))
    source.write_text(prefix + "\n" + original + "\n" + code)


def build(output=None, compiler=None):
    sys.path.insert(0, str(ROOT / "src"))
    from fastdyn.fmu_build import FmuBuild
    from fastdyn.fmi3_package import package_sources

    output = Path(output or OUT / "waypoint-attacks/build").resolve()
    output.mkdir(parents=True, exist_ok=True)
    # Refuse in-place rebuilds: compiled FMUs may be in use by a running process.
    pristine = output / "pristine"
    observed = output / "observed"
    if pristine.exists() or observed.exists():
        raise FileExistsError(f"choose a fresh --output (build already exists): {output}")
    config = FmuBuild("quadrotor", "PhysicsAnalysis.Copter",
                      DEMO / "models/PhysicsAnalysis/Copter.mo", DEMO / "models",
                      (DEMO / "models/PhysicsAnalysis/AttackInputs.mo", ROOT / "modelica/FastDyn/Copter.mo", ROOT / "modelica/FastDyn/QuadrotorWithExternalWrench.mo",
                       ROOT / "third_party/common/modelica_models"),
                      pristine, ROOT)
    binary = Path(compiler) if compiler else RUMOCA / "target/debug/rumoca"
    if not binary.is_file():
        raise FileNotFoundError("build Rumoca first, or supply --compiler /path/to/rumoca")
    cmd = [str(binary.resolve()), "compile", str(config.model_file), "--model", config.model]
    # Explicit FastDyn files avoid ambiguous lazy lookup of the two Copter classes.
    for source in config.extra_source_roots:
        cmd += ["--source-root", str(source)]
    cmd += ["--target", "fmi3", "--output", str(pristine)]
    subprocess.run(cmd, cwd=ROOT, check=True)
    package_sources(config)
    model_dir = config.model.replace(".", "_")
    shutil.copytree(pristine / model_dir, observed / model_dir)
    instrument(observed / model_dir / "sources/model.c", observed / model_dir / "modelDescription.xml")
    package_sources(replace(config, output=observed))
    write_json(output / "build.json", {
        "schema_version": 2, "profile": "waypoint-attacks-v1", "model": config.model,
        "fastdyn_commit": revision(ROOT), "rumoca_commit": revision(RUMOCA),
        "model_sha256": sha256(config.model_file), "compiler_sha256": sha256(binary),
        "core_model_sha256": {str(p.relative_to(ROOT)): sha256(p) for p in config.extra_source_roots if p.is_file()},
        "model_library_commit": revision(ROOT / "third_party/common/modelica_models"),
        "observer_sha256": sha256(DEMO / "observer.c.in"),
        "pristine_fmu_sha256": sha256(config.fmu_path),
        "observed_fmu_sha256": sha256(observed / config.fmu_path.name),
        "model_description_sha256": sha256(observed / model_dir / "modelDescription.xml"),
    })
    print(observed / config.fmu_path.name)
    return output
