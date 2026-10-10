#!/usr/bin/env bash
# Run the FMU-backed rover rectangle and produce measured 2D/3D plots.
set -Eeuo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
if [[ -f fastdyn-env/bin/activate ]]; then
    source fastdyn-env/bin/activate
fi
run_dir="${1:-out/rover-trajectory/$(date -u +%Y%m%dT%H%M%SZ)}"
if [[ -e "$run_dir" ]]; then
    printf 'Output already exists; choose a new directory: %s\n' "$run_dir" >&2
    exit 1
fi
# Default vehicle endpoints are shared; check before starting the local simulator.
python3 - <<'PY'
import os
import socket
sockets = []
try:
    for kind, port in [(socket.SOCK_DGRAM, int(os.getenv('FASTDYN_MAVLINK_FIRMWARE_PORT', '14551'))),
                       (socket.SOCK_DGRAM, int(os.getenv('FASTDYN_MAVLINK_GCS_PORT', '14552'))),
                       (socket.SOCK_STREAM, 5565)]:
        sock = socket.socket(socket.AF_INET, kind)
        sockets.append(sock)
        sock.bind(('127.0.0.1', port))
except OSError as exc:
    raise SystemExit(f'Simulation port already in use: {port}: {exc}')
finally:
    for sock in sockets:
        sock.close()
PY
mkdir -p "$run_dir"
run_dir="$(cd -- "$run_dir" && pwd)"
export FASTDYN_MAVLINK_LOG="$run_dir/mission.tlog"
cp configs/rover462_trajectory.toml "$run_dir/config.toml"
cp virtuals/physics/flight_controllers/courbet/mavlink/rover_rectangle.txt "$run_dir/mission.waypoints"
cp virtuals/physics/flight_controllers/courbet/mavlink/rover_fmu.param "$run_dir/rover_fmu.param"
printf 'Running rover; console log: %s/console.log\n' "$run_dir"
timeout --signal=INT --kill-after=15s 420s fastdyn run \
    -c configs/rover462_trajectory.toml -o "$run_dir/work" > "$run_dir/console.log" 2>&1
# The helper exit code is not propagated by every FastDyn version.
if ! rg -q '\[mission\] final waypoint reached' "$run_dir/console.log"; then
    tail -60 "$run_dir/console.log" >&2
    printf 'Mission completion was not confirmed. See console.log.\n' >&2
    exit 1
fi
python3 utils/plot_rover_trajectory.py --log "$run_dir/mission.tlog" \
    --mission "$run_dir/mission.waypoints" --output "$run_dir/trajectory" --require-complete
printf 'Saved trajectory plots and CSV/JSON: %s\n' "$run_dir"
