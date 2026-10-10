#!/usr/bin/env python3
"""Plot an ArduRover MAVLink recording in local North/East/Up coordinates."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil, mavwp

EARTH_RADIUS_M = 6378137.0


def local_ne(lat, lon, lat0, lon0):
    return (EARTH_RADIUS_M * np.radians(np.asarray(lat) - lat0),
            EARTH_RADIUS_M * math.cos(math.radians(lat0)) * np.radians(np.asarray(lon) - lon0))


def read_track(log, system_id=1):
    """Use firmware time, not accelerated simulation's wall-clock tlog timestamps."""
    mav = mavutil.mavlink_connection(str(log))
    positions, reached = [], []
    armed_auto = False
    start_boot = None
    mission_item = 0
    try:
        while (msg := mav.recv_match()) is not None:
            if msg.get_srcSystem() != system_id:
                continue
            kind = msg.get_type()
            if kind == "HEARTBEAT" and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                armed_auto = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) and mavutil.mode_string_v10(msg) == "AUTO"
            elif kind == "MISSION_CURRENT":
                mission_item = int(msg.seq)
            elif kind == "MISSION_ITEM_REACHED":
                reached.append(int(msg.seq))
            elif kind == "GLOBAL_POSITION_INT" and (msg.lat or msg.lon):
                boot = msg.time_boot_ms / 1000.0
                if positions and boot < positions[-1]["boot_time_s"]:
                    raise ValueError("Firmware clock moved backwards; use a single simulation recording")
                if positions and boot == positions[-1]["boot_time_s"]:
                    continue
                if armed_auto and start_boot is None:
                    start_boot = boot
                positions.append(dict(
                    boot_time_s=boot, latitude_deg=msg.lat / 1e7,
                    longitude_deg=msg.lon / 1e7, altitude_m=msg.alt / 1000.0,
                    relative_altitude_m=msg.relative_alt / 1000.0,
                    speed_mps=math.hypot(msg.vx, msg.vy) / 100.0,
                    heading_deg=msg.hdg / 100.0 if msg.hdg != 65535 else float("nan"),
                    mission_item=mission_item,
                ))
    finally:
        mav.close()
    if start_boot is None:
        raise ValueError("No position samples after an armed AUTO heartbeat in the recording")
    # The firmware's home-relative altitude can have a zero home-altitude datum.
    # Use the stationary ground estimate immediately before AUTO in the SAME field.
    initial = [p["altitude_m"] for p in positions if start_boot - 2 <= p["boot_time_s"] <= start_boot]
    ground = float(np.median(initial))
    rows = [p for p in positions if p["boot_time_s"] >= start_boot]
    if len(rows) < 2:
        raise ValueError("At least two position samples are required")
    for row in rows:
        row["time_s"] = row["boot_time_s"] - start_boot
        row["height_above_initial_ground_m"] = row["altitude_m"] - ground
    return rows, dict(auto_start_boot_time_s=start_boot, ground_altitude_reference_m=ground,
                      reached_items=sorted(set(reached)))


def read_waypoints(path):
    loader = mavwp.MAVWPLoader()
    loader.load(str(path))
    if loader.count() < 2:
        raise ValueError("Expected a QGC home row and at least one waypoint")
    home = loader.wp(0)
    waypoints = []
    for i in range(1, loader.count()):
        wp = loader.wp(i)
        if wp.command != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            continue
        n, e = local_ne(wp.x, wp.y, home.x, home.y)
        # Rover waypoints constrain horizontal position, not flying altitude.
        waypoints.append(dict(seq=int(wp.seq), north_m=float(n), east_m=float(e)))
    if not waypoints:
        raise ValueError("No navigation waypoints in the mission")
    return float(home.x), float(home.y), waypoints


def select_window(rows, start, end):
    if not math.isfinite(start) or start < 0:
        raise ValueError("--start must be finite and nonnegative")
    end = rows[-1]["time_s"] if end is None else end
    if not math.isfinite(end) or end <= start:
        raise ValueError("--end must be finite and greater than --start")
    if end > rows[-1]["time_s"] + 1e-6:
        raise ValueError(f"Requested end {end:g}s exceeds recording duration {rows[-1]['time_s']:.3f}s")
    selected = [p for p in rows if start <= p["time_s"] <= end]
    if len(selected) < 2:
        raise ValueError("Fewer than two position samples in the selected time window")
    return selected


def plot(rows, waypoints, prefix):
    n = np.array([r["north_m"] for r in rows])
    e = np.array([r["east_m"] for r in rows])
    z = np.array([r["height_above_initial_ground_m"] for r in rows])
    wn = np.array([w["north_m"] for w in waypoints])
    we = np.array([w["east_m"] for w in waypoints])
    start, end = rows[0]["time_s"], rows[-1]["time_s"]
    title = f"ArduRover WP Following Mission\nRectangle trajectory | {start:.1f}\u2013{end:.1f} s after AUTO"
    plt.rcParams.update({"font.size": 12, "axes.titlesize": 16, "savefig.facecolor": "white"})
    for dimension in ("3d", "2d"):
        fig = plt.figure(figsize=(9.5, 8.5), layout="constrained")
        ax = fig.add_subplot(111, projection="3d" if dimension == "3d" else None)
        if dimension == "3d":
            ax.plot(n, e, z, color="royalblue", lw=2.4, label="trajectory")
            ax.scatter(n[0], e[0], z[0], c="green", marker="D", s=75, label=f"start at {start:.1f} s", depthshade=False)
            ax.scatter(n[-1], e[-1], z[-1], c="red", marker="X", s=90, label=f"end at {end:.1f} s", depthshade=False)
            ax.scatter(wn, we, np.zeros(len(wn)), c="black", s=42, label="mission waypoints", depthshade=False)
            for w in waypoints:
                ax.text(w["north_m"], w["east_m"], 0.2, str(w["seq"]), fontsize=14)
            ax.set(xlabel="North (m)", ylabel="East (m)", zlabel="Height above initial ground (m)")
            ax.set_zlim(min(-1, float(z.min()) - 0.5), max(3, float(z.max()) + 0.5))
            horizontal = [np.ptp(np.r_[n, wn]), np.ptp(np.r_[e, we])]
            ax.set_box_aspect([max(horizontal[0], 1), max(horizontal[1], 1), max(horizontal) * 0.45])
            ax.view_init(elev=26, azim=-58)
            ax.zaxis.labelpad = 12
        else:
            ax.plot(e, n, color="royalblue", lw=2.4, label="trajectory")
            ax.scatter(e[0], n[0], c="green", marker="D", s=75, label=f"start at {start:.1f} s", zorder=4)
            ax.scatter(e[-1], n[-1], c="red", marker="X", s=90, label=f"end at {end:.1f} s", zorder=5)
            ax.scatter(we, wn, c="black", s=42, label="mission waypoints", zorder=3)
            for w in waypoints:
                ax.annotate(str(w["seq"]), (w["east_m"], w["north_m"]), xytext=(7, 8), textcoords="offset points", fontsize=14)
            ax.set(xlabel="East (m)", ylabel="North (m)")
            ax.set_aspect("equal", adjustable="box")
            ax.margins(0.18)
            ax.grid(alpha=0.3)
        ax.set_title(title, pad=18)
        ax.legend(loc="upper left", fontsize=10, framealpha=0.92)
        fig.supxlabel("MAVLink estimated position; firmware simulation time", fontsize=10)
        for suffix in ("png", "svg", "pdf"):
            fig.savefig(f"{prefix}_{dimension}.{suffix}", dpi=180)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--mission", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="Output prefix, without extension")
    parser.add_argument("--start", type=float, default=0.0, help="Simulation seconds since first armed AUTO position")
    parser.add_argument("--end", type=float)
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--require-complete", action="store_true")
    args = parser.parse_args()
    rows, metadata = read_track(args.log, args.system_id)
    lat0, lon0, waypoints = read_waypoints(args.mission)
    n, e = local_ne([r["latitude_deg"] for r in rows], [r["longitude_deg"] for r in rows], lat0, lon0)
    for r, ni, ei in zip(rows, n, e):
        r.update(north_m=float(ni), east_m=float(ei))
    completed = waypoints[-1]["seq"] in metadata["reached_items"]
    if args.require_complete and not completed:
        raise ValueError("Final MISSION_ITEM_REACHED was not recorded; mission completion is unconfirmed")
    selected = select_window(rows, args.start, args.end)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    plot(selected, waypoints, args.output)
    with Path(f"{args.output}.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(selected[0]))
        writer.writeheader()
        writer.writerows(selected)
    distances = {str(w["seq"]): float(np.min(np.hypot(n - w["north_m"], e - w["east_m"]))) for w in waypoints}
    summary = dict(
        log=str(args.log.resolve()), mission=str(args.mission.resolve()),
        position_source="GLOBAL_POSITION_INT (firmware estimated position)",
        time_reference="Firmware time_boot_ms, relative to first position after armed AUTO heartbeat",
        altitude_reference="GLOBAL_POSITION_INT.alt minus median stationary altitude in the 2 seconds before AUTO",
        origin_latitude_deg=lat0, origin_longitude_deg=lon0,
        **metadata, mission_completed=completed, full_duration_s=rows[-1]["time_s"],
        plotted_window_s=[selected[0]["time_s"], selected[-1]["time_s"]],
        position_samples=len(selected), full_position_samples=len(rows),
        full_path_length_m=float(np.hypot(np.diff(n), np.diff(e)).sum()),
        max_ground_speed_mps=max(r["speed_mps"] for r in rows),
        minimum_waypoint_distance_m=distances, waypoints=waypoints,
    )
    Path(f"{args.output}.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
