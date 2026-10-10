#!/usr/bin/env python3
"""Plot measured Copter trajectory and velocity/attitude/body-rate telemetry."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.path import Path as MarkerPath
from matplotlib.transforms import Affine2D
import numpy as np
from pymavlink import mavutil, mavwp

ROOT = Path(__file__).resolve().parents[1]
BLUE, GREEN, RED, INK = "#245eea", "#159d4f", "#e4242d", "#111827"
COMPONENT_COLORS = ("#dc493a", "#219669", "#2962ce")
EARTH_RADIUS_M = 6378137.0


def read_telemetry(path: Path, system_id: int = 1):
    connection = mavutil.mavlink_connection(str(path))
    positions, attitudes, events = [], [], []
    armed, auto, seen_armed = False, False, False
    start_boot = None
    mission_item = 0
    vehicle_type = None
    last_boot = None
    final_disarmed = False
    try:
        while (msg := connection.recv_match()) is not None:
            if msg.get_srcSystem() != system_id:
                continue
            kind = msg.get_type()
            if kind == "HEARTBEAT" and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                vehicle_type = int(msg.type)
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                auto = mavutil.mode_string_v10(msg) == "AUTO"
                if seen_armed and not armed:
                    final_disarmed = True
                seen_armed |= armed
            elif kind == "MISSION_CURRENT":
                mission_item = int(msg.seq)
            elif kind == "MISSION_ITEM_REACHED":
                events.append({"boot_time_s": last_boot, "reached_item": int(msg.seq)})
            elif kind in ("GLOBAL_POSITION_INT", "ATTITUDE"):
                boot = msg.time_boot_ms / 1000.0
                last_boot = boot
                target = positions if kind == "GLOBAL_POSITION_INT" else attitudes
                if target and boot < target[-1]["boot_time_s"]:
                    raise ValueError(f"{kind} clock moved backwards; use a single-run recording")
                if target and boot == target[-1]["boot_time_s"]:
                    continue
                if kind == "GLOBAL_POSITION_INT":
                    if not (msg.lat or msg.lon):
                        continue
                    if armed and auto and start_boot is None:
                        start_boot = boot
                    positions.append(dict(
                        boot_time_s=boot, latitude_deg=msg.lat / 1e7, longitude_deg=msg.lon / 1e7,
                        altitude_m=msg.alt / 1000.0, relative_altitude_m=msg.relative_alt / 1000.0,
                        velocity_north_mps=msg.vx / 100.0, velocity_east_mps=msg.vy / 100.0,
                        velocity_down_mps=msg.vz / 100.0,
                        speed_mps=math.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2) / 100.0,
                        mission_item=mission_item,
                    ))
                else:
                    attitudes.append(dict(
                        boot_time_s=boot,
                        roll_deg=math.degrees(msg.roll), pitch_deg=math.degrees(msg.pitch),
                        yaw_deg=math.degrees(msg.yaw),
                        p_deg_s=math.degrees(msg.rollspeed), q_deg_s=math.degrees(msg.pitchspeed),
                        r_deg_s=math.degrees(msg.yawspeed),
                    ))
    finally:
        connection.close()
    if vehicle_type != mavutil.mavlink.MAV_TYPE_QUADROTOR:
        raise ValueError(f"Expected an ArduCopter quadrotor log, got MAV_TYPE={vehicle_type}")
    if start_boot is None or len(attitudes) < 2:
        raise ValueError("Log needs armed AUTO position samples and ATTITUDE telemetry")
    for row in positions + attitudes:
        row["time_s"] = row["boot_time_s"] - start_boot
    # Unwrap before selecting a window; preserve the original wrapped yaw in CSV.
    yaw = np.degrees(np.unwrap(np.radians([r["yaw_deg"] for r in attitudes])))
    for row, value in zip(attitudes, yaw):
        row["yaw_unwrapped_deg"] = float(value)
    return positions, attitudes, dict(auto_start_boot_time_s=start_boot,
                                     disarm_observed=final_disarmed, reached_events=events)


def read_waypoints(path: Path):
    loader = mavwp.MAVWPLoader()
    loader.load(str(path))
    if loader.count() < 2:
        raise ValueError("Mission needs a home row and navigation items")
    home = loader.wp(0)
    points = []
    commands = {mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, mavutil.mavlink.MAV_CMD_NAV_LAND}
    for i in range(1, loader.count()):
        wp = loader.wp(i)
        if wp.command not in commands:
            continue
        if wp.frame in (mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT):
            altitude = float(wp.z)
        elif wp.frame in (mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_FRAME_GLOBAL_INT):
            altitude = float(wp.z - home.z)
        else:
            raise ValueError(f"Waypoint {wp.seq}: unsupported altitude frame {wp.frame}")
        # Zero XY for a takeoff means the current launch location.
        lat, lon = (home.x, home.y) if wp.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF and wp.x == wp.y == 0 else (wp.x, wp.y)
        n, e = local_ne(lat, lon, home.x, home.y)
        points.append(dict(seq=int(wp.seq), command=int(wp.command), north_m=float(n),
                           east_m=float(e), relative_altitude_m=altitude))
    return float(home.x), float(home.y), points


def local_ne(lat, lon, lat0, lon0):
    return (EARTH_RADIUS_M * np.radians(np.asarray(lat) - lat0),
            EARTH_RADIUS_M * math.cos(math.radians(lat0)) * np.radians(np.asarray(lon) - lon0))


def select_window(rows, start, end):
    return [r for r in rows if start <= r["time_s"] <= end]


def body_to_neu(roll, pitch, yaw):
    """Body FRD -> local North/East/Up, using ZYX Euler attitude in radians."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]])
    return np.diag([1, 1, -1]) @ rotation


def attitude_at(attitudes, time_s):
    times = np.array([r["time_s"] for r in attitudes])
    if time_s < times[0] or time_s > times[-1]:
        raise ValueError("Endpoint attitude is outside the ATTITUDE recording")
    # Linear interpolation in Euler angles is only used to orient the two glyphs.
    return np.radians([np.interp(time_s, times, [r[k] for r in attitudes])
                       for k in ("roll_deg", "pitch_deg", "yaw_unwrapped_deg")])


def draw_quadrotor(ax, center, attitude, arm, color):
    rotation = body_to_neu(*attitude)
    center = np.asarray(center)
    motors = arm / math.sqrt(2) * np.array([[1, 1, 0], [1, -1, 0], [-1, -1, 0], [-1, 1, 0]])
    def line(body_points, width=2.2):
        points = np.asarray(body_points) @ rotation.T + center
        ax.plot(*points.T, color=color, linewidth=width, zorder=15)
    for motor in motors:
        line([np.zeros(3), motor], 2.8)
        angle = np.linspace(0, 2 * np.pi, 49)
        circle = np.c_[np.cos(angle), np.sin(angle), np.zeros(len(angle))] * (arm * 0.24)
        line(circle + motor, 1.9)
        line([motor + [-arm*.17, 0, 0], motor + [arm*.17, 0, 0]], 1.1)
    # Body outline and forward-pointing nose; geometry follows measured attitude.
    line(arm * np.array([[.4, 0, 0], [-.23, .22, 0], [-.23, -.22, 0], [.4, 0, 0]]), 2.1)


def quadrotor_marker():
    parts = []
    for x, y in ((.7, .7), (.7, -.7), (-.7, -.7), (-.7, .7)):
        parts.append(MarkerPath([[0, 0], [x, y]], [MarkerPath.MOVETO, MarkerPath.LINETO]))
        parts.append(MarkerPath.unit_circle().transformed(Affine2D().scale(.24).translate(x, y)))
    return MarkerPath.make_compound_path(*parts)


def save_figure(fig, prefix, name):
    for extension in ("png", "pdf", "svg"):
        fig.savefig(f"{prefix}_{name}.{extension}", dpi=200, bbox_inches="tight", pad_inches=0.18)
    plt.close(fig)


def plot_trajectory(positions, attitudes, waypoints, prefix):
    xyz = np.array([[r["north_m"], r["east_m"], r["relative_altitude_m"]] for r in positions])
    wp = np.array([[w["north_m"], w["east_m"], w["relative_altitude_m"]] for w in waypoints])
    all_points = np.vstack([xyz, wp])
    span = np.maximum(np.ptp(all_points, axis=0), 1)
    arm = max(span.max() * .05, .4)
    fig = plt.figure(figsize=(11.5, 9.5))
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
    ax.plot(*xyz.T, color=BLUE, linewidth=2.5, zorder=3)
    ax.scatter(*wp.T, color=INK, s=48, depthshade=False, zorder=4)
    for w in waypoints:
        ax.text(w["north_m"]+.25*arm, w["east_m"]+.25*arm,
                w["relative_altitude_m"]+.65*arm, str(w["seq"]), fontsize=15, color=INK)
    for row, center, color in ((positions[0], xyz[0], GREEN), (positions[-1], xyz[-1], RED)):
        draw_quadrotor(ax, center, attitude_at(attitudes, row["time_s"]), arm, color)
    for setter, axis in ((ax.set_xlim, 0), (ax.set_ylim, 1), (ax.set_zlim, 2)):
        setter(all_points[:, axis].min() - 1.6*arm, all_points[:, axis].max() + 1.6*arm)
    ax.set(xlabel="North (m)", ylabel="East (m)", zlabel="Altitude above home (m)")
    ax.xaxis.labelpad, ax.yaxis.labelpad, ax.zaxis.labelpad = 12, 12, 10
    ax.set_box_aspect(span + 3.2*arm)
    ax.view_init(elev=27, azim=-60)
    handles = [Line2D([], [], color=BLUE, lw=2.5, label="trajectory")]
    for label, color, row in (("start", GREEN, positions[0]), ("end", RED, positions[-1])):
        handles.append(Line2D([], [], color=color, linestyle="none", marker=quadrotor_marker(),
                              markerfacecolor="none", markeredgewidth=1.5, markersize=19,
                              label=f"{label} quadrotor ({row['time_s']:.1f} s)"))
    handles.append(Line2D([], [], color=INK, marker="o", linestyle="none", markersize=7, label="mission waypoints"))
    ax.legend(handles=handles, loc="upper right", bbox_to_anchor=(1.04, 1.03), framealpha=.95)
    fig.suptitle("ArduCopter WP Following Mission", fontsize=20, y=.97)
    fig.text(.5, .925, f"Current copter462 mission | {positions[0]['time_s']:.1f}–{positions[-1]['time_s']:.1f} s after AUTO", ha="center", fontsize=12, color="#4b5563")
    fig.text(.5, .045, "Measured MAVLink position; quadrotor glyphs enlarged and oriented with measured attitude", ha="center", fontsize=10, color="#4b5563")
    fig.subplots_adjust(left=.04, right=.93, bottom=.09, top=.88)
    save_figure(fig, prefix, "trajectory_3d")


def plot_dynamics(positions, attitudes, prefix):
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12.5, 9), layout="constrained")
    position_time = [r["time_s"] for r in positions]
    attitude_time = [r["time_s"] for r in attitudes]
    groups = [
        (positions, position_time, ("velocity_north_mps", "velocity_east_mps", "velocity_down_mps"), ("North", "East", "Down"), "Velocity (m/s)"),
        (attitudes, attitude_time, ("roll_deg", "pitch_deg", "yaw_unwrapped_deg"), ("Roll", "Pitch", "Yaw (unwrapped)"), "Attitude (deg)"),
        (attitudes, attitude_time, ("p_deg_s", "q_deg_s", "r_deg_s"), ("p (roll axis)", "q (pitch axis)", "r (yaw axis)"), "Body angular rate (deg/s)"),
    ]
    for ax, (rows, times, keys, labels, ylabel) in zip(axes, groups):
        for key, label, color in zip(keys, labels, COMPONENT_COLORS):
            ax.plot(times, [r[key] for r in rows], label=label, color=color, linewidth=1.55)
        if ax is axes[0]:
            ax.plot(position_time, [r["speed_mps"] for r in positions], color=INK, linestyle="--", linewidth=1.4, label="3D speed")
        ax.set_ylabel(ylabel)
        ax.axhline(0, color="#9ca3af", linewidth=.6)
        ax.grid(color="#d1d5db", alpha=.65, linewidth=.65)
        ax.legend(loc="upper right", ncols=4 if ax is axes[0] else 3, fontsize=10)
        ax.margins(x=0)
        ax.spines[["top", "right"]].set_visible(False)
    axes[-1].set_xlim(positions[0]["time_s"], positions[-1]["time_s"])
    axes[-1].set_xlabel("Simulation time since AUTO (s)")
    fig.suptitle("ArduCopter — Velocity, Attitude and Body Angular Rates", fontsize=17)
    fig.supxlabel("Velocity: GLOBAL_POSITION_INT (NED)  ·  Attitude and body rates: ATTITUDE  ·  Original telemetry samples", fontsize=10)
    save_figure(fig, prefix, "dynamics")


def write_csv(path, rows):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--mission", type=Path, default=ROOT / "virtuals/physics/flight_controllers/courbet/mavlink/copter_mission.waypoints")
    parser.add_argument("--output", type=Path, default=ROOT / "out/copter-trajectory/copter", help="Output filename prefix")
    parser.add_argument("--start", type=float, default=0, help="Seconds after first armed AUTO position")
    parser.add_argument("--end", type=float)
    parser.add_argument("--system-id", type=int, default=1)
    args = parser.parse_args()
    positions, attitudes, metadata = read_telemetry(args.log, args.system_id)
    lat0, lon0, waypoints = read_waypoints(args.mission)
    if not waypoints:
        raise ValueError("No navigation waypoints to plot")
    north, east = local_ne([r["latitude_deg"] for r in positions], [r["longitude_deg"] for r in positions], lat0, lon0)
    for row, n, e in zip(positions, north, east):
        row.update(north_m=float(n), east_m=float(e))
    available_end = min(positions[-1]["time_s"], attitudes[-1]["time_s"])
    end = available_end if args.end is None else args.end
    if not math.isfinite(args.start) or not math.isfinite(end) or args.start < 0 or end <= args.start or end > available_end + 1e-6:
        raise ValueError(f"Window must satisfy 0 <= start < end <= {available_end:.3f} s")
    selected_pos = select_window(positions, args.start, end)
    selected_att = select_window(attitudes, args.start, end)
    if min(len(selected_pos), len(selected_att)) < 2:
        raise ValueError("Requested window needs at least two position and attitude samples")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 11, "axes.labelsize": 12, "savefig.facecolor": "white"})
    plot_trajectory(selected_pos, attitudes, waypoints, args.output)
    plot_dynamics(selected_pos, selected_att, args.output)
    write_csv(Path(f"{args.output}_position.csv"), selected_pos)
    write_csv(Path(f"{args.output}_attitude.csv"), selected_att)
    summary = dict(
        log=str(args.log.resolve()), mission=str(args.mission.resolve()), **metadata,
        time_reference="Firmware time_boot_ms minus first position after armed AUTO heartbeat",
        altitude_reference="GLOBAL_POSITION_INT.relative_alt (above firmware home)",
        velocity_frame="North/East/Down, m/s; speed is Euclidean 3D magnitude",
        attitude="ZYX roll/pitch/yaw, degrees; yaw unwrapped for plotting; raw yaw retained in CSV",
        angular_rate="ATTITUDE.rollspeed/pitchspeed/yawspeed, body p/q/r in deg/s (not Euler angle derivatives)",
        endpoint_icons="Enlarged quadrotor geometry transformed from body FRD to North/East/Up",
        plotted_window_s=[selected_pos[0]["time_s"], selected_pos[-1]["time_s"]],
        position_samples=len(selected_pos), attitude_samples=len(selected_att),
        position_sample_interval_median_s=float(np.median(np.diff([r["time_s"] for r in selected_pos]))),
        max_relative_altitude_m=max(r["relative_altitude_m"] for r in selected_pos),
        final_relative_altitude_m=selected_pos[-1]["relative_altitude_m"],
        max_speed_mps=max(r["speed_mps"] for r in selected_pos),
        final_mission_item=selected_pos[-1]["mission_item"],
        waypoints=waypoints,
    )
    Path(f"{args.output}_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
