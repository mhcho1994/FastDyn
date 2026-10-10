"""FastDyn MAVCesium adapter.

FastDyn vehicle configs use WGS84 ellipsoid altitude so MAVCesium can render
positions directly. A display-only altitude offset is still accepted for older
configs, but defaults to zero.
"""

from __future__ import annotations

import logging
import os
import re
from pathlib import Path
import time

from MAVProxy.modules.mavproxy_cesium import CesiumModule
from MAVProxy.modules.mavproxy_cesium.app import cesium_web_server
from tornado.httpserver import HTTPServer
from tornado.web import RequestHandler, StaticFileHandler


_ASSETS = Path(__file__).with_name("cesium_assets")


class _QuadrotorCoreHandler(RequestHandler):
    def get(self):
        # Keep upstream controls/telemetry intact and replace only its model URL.
        core = Path(cesium_web_server.APP_STATIC) / "DST/js/core.js"
        script = core.read_text(encoding="utf-8")
        original = "models/Cesium_Air.glb"
        if original not in script:
            raise RuntimeError("MAVCesium core.js no longer contains the expected aircraft model")
        script = script.replace(original, "models/quadrotor.glb")
        script += """
// Model credit remains visible alongside Cesium's map attribution.
viewer.entities.getById('vehicle').name = 'Quadrotor';
var quadrotorCredit = document.createElement('div');
quadrotorCredit.style.cssText = 'position:absolute;bottom:30px;left:8px;z-index:5;' +
    'padding:2px 6px;background:rgba(0,0,0,0.65);color:white;font:11px sans-serif;';
quadrotorCredit.innerHTML = '<a style="color:white" target="_blank" rel="noopener" ' +
    'href="https://sketchfab.com/3d-models/drone-2588b8a0917d474e97886763c98a65af">' +
    'Drone by Cafitz3D</a> · <a style="color:white" target="_blank" rel="noopener" ' +
    'href="https://creativecommons.org/licenses/by/4.0/">CC BY 4.0</a> (adapted)';
viewer.container.appendChild(quadrotorCredit);
"""
        self.set_header("Content-Type", "application/javascript; charset=utf-8")
        self.set_header("Cache-Control", "no-store")
        self.write(script)


class _FastDynCesiumApplication(cesium_web_server.Application):
    def __init__(self, config, module_callback):
        super().__init__(config, module_callback)
        prefix = re.escape("/" + config.APP_PREFIX + "static/DST/")
        self.add_handlers(r".*$", [
            (prefix + r"js/core\.js", _QuadrotorCoreHandler),
            (prefix + r"models/(quadrotor\.glb)", StaticFileHandler,
             {"path": str(_ASSETS)}),
        ])


def _start_fastdyn_app(config, module_callback):
    logging.getLogger("tornado").setLevel(logging.WARNING)
    application = _FastDynCesiumApplication(config, module_callback)
    server = HTTPServer(application)
    server.listen(port=int(config.SERVER_PORT), address=str(config.SERVER_INTERFACE))
    return server


def _write_dynamic_config(
    *,
    port: int,
    websocket_port: int | None = None,
    server_interface: str = "0.0.0.0",
    websocket_interface: str = "0.0.0.0",
) -> Path:
    websocket_port = port if websocket_port is None else websocket_port
    work_dir = Path(os.environ.get("FASTDYN_WORK_DIR", ".")).expanduser().resolve()
    work_dir.mkdir(parents=True, exist_ok=True)
    config_path = work_dir / "mavcesium.ini"
    config_path.write_text(
        "\n".join(
            [
                "[general]",
                f"server_interface = {server_interface}",
                f"server_port = {port}",
                f"websocket_interface = {websocket_interface}",
                f"websocket_port = {websocket_port}",
                "app_secret_key = ''",
                "app_prefix = mavcesium/",
                "",
                "[api_keys]",
                "",
                "[debug]",
                "app_debug = 1",
                "module_debug = 1",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return config_path


class FastDynCesiumModule(CesiumModule):
    def start_server(self):
        # Reuse the upstream server lifecycle with FastDyn's asset routes.
        cesium_web_server.start_app = _start_fastdyn_app
        super().start_server()

    def __init__(
        self,
        mpstate,
        *,
        altitude_offset_m: float | str = 0.0,
        msl_to_wgs84_offset_m: float | str | None = None,
        port: int | str | None = None,
        server_port: int | str | None = None,
        websocket_port: int | str | None = None,
        server_interface: str = "0.0.0.0",
        websocket_interface: str = "0.0.0.0",
        **kwargs,
    ):
        if msl_to_wgs84_offset_m is not None:
            altitude_offset_m = msl_to_wgs84_offset_m
        self.altitude_offset_m = float(altitude_offset_m)
        selected_port = port if port is not None else server_port
        if selected_port is not None and "configuration" not in kwargs:
            config_path = _write_dynamic_config(
                port=int(selected_port),
                websocket_port=int(websocket_port) if websocket_port is not None else None,
                server_interface=server_interface,
                websocket_interface=websocket_interface,
            )
            kwargs["configuration"] = str(config_path)
        super().__init__(mpstate, **kwargs)
        self.description = "FastDyn Cesium map module"
        self.mpstate.console.writeln(
            "FastDyn MAVCesium display altitude offset: "
            f"{self.altitude_offset_m:g} m"
        )

    def _display_alt_mm(self, altitude_mm: int | float) -> int:
        return int(round(float(altitude_mm) + self.altitude_offset_m * 1000.0))

    def _display_alt_m(self, altitude_m: int | float) -> float:
        return float(altitude_m) + self.altitude_offset_m

    def send_mission(self):
        self.mission = {}
        self.mission_points_to_send = self.mpstate.public_modules["wp"].wploader.wpoints
        for point in self.mission_points_to_send:
            point_dict = point.to_dict()
            seq = point_dict["seq"]
            del point_dict["seq"]
            if int(point_dict.get("frame", -1)) == 0 and "z" in point_dict:
                point_dict["z"] = self._display_alt_m(point_dict["z"])
            self.mission[seq] = point_dict
        self.send_data({"mission_data": self.mission})

    def mavlink_packet(self, m):
        if self.master.flightmode != self.flightmode:
            self.send_flightmode()

        if m.get_type() == "POSITION_TARGET_GLOBAL_INT":
            msg_dict = m.to_dict()
            self.pos_target["lat"] = msg_dict["lat_int"]
            self.pos_target["lon"] = msg_dict["lon_int"]
            self.pos_target["alt_wgs84"] = self._display_alt_m(msg_dict["alt"])

            if None not in self.pos_target.values():
                self.send_data({"pos_target_data": self.pos_target})

        if m.get_type() in self.data_stream:
            msg_dict = m.to_dict()
            if m.get_type() == "GLOBAL_POSITION_INT":
                msg_dict["alt"] = self._display_alt_mm(msg_dict["alt"])
            msg_dict["timestamp"] = m._timestamp
            self.send_data({"mav_data": msg_dict})

        last_wp_change = self.module("wp").wploader.last_change
        if self.wp_change_time != last_wp_change and abs(time.time() - last_wp_change) > 1:
            self.wp_change_time = last_wp_change
            self.send_mission()
            self.rally_change_time = time.time()

        if self.fence_change_time != self.module("fence").wploader.last_change:
            self.fence_change_time = self.module("fence").wploader.last_change
            self.send_fence()


def init(mpstate, **kwargs):
    return FastDynCesiumModule(mpstate, **kwargs)
