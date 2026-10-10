# Sources and licenses

The demo adapts the visualization/analysis workflow from the sibling
`quad_dc_fastdyn_rumoca_analysis` project. Its recordings, reports, firmware
patches, satellite imagery and claimed experimental outcomes are not copied.

The live viewer shell and the quadrotor scene are loaded at runtime from
FastDyn's pinned Rumoca submodule:

- `packages/rumoca-web/viz/viewer.html`
- `examples/interactive/quadrotor/quadrotor_scene.js`
- `examples/assets/` (model, skybox and ground textures)

Rumoca is Apache-2.0; see [RUMOCA_LICENSE.txt](licenses/RUMOCA_LICENSE.txt).
The demo appends its own trajectory/scale/propeller display adapter in
`web/scene_extension.js`. The original-study scene used the same Rumoca scene
family; its original source revision is retained in `provenance.json`.

Three.js 0.170.0, GLTFLoader and BufferGeometryUtils are copied unchanged from
the original study's local vendor files. They are MIT licensed; see
[THREE_LICENSE.txt](licenses/THREE_LICENSE.txt). Their hashes and source paths
are recorded in `provenance.json`. No CDN is needed.

The drone mesh is based on [Drone](https://sketchfab.com/3d-models/drone-2588b8a0917d474e97886763c98a65af)
by [Cafitz3D](https://sketchfab.com/Cafitz3D), licensed under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
Attribution also appears in the offline player. The displayed model and its
scale are illustrative; they do not represent the simulated collision geometry.
The asset-specific records remain in Rumoca's `examples/assets/models/README.md`
and `examples/assets/skybox/README_skiingpenguins_arid2.txt`.

FastDyn's existing Copter Modelica sources and the existing ArduCopter 4.6.2
firmware are used in place. No new firmware binary or ArduCopter 4.7.0 hooks
are redistributed by this demo.

The attack demo references the sibling study's
`models/FastDyn/CopterMissionEnsemble.mo` (relative-air wind and sin^8 gust idea),
`config/base_vehicle_overlay.toml` and
`src/fire_analysis/simulation/mission_runtime.py` (current/load coefficient and
body-FRD current-to-magnetic slopes). Source revision and hashes are recorded
in `provenance.json`. The finite attack schedule, smooth GPS position/velocity
spoofing, and configurable gyro bias/false-rate model are new demo code.
No current compensation, battery behavior, or original-study results are claimed.
