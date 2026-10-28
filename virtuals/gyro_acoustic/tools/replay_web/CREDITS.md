# Acoustic replay sources

The replay adapts the scene integration, two-pane playback and quaternion
interpolation from FastDyn's `demos/physics_analysis` at commit
`74afbccff4386b5b71a003b14d5203d3b5a74920`. The acoustic exporter, controls,
signal panels and local server are maintained by `virtuals/gyro_acoustic`.

The quadrotor scene is loaded unchanged from the selected Rumoca checkout:

- `examples/interactive/quadrotor/quadrotor_scene.js`
- `examples/assets/` (drone, skybox and ground textures)

Rumoca is Apache-2.0; see `licenses/RUMOCA_LICENSE.txt`. No compiler or
simulation runtime is started by this viewer. The scene consumes recorded
poses; the displayed drone mesh has an illustrative 3x scale. Motor RPM and
arming state are not inferred from PWM, so propeller rotation and the arming
beacon are not used.

Three.js 0.170.0, GLTFLoader and BufferGeometryUtils were copied unchanged
from the same FastDyn demo. They are MIT licensed; see
`licenses/THREE_LICENSE.txt`. Original paths and SHA-256 hashes are in
`provenance.json`. All runtime assets are served locally; no CDN is required.

The drone mesh is based on [Drone](https://sketchfab.com/3d-models/drone-2588b8a0917d474e97886763c98a65af)
by [Cafitz3D](https://sketchfab.com/Cafitz3D), licensed under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). Asset-specific
records remain in Rumoca's `examples/assets/models/README.md`,
`examples/assets/skybox/README_skiingpenguins_arid2.txt` and the ground texture
directory.
