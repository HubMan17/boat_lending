# Upstream attribution

The following files in this tree are copied verbatim from ArduPilot
(`libraries/SITL/examples/Webots_Python/` in ArduPilot 4.6.3) and carry
the upstream project's GPL-3.0-or-later licence:

- `protos/Iris.proto`
- `protos/ArucoMarker.proto`
- `protos/meshes/iris.dae`, `iris_prop_ccw.dae`, `iris_prop_cw.dae`
- `protos/textures/aruco_{0..8}.png`
- `controllers/ardupilot_vehicle_controller/ardupilot_vehicle_controller.py`
- `controllers/ardupilot_vehicle_controller/webots_vehicle.py`
- `../params/iris.parm`

Source: <https://github.com/ArduPilot/ardupilot/tree/Plane-4.6.3/libraries/SITL/examples/Webots_Python>

World files (`worlds/*.wbt`) under this tree are authored locally,
informed by the upstream `iris_camera.wbt` but not copied verbatim.
