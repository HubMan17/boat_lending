# boat_lending

Simulation stand for autonomous QuadPlane precision landing on a ship deck.

A fully closed-loop SIL pipeline `perception → LANDING_TARGET → ArduPlane PRECLAND` for live tuning of parameters, logs, and behaviour without touching real hardware.

The detector starts as **ArUco/AprilTag (OpenCV)**: deterministic detection isolates control errors from perception errors. A real CNN on QR/target markers can plug into the same MAVLink interface later with no changes to ArduPlane.

## Stack

- **Webots R2023b+** (Windows native) — scene simulation, camera, supervisor for ground truth
- **ArduPlane 4.6.3 SITL** (WSL2 Ubuntu 22.04) — autopilot with PRECLAND + Kalman target estimator
- **Python companion** (Windows) — perception + MAVLink bridge
- **Mission Planner / MAVProxy** — GCS, visualization, tuning

## Pipeline architecture

```
Webots (ship + ArUco-deck + QuadPlane + camera)
  │ camera frames via Webots controller
  ▼
companion.py
  ├── cv2.aruco.detectMarkers
  ├── pixel offset → body-frame metres (intrinsics + height)
  └── MAVLink LANDING_TARGET → UDP 127.0.0.1:14551
  ▼
ArduPlane SITL 4.6.3 (QuadPlane, QLAND + PRECLAND)
  │ MAVLink out → UDP 127.0.0.1:14550
  ▼
Mission Planner / MAVProxy (GCS)
```

The Webots Supervisor API provides ground-truth positions of the ship and the QuadPlane in parallel — a separate channel into the companion log used to isolate perception error `(gt_offset − detected_offset)` from control error.

## Phases

| Phase | Scene | Success criterion |
|-------|-------|-------------------|
| 0 | Environment setup (SITL in WSL + Webots on Windows + Python) | smoke test each component in isolation |
| 1 | Static ArUco on the ground | touchdown within ±0.5 m of marker centre, `PL.target_x/y → 0` monotonic |
| 2 | Static ship with ArUco deck plate | touchdown on deck (not water), clean PL log |
| 3 | Moving ship up to 8 m/s | ≤1 m error at 5 m/s; 8 m/s may break — record as stand limit |
| 4 | Wind up to 8 m/s + moving ship | ±1 m at 8 m/s wind + 5 m/s ship, `NKF*` with no variance spikes |

## Repository layout

```
boat_lending/
├── companion/                    # Python perception + MAVLink bridge (Windows)
│   ├── companion.py              # main loop: frames → detect → MAVLink
│   ├── detector.py               # OpenCV ArUco + pixel→body frame
│   ├── mavlink_sender.py         # LANDING_TARGET + DISTANCE_SENSOR packers
│   ├── camera_intrinsics.yaml    # fx, fy, cx, cy from Webots
│   └── groundtruth_logger.py     # parallel gt vs detected log
├── webots/
│   ├── worlds/                   # stage1_static.wbt, stage2_static_ship.wbt, stage3_moving_ship.wbt
│   ├── controllers/              # quadplane_camera/, ship_mover/
│   └── protos/                   # ArucoMarker.proto
├── params/
│   └── precland_quadplane.parm   # ArduPlane params for Mission Planner
├── scripts/
│   ├── start_sitl.sh             # sim_vehicle.py wrapper (runs in WSL)
│   └── analyze_precland_log.py   # .BIN parser for PL/CTUN/NKF
└── README.md
```

## Windows / WSL split

| Component | Environment | Reason |
|-----------|-------------|--------|
| Webots + worlds + controllers | Windows native | GUI simulator, easier to run on Windows |
| companion.py + OpenCV + pymavlink | Windows | lives next to Webots, talks over TCP/shmem |
| ArduPlane SITL + MAVProxy | WSL2 Ubuntu 22.04 | `install-prereqs-ubuntu.sh` is the canonical route |
| Mission Planner | Windows | Windows-only binary |

MAVLink UDP traffic between WSL and Windows flows over localhost without extra configuration (WSL2 auto-forwards).

## MAVLink endpoints

| Port | Direction | Payload |
|------|-----------|---------|
| 14550 | SITL → GCS | Mission Planner / MAVProxy HUD |
| 14551 | companion → SITL | `LANDING_TARGET`, `DISTANCE_SENSOR`, `HEARTBEAT` |

## Workflow

- One task = one branch = one session → commit → merge
- Branch naming: `phase0/env-setup`, `phase1/static-aruco-world`, `feat/companion-detector`, ...
- `main` is protected; everything lands via PR/merge

## Explicit non-goals

- CNN training (separate effort after Phase 4)
- Water physics / 6-DoF wave motion (Phase 5, possibly on Gazebo)
- Real hardware
- GPS-denied terminal guidance
