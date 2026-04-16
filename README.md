# boat_lending

Simulation stand for autonomous precision landing on a ship deck.

A fully closed-loop SIL pipeline `perception → LANDING_TARGET → ArduPilot PRECLAND` for live tuning of parameters, logs, and behaviour without touching real hardware.

The detector starts as **ArUco/AprilTag (OpenCV)**: deterministic detection isolates control errors from perception errors. A real CNN on QR/target markers can plug into the same MAVLink interface later with no changes to ArduPilot.

## Stack

- **Webots R2023b+** (Windows native) — physics (ODE) + scene simulation + camera + supervisor for ground truth
- **ArduCopter 4.6.3 SITL** (WSL2 Ubuntu 22.04) — autopilot with PRECLAND + Kalman target estimator, driven by Webots FDM via the JSON-SITL backend (`--model webots-python`). ArduPlane QuadPlane SITL is also built (retained for Phase 4 weathervane work outside Webots).
- **Python companion** (Windows) — perception + MAVLink bridge
- **Mission Planner / MAVProxy** — GCS, visualization, tuning

Phase 1–3 fly ArduCopter because ArduPilot's upstream Webots bridge ships only multirotor/rover flight models (fixed-wing needs a custom lift/drag plugin). ArduPlane QuadPlane's QLAND is the same PRECLAND + position-hold code path as Copter LAND, so the companion code and `LANDING_TARGET` interface port 1:1 between the two.

## Pipeline architecture

```
Webots (ODE physics + ship + ArUco-deck + Iris + camera)
  ├── ardupilot_vehicle_controller  (UDP 9002/9003 JSON-SITL ↔ SITL)
  └── TCP 5599 grayscale frames
        ▼
companion.py
  ├── cv2.aruco.detectMarkers
  ├── pixel offset → body-frame metres (intrinsics + height)
  └── MAVLink LANDING_TARGET → UDP 127.0.0.1:14551
  ▼
ArduCopter SITL 4.6.3 (--model webots-python, LAND + PRECLAND)
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
│   ├── worlds/                   # smoke_iris.wbt, stage1_static.wbt, stage2_static_ship.wbt, stage3_moving_ship.wbt
│   ├── controllers/              # ardupilot_vehicle_controller/ (from ArduPilot upstream), ship_mover/
│   ├── protos/                   # Iris.proto, ArucoMarker.proto (+ meshes/, textures/)
│   └── UPSTREAM.md               # attribution for imported ArduPilot assets
├── params/
│   ├── iris.parm                 # Copter defaults for Webots
│   └── precland_copter.parm      # PRECLAND / LAND tuning overlay
├── scripts/
│   ├── start_arducopter.sh       # arducopter + sitl_udp_bridge.py (WSL)
│   ├── start_sitl.sh             # arduplane QuadPlane launcher (retained for Phase 4)
│   ├── sitl_udp_bridge.py        # byte-level TCP 5760 ↔ UDP 14550 relay (replaces MAVProxy)
│   ├── smoke_arducopter.py       # headless SITL-side smoke gate
│   └── analyze_precland_log.py   # .BIN parser for PL/CTUN/NKF
└── README.md
```

## Windows / WSL split

| Component | Environment | Reason |
|-----------|-------------|--------|
| Webots + worlds + controllers | Windows native | GUI simulator, easier to run on Windows |
| companion.py + OpenCV + pymavlink | Windows | lives next to Webots, talks over TCP/UDP |
| ArduCopter / ArduPlane SITL + transport bridge | WSL2 Ubuntu 22.04 | `install-prereqs-ubuntu.sh` is the canonical route |
| Mission Planner | Windows | Windows-only binary |

TCP across the WSL ↔ Windows boundary flows over `localhost` (WSL2 forwards by default). UDP across that boundary needs a known WSL IP — the launch scripts print it and the Webots controller takes it via `--sitl-address`.

## Network endpoints

| Port | Transport | Direction | Payload |
|------|-----------|-----------|---------|
| 5760 | TCP | SITL ↔ bridge | arducopter's primary MAVLink serial |
| 9002 | UDP | SITL → Webots controller | servo / PWM controls |
| 9003 | UDP | Webots controller → SITL | FDM: IMU, GPS, orientation, pose |
| 14550 | UDP | SITL → GCS (via bridge) | Mission Planner HUD |
| 14551 | UDP | companion ↔ SITL | `LANDING_TARGET`, `DISTANCE_SENSOR`, `HEARTBEAT` |
| 5599 | TCP | Webots controller → companion | grayscale camera frames |

## Setup

- [SITL in WSL2](docs/setup/sitl_wsl.md) — ArduPlane 4.6.3 QuadPlane build and smoke test
- [Webots on Windows](docs/setup/webots_windows.md) — Webots R2025a install and smoke test
- [Python companion on Windows](docs/setup/python_companion_windows.md) — venv, pymavlink, OpenCV ArUco
- [Mission Planner on Windows](docs/setup/mission_planner_windows.md) — GCS install and SITL connect recipes
- [End-to-end smoke test (ArduPlane)](docs/setup/smoke_e2e.md) — Phase 0.5 runbook: SITL + Mission Planner + Webots stock scene running concurrently
- [ArduCopter + Webots JSON-SITL bring-up](docs/setup/arducopter_webots.md) — Phase 0.6 runbook: the Iris + ArduCopter stack used by Phases 1–3

## Design docs

- [Phase 0.6 — Webots + ArduCopter JSON-SITL bring-up](docs/design/2026-04-16-phase0-webots-arducopter-bringup.md)

## Workflow

- One task = one branch = one session → commit → merge
- Branch naming: `phase0/env-setup`, `phase1/static-aruco-world`, `feat/companion-detector`, ...
- `main` is protected; everything lands via PR/merge

## Explicit non-goals

- CNN training (separate effort after Phase 4)
- Water physics / 6-DoF wave motion (Phase 5, possibly on Gazebo)
- Real hardware
- GPS-denied terminal guidance
