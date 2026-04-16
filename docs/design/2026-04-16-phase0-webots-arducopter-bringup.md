# Phase 0 — Webots + ArduCopter JSON-SITL bring-up

Status: design, 2026-04-16.

## Context

Original plan (PROGRESS.md) assumed ArduPlane QuadPlane SITL as the sole physics engine and Webots as a renderer slaved to SITL pose via MAVLink. This is how FlightGear + ArduPilot works, but it is **not** how ArduPilot integrates with Webots: Webots is the physics engine (ODE), the autopilot consumes Webots-generated IMU/GPS over a UDP JSON-SITL protocol, and the autopilot's PWM output drives Webots motor nodes.

ArduPilot's stock `libraries/SITL/examples/Webots_Python/` ships multirotor and rover examples but **no fixed-wing** (fixed-wing needs a custom lift/drag physics plugin that the upstream example does not provide). Writing one is out of scope for this project.

Workaround: run **ArduCopter** (not ArduPlane) against Webots. QLAND on ArduPlane's QuadPlane is, in hover, the same PRECLAND + position-hold code that ArduCopter runs in LAND. The `LANDING_TARGET` MAVLink interface and the `PLND_*` parameters are identical across both vehicles. The perception pipeline we build for ArduCopter in Webots will port to ArduPlane QuadPlane with no companion-side changes; only SITL invocation switches from `--model webots-python` (Copter) to `--model quadplane` (Plane, non-Webots physics) for Phase 4 re-validation.

## Goal of this task (P0.6)

Bring up the Webots ↔ ArduCopter SITL JSON-SITL stack end-to-end on this machine, with the stock Iris example scene as the smoke fixture. No custom world, no ArUco detection, no companion MAVLink traffic yet — those are P1.x.

**Success criteria:**

- `arducopter` binary built in `~/boat_lending/ardupilot/build/sitl/bin/`.
- Stock Webots_Python assets (Iris proto + meshes, ArucoMarker proto + textures, `ardupilot_vehicle_controller`, `iris.parm`) copied into repo under `webots/` and `params/`.
- `scripts/start_arducopter.sh` launches `arducopter --model webots-python:<WIN_IP>:9002`, forwards MAVLink to MP on UDP 14550 via the existing `sitl_udp_bridge.py`.
- A minimal smoke world `webots/worlds/smoke_iris.wbt` (adapted from stock `iris_camera.wbt`, distractors removed) boots, the controller connects to SITL (UDP 9002/9003 across WSL↔Windows boundary), Mission Planner shows HEARTBEAT with `MAV_TYPE=MAV_TYPE_QUADROTOR`, `AUTOPILOT_VERSION` string contains `ArduCopter`.
- Automated SITL-side test (bash + Python + pymavlink) that validates these without needing Webots running.
- Manual runbook in `docs/setup/` covering the Webots + MP side.

**Explicit non-goals for this task:**

- Our custom `stage1_static.wbt` — separate task P1.1.
- Camera TCP stream integration on companion side — P1.3.
- Any MAVLink from companion — P1.4+.
- Arming / hovering verification that requires stick inputs — manual only, in runbook.

## Architecture

```
WSL Ubuntu 22.04                              Windows 10
┌──────────────────────────┐                  ┌───────────────────────────┐
│ arducopter               │                  │ Webots R2025a             │
│   --model                │  UDP 9002 ctrl   │  ardupilot_vehicle_       │
│     webots-python:       │ ───────────────▶ │    controller (Python)    │
│     <WIN_IP>:9002        │                  │                           │
│                          │ ◀──── UDP 9003   │                           │
│                          │       FDM        │                           │
│                          │                  │                           │
│                          │  UDP 14550       │                           │
│                          │ ───────────────▶ │ Mission Planner (GCS)     │
│                          │  (via            │                           │
│                          │   sitl_udp_      │                           │
│                          │   bridge.py)     │                           │
└──────────────────────────┘                  └───────────────────────────┘
```

**Port map:**

| Channel | Direction | Port | Transport | Notes |
|---|---|---|---|---|
| SITL → Webots (controls / PWM) | WSL → Windows | 9002 | UDP | `--model webots-python:<WIN_IP>:9002`; Webots controller binds `0.0.0.0:9002` |
| Webots → SITL (FDM / sensors) | Windows → WSL | 9003 | UDP | Controller sends to `<sitl_address>:9003` via `--sitl-address` CLI arg |
| SITL → Mission Planner | WSL → Windows | 14550 | UDP | `sitl_udp_bridge.py` TCP :5760 ↔ UDP 14550 (reused from P0.5 fix) |
| (Future P1.3) camera stream | Windows controller → Windows companion | 5599 | TCP | Controller `--camera-port 5599` |
| (Future P1.4) companion MAVLink | Windows ↔ WSL | 14551 | UDP | not established in this task |

**IP resolution:**

- `<WIN_IP>` (Windows host from WSL) — already auto-detected in `scripts/start_arducopter.sh` via `ip route show | awk '/^default/ {print $3}'`, same trick as `start_sitl.sh`.
- `<WSL_IP>` (WSL host from Windows) — dynamic on each WSL boot. The launch script prints it; user pastes it into `--sitl-address` in the Webots world's `controllerArgs`, OR exports `SITL_ADDRESS` env var before opening Webots. We avoid hard-coding it.

## Repo layout changes

New/copied:

```
params/iris.parm                           (copied from upstream, Copter defaults for Webots)
webots/protos/Iris.proto                   (copied)
webots/protos/ArucoMarker.proto            (copied)
webots/protos/meshes/{iris,iris_prop_ccw,iris_prop_cw}.dae   (copied)
webots/protos/textures/aruco_{0..8}.png    (copied)
webots/controllers/ardupilot_vehicle_controller/
  ardupilot_vehicle_controller.py          (copied)
  webots_vehicle.py                        (copied)
webots/worlds/smoke_iris.wbt               (adapted from upstream iris_camera.wbt — distractors removed, Iris + camera + ArucoMarker id=0 on a Floor only)
scripts/start_arducopter.sh                (new)
scripts/smoke_arducopter.py                (new, pymavlink 20-s HB/version check)
docs/setup/arducopter_webots.md            (new, runbook)
```

Retained (Phase 4 may still need them):

```
scripts/start_sitl.sh                      (ArduPlane launcher, kept for Phase 4)
scripts/sitl_udp_bridge.py                 (transport-transparent, reused by both)
```

## Implementation steps

1. **Build ArduCopter in WSL.** Incremental — `waf` has already configured for board `sitl` for Plane. Run `./waf copter` in `~/boat_lending/ardupilot`. Expect ~10–15 min on this box.
2. **Copy Webots_Python assets** into `webots/` and `params/`. Verify EXTERNPROTO paths in copied `smoke_iris.wbt` resolve relative to its location (change `../protos/…` if needed — they already use that form, so this should be a direct copy with path adjustments only for background/textured-floor protos that reference upstream `images/`).
3. **Strip `smoke_iris.wbt`** down to: `WorldInfo` + `Viewpoint` + `TexturedBackground` + `TexturedBackgroundLight` + `Floor` (Grass, 50×50) + `Iris` (controller=`ardupilot_vehicle_controller`, extensionSlot with Camera, `--camera-port 5599`) + `ArucoMarker` (id=0, size 1.0, centered at origin). Drop Rabbit/Gnome/Forklift/Dog/OilBarrel/Table/AdvertisingBoard/StraightRoadSegment.
4. **`scripts/start_arducopter.sh`**: kill-prev; `echo` WSL IP for user; launch `arducopter -S -I0 --model webots-python:<WIN_IP>:9002 --defaults params/iris.parm`; wait for TCP 5760; exec `sitl_udp_bridge.py`. Same structure as existing `start_sitl.sh`.
5. **`scripts/smoke_arducopter.py`**: pymavlink, connect `tcp:127.0.0.1:5760`, wait up to 20 s for HB, assert `MAV_TYPE == MAV_TYPE_QUADROTOR`, request `AUTOPILOT_VERSION`, assert version string contains `ArduCopter`, count HB rate over 10 s (expect ≥ 1 Hz, same pattern as P0.5).
6. **Docs**: `docs/setup/arducopter_webots.md` — step-by-step: (a) terminal in WSL: `scripts/start_arducopter.sh`; (b) copy printed WSL IP; (c) open `webots/worlds/smoke_iris.wbt` in Webots; (d) edit `Iris.controllerArgs` to set `--sitl-address <WSL_IP>`; (e) Run in Webots; (f) confirm MP attaches on UDP 14550 with Copter HB.
7. **PROGRESS.md update** (local): mark P0.6 complete with date/branch; rewrite Phase 1 queue to reflect the ArduCopter-on-Webots architecture (see "Plan delta" below).

## Plan delta (for PROGRESS.md)

Phase 0 gains a new entry:

- **P0.6** Webots ↔ ArduCopter JSON-SITL bring-up: ArduCopter built, Webots_Python assets in repo, stock-ish `smoke_iris.wbt` boots, MP attaches with Copter HB.

Phase 1 rewrites to:

- **P1.1** `webots/worlds/stage1_static.wbt` — clean Floor 50×50, ArucoMarker id=0 size 1.0 m at origin, Iris spawn offset (e.g. 5 m, 10 m altitude), Camera in extensionSlot, no distractors.
- **P1.2** companion `camera_receive.py` — TCP client to 5599, decode grayscale frames (header `=HH` width/height + raw bytes), adapt from upstream `example_camera_receive.py`.
- **P1.3** companion `detector.py` — OpenCV ArUco DICT_4X4_50 detection on received frames, pixel → body-frame projection using Webots camera intrinsics (width/height + FOV).
- **P1.4** companion `mavlink_sender.py` — pack `LANDING_TARGET` (#149) + `DISTANCE_SENSOR`, send to SITL via UDP 14551.
- **P1.5** companion `companion.py` — 20 Hz loop, CLI `--stage`.
- **P1.6** `params/precland_copter.parm` — add `PLND_*`, `LAND_SPEED`, `LOG_BITMASK` on top of `iris.parm`.
- **P1.7** `scripts/start_arducopter.sh` extended with PRECLAND flags (or an `env` overlay).
- **P1.8** `webots/controllers/groundtruth_logger/` or supervisor-mode augmentation for GT poses.
- **P1.9** `scripts/analyze_precland_log.py` — unchanged.
- **P1.10** End-to-end: AUTO takeoff (or GUIDED) → hover → LAND with PRECLAND → ±0.5 m.

Phase 4 carries a known debt: weathervane + QuadPlane-specific parameters cannot be validated on Copter. Deferred to real-hardware or a non-Webots SITL run.

## Risks

- **JSON-SITL UDP across the WSL↔Windows boundary.** WSL2 port forwarding is solid for TCP localhost in both directions on recent Windows builds, but UDP is historically flakier. If the controller never receives packets from SITL on 9002, fall back to using WSL IP explicitly via `netsh interface portproxy` rules or switch to mirrored networking mode (`[wsl2] networkingMode=mirrored` in `.wslconfig`, Windows 11+ only — we're on Win10 LTSC so this is not available). Mitigation: document clear diagnostics in the runbook and capture tcpdump on the WSL side if needed.
- **`--sitl-address` dynamic WSL IP.** User-friction. Mitigated by having the launch script print the IP prominently and by documenting the env-var override path.
- **ArduCopter build may pull extra deps** that were not needed for ArduPlane. Mitigation: `waf` is the same toolchain, most of the SITL work is shared; should be incremental and fast.
- **Webots R2025a vs. upstream examples assuming R2023a** (EXTERNPROTO URLs pin `R2023a`). These URLs are pinned to that Webots version's proto repo — should still work in R2025a, but if not, re-pin to `R2025a` in our copy of `smoke_iris.wbt`.

## Verification before calling this task done

- `./waf copter` succeeds.
- `scripts/start_arducopter.sh` starts arducopter, bridge streams to UDP 14550.
- `scripts/smoke_arducopter.py` exits 0 (HB received, MAV_TYPE OK, version string OK) against a running SITL, no Webots.
- Manually: open `smoke_iris.wbt` in Webots, controller connects, MP shows HB, runbook is followable end-to-end without tribal knowledge.
- `git status` clean, PROGRESS.md updated locally, branch `phase0/webots-arducopter-bringup` merged into `main`.
