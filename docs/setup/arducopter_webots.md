# ArduCopter + Webots bring-up (Phase 0.6)

Confirms that **Webots R2025a on Windows** can act as the physics engine for
**ArduCopter 4.6.3 SITL running in WSL2**, with MAVLink reaching Mission
Planner on Windows and a sane Copter vehicle type / rate.

Phases 1–3 build on this stack. Phase 4 (wind + QuadPlane weathervane) falls
back to `scripts/start_sitl.sh` + ArduPlane outside Webots; see
`docs/setup/smoke_e2e.md` for that.

## Topology

```
┌──────────────── WSL2 (Ubuntu-22.04) ───────────────┐
│                                                    │
│  arducopter --model webots-python                  │
│     --sim-address <WIN_IP>                         │
│                                                    │
│     UDP :9003 ◀── FDM (IMU, GPS, pose) ─── Webots  │
│     UDP 9002 ──▶ servo / PWM  ──────────▶ Webots   │
│                                                    │
│     TCP :5760  ◀──────────────┐                    │
│                               │                    │
│          scripts/sitl_udp_bridge.py  ◀──── Ctrl+C  │
│                               │                    │
│          UDP :14550 ──────────┼────▶ MP (Windows)  │
└───────────────────────────────┼────────────────────┘
                                │
┌─────────── Windows ────── ────▼──────────────────┐
│  Webots R2025a                                   │
│    worlds/smoke_iris.wbt                         │
│    controller: ardupilot_vehicle_controller      │
│      --sitl-address <WSL_IP>                     │
│  Mission Planner on UDP :14550                   │
└──────────────────────────────────────────────────┘
```

## Prerequisites

- `docs/setup/sitl_wsl.md` completed (ArduPilot tree at `~/boat_lending/ardupilot`, prereqs installed).
- `docs/setup/webots_windows.md` completed (Webots R2025a at `C:\Program Files\Webots`).
- `docs/setup/mission_planner_windows.md` completed.
- ArduCopter binary built in WSL:

  ```bash
  cd ~/boat_lending/ardupilot
  ./waf configure --board sitl   # idempotent, skip if already configured
  ./waf copter
  ```

  Incremental build — shares most objects with arduplane. Expect a few minutes.

## 1. Headless SITL-side smoke

This gate checks the ArduCopter binary + JSON-SITL wiring without needing
Webots or Mission Planner. It launches arducopter, runs a minimal
zeroed-FDM mock on UDP 9002/9003, and verifies HEARTBEAT on TCP 5760.

```bash
cd /mnt/c/Users/user/Desktop/projects/boat_lending
python3 scripts/smoke_arducopter.py
```

Expected output:

```
[smoke] launching: .../arducopter -S -I0 --model webots-python --sim-address 127.0.0.1 ...
[smoke] first HB: type=2 autopilot=3 mode=0
[smoke] 10s: 11 HBs, 30 msg types
[smoke] PASS
```

`type=2` is `MAV_TYPE_QUADROTOR`, `autopilot=3` is `MAV_AUTOPILOT_ARDUPILOTMEGA`.
Reference run (2026-04-16): 11 HBs / 30 msg types in 10 s.

## 2. Launch SITL for the real Webots + MP path

In a WSL terminal:

```bash
cd /mnt/c/Users/user/Desktop/projects/boat_lending
bash scripts/start_arducopter.sh
```

The script prints:

```
windows host   : <WIN_IP>
WSL IP (paste into Webots --sitl-address): <WSL_IP>
JSON-SITL UDP  : 9002 (ctrl in)  9003 (FDM out)
GCS UDP port   : 14550
```

Copy `<WSL_IP>` — you'll need it in step 3. MAVProxy is **not** used;
`scripts/sitl_udp_bridge.py` takes its place as a transparent TCP↔UDP relay.

## 3. Open the Webots smoke world

1. Launch Webots R2025a.
2. `File → Open World...` → `webots/worlds/smoke_iris.wbt`.
3. Click the Iris node in the scene tree, expand `controllerArgs`, and
   change the `--sitl-address` entry from `127.0.0.1` to `<WSL_IP>` from
   step 2.
4. Save the world (`Ctrl+Shift+S`) **only if** you want the IP persisted.
   (Avoid committing that; WSL IPs change across reboots.)
5. Press ▶ in Webots.

In the WSL terminal from step 2 you should see `to-GCS` / `to-SITL` byte
counters tick up, confirming bidirectional traffic.

## 4. Attach Mission Planner

1. Start Mission Planner.
2. Connection type: **UDP**. Click **Connect**. Accept port **14550**.
3. HUD populates within a few seconds:
   - Vehicle is identified as **ArduCopter**.
   - Mode: `STABILIZE`.
   - Attitude indicator responds to any arm/throttle you apply via RC
     overrides or `rc` commands.

## Pass criteria

| # | Check | Evidence |
|---|-------|----------|
| 1 | Headless SITL smoke passes | `scripts/smoke_arducopter.py` exits 0 |
| 2 | Webots controller connects to SITL | `start_arducopter.sh` bridge shows non-zero `to-SITL` bytes |
| 3 | MP receives MAVLink on UDP 14550 | HUD shows ArduCopter HB, mode string |
| 4 | Webots ODE ticks under SITL's servo commands | Iris responds to arm + throttle; no FDM timeouts in arducopter log |

## Shutdown

- `Ctrl+C` in the WSL terminal — stops the bridge and kills arducopter.
- Stop simulation in Webots (■) and close the app or leave it open for the
  next run.
- Close Mission Planner normally.

## Troubleshooting

- **Smoke script exits with `arducopter did not open TCP 5760 within 20s`** —
  the binary crashed early. Check `/tmp/arducopter_smoke.log` for errors.
  Common cause: missing `params/iris.parm` after a checkout — verify it
  exists and is not empty.
- **Bridge bytes go only one way (`to-GCS` increases, `to-SITL` is 0)** —
  Mission Planner hasn't sent anything yet. Harmless unless MP never
  populates the HUD (then the issue is outbound-side, usually a Windows
  firewall rule blocking UDP 14550 inbound).
- **Webots controller says `Listening for ardupilot SITL` forever** — FDM
  socket on UDP 9002 never received SITL's control packet.
  - Verify `start_arducopter.sh` printed the expected `<WIN_IP>` (it should
    match `ipconfig` on Windows for the vEthernet (WSL) adapter).
  - Windows firewall may be blocking inbound UDP 9002 to Webots. Either
    disable the firewall for this session or add an inbound UDP rule.
- **arducopter log shows `recv_fdm timeout`** — Webots controller is not
  sending FDM back to SITL on UDP 9003.
  - WSL IP pasted into `--sitl-address` is stale (WSL rebooted between
    runs). Re-run `start_arducopter.sh` to get the current one.
  - Windows-to-WSL UDP 9003 is blocked. On Windows 11 consider
    `wsl --set-default-version 2` with `[wsl2] networkingMode=mirrored` in
    `.wslconfig`. On this Windows 10 LTSC box that mode is unavailable;
    confirm no third-party firewall is blocking the adapter.
- **Wrong vehicle type in MP** (Plane instead of Copter) — you are still
  attached to the ArduPlane instance from `start_sitl.sh`. Ensure only one
  SITL is running (`pgrep -x arduplane arducopter` in WSL).
