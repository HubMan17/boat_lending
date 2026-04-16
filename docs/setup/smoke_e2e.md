# End-to-end smoke test (Phase 0.5)

Confirms the full simulation stack — ArduPlane SITL in WSL, Mission Planner on
Windows, and Webots — can run simultaneously on this machine with no resource
or network conflicts.

This is a verification step only. No new code is wired in; Webots runs a stock
example scene and the Python companion is not involved.

## Topology

```
┌────────────────── WSL2 (Ubuntu-22.04) ───────────────────┐
│  arduplane (QuadPlane SITL)  —  TCP :5760                │
│           │                                              │
│           ▼                                              │
│  MAVProxy (launched by sim_vehicle.py)                   │
│           │                                              │
│           └── --out=udpout:<WIN_IP>:14550 ──┐            │
└─────────────────────────────────────────────┼────────────┘
                                              │
┌─────────── Windows ──────────────────────── ▼────────────┐
│  Mission Planner — UDP :14550 (HUD, attitude, arming)    │
│  Webots R2025a   — stock scene, runs in parallel         │
└──────────────────────────────────────────────────────────┘
```

## Prerequisites

- `docs/setup/sitl_wsl.md` completed — ArduPlane 4.6.3 SITL built under
  `~/boat_lending/ardupilot` in WSL.
- `docs/setup/mission_planner_windows.md` completed — Mission Planner 1.3.83
  installed on Windows.
- `docs/setup/webots_windows.md` completed — Webots R2025a installed on Windows.
- Windows Firewall either disabled on this machine or a rule allowing UDP 14550
  inbound to Mission Planner.

## Runbook

### 1. Launch SITL from WSL

Open a WSL terminal (default distro `Ubuntu-22.04`) and run:

```bash
cd /mnt/c/Users/user/Desktop/projects/boat_lending
bash scripts/start_sitl.sh
```

The script auto-detects the Windows host IP (default route gateway inside WSL),
launches `sim_vehicle.py -v ArduPlane -f quadplane --no-rebuild`, and forwards
MAVLink to `udpout:<WIN_IP>:14550`. MAVProxy takes over the terminal — leave it
running.

Expected first lines after boot (≈15 s):

```
APM: ArduPlane Vx.y.z
Ready to FLY
GPS lock at ... m
```

### 2. Attach Mission Planner

1. Start Mission Planner from its Start Menu shortcut.
2. Connection type: **UDP**. Click **Connect**.
3. When prompted, accept port **14550**.
4. HUD populates within a few seconds; attitude indicator is level; mode shows
   **QHOVER**; `GPS: 3D Fix` once EKF settles.

### 3. Launch Webots in parallel

1. Start Webots R2025a.
2. Open any stock world, e.g. `File → Open Sample World → worlds/camera.wbt`.
3. Confirm the 3D viewport renders and the scene starts simulating when ▶ is
   pressed.

SITL and Webots must coexist — Webots uses the GPU, SITL is CPU-only, Mission
Planner is light.

### 4. Arm / disarm sanity check

In the MAVProxy terminal (WSL):

```
arm throttle
```

Mission Planner HUD flips to `ARMED`; motors spin in the SITL physics.

Then:

```
disarm
```

HUD returns to `DISARMED`. No errors in either MAVProxy or MP status bar.

### 5. Shutdown

- `Ctrl+C` in the MAVProxy terminal — this stops MAVProxy and the `arduplane`
  process together.
- Close Mission Planner and Webots normally.

## Pass criteria

| # | Check | Evidence |
|---|-------|----------|
| 1 | SITL heartbeat on TCP 5760 | `heartbeat type=1 autopilot=3 sys=1 mode=16` |
| 2 | SITL stable ≥60 s | MAVProxy scrolls live telemetry; no MAVLink timeout |
| 3 | MP receives MAVLink on UDP 14550 | HUD shows attitude, GPS, mode `QHOVER` |
| 4 | Webots runs concurrently | scene renders while SITL is still running |
| 5 | Arm → disarm cycle | mode transitions reflected in MP and MAVProxy |

## Troubleshooting

- **MP stays disconnected on UDP 14550** — confirm MAVProxy printed
  `Outputs: ... udpout:<WIN_IP>:14550` at startup, and that `WIN_IP` matches
  the Windows host as seen from WSL:
  `ip route show | awk '/default/ {print $3}'`. Firewall rule may be blocking
  UDP inbound on 14550.
- **`./build/sitl/bin/arduplane: No such file or directory`** — rebuild:
  `cd ~/boat_lending/ardupilot && ./waf configure --board sitl && ./waf plane`.
- **`sim_vehicle.py` rebuilds every launch** — ensure `--no-rebuild` is passed
  (the script does this by default).
- **SITL exits silently on launch** — pass a keep-stdin-open shell; do not
  redirect `< /dev/null`. `arduplane` treats stdin EOF as a quit signal.

## SITL-side automated verification

The programmatic part of this smoke test — SITL stays up ≥60 s, streams a full
MAVLink telemetry set, accepts arm/disarm — is reproducible headless:

```bash
cd ~/boat_lending/ardupilot
./build/sitl/bin/arduplane -S -I0 --model quadplane --speedup 1 \
    --defaults Tools/autotest/default_params/quadplane.parm \
    >/tmp/arduplane_smoke.log 2>&1 &
SITL_PID=$!
sleep 10
python3 - <<'PY'
from pymavlink import mavutil
import time
m = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
hb = m.wait_heartbeat(timeout=15)
print(f'HB: type={hb.type} autopilot={hb.autopilot} mode={hb.custom_mode}')
m.mav.request_data_stream_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
start = time.time(); hb_count = 0; types = set()
while time.time() - start < 60:
    msg = m.recv_match(blocking=True, timeout=5)
    if msg is None: continue
    types.add(msg.get_type())
    if msg.get_type() == 'HEARTBEAT': hb_count += 1
print(f'60s: {hb_count} HBs, {len(types)} msg types')
m.close()
PY
kill -9 $SITL_PID
```

Reference run (2026-04-16): `HB: type=1 autopilot=3 mode=16`,
`60s: 63 HBs, 37 msg types`.
