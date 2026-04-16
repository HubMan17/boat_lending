# Mission Planner on Windows

Reproducible install of Mission Planner — the primary GCS for ArduPlane SITL.
GUI runs on Windows; SITL runs in WSL2; they speak MAVLink over UDP/TCP.

## Install

Download the official installer from ArduPilot's firmware server:

- `MissionPlanner-latest.msi` (~50 MB)
- https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.msi

Silent install (default path `C:\Program Files (x86)\Mission Planner\`):

```powershell
Start-Process msiexec.exe `
    -ArgumentList '/i',"$env:USERPROFILE\Downloads\MissionPlanner-latest.msi",'/qn','/norestart' `
    -Verb RunAs -Wait
```

Interactive install is equivalent — run the `.msi`, accept UAC, keep defaults.

Tested on `1.3.83`. Any `1.3.8x` release is expected to work.

## Layout

```
C:\Program Files (x86)\Mission Planner\
├── MissionPlanner.exe               # main binary
├── ChangeLog.txt
├── ArduPlane.param, ArduCopter.param, ...   # param templates
└── ...                              # .NET assemblies, plugins, dialects
```

User config and logs:

```
%USERPROFILE%\Documents\Mission Planner\
├── logs\                            # tlogs + downloaded .BIN
├── config.xml                       # window / last-port settings
└── default.waypoints                # last mission plan
```

## Launch check

Confirms the binary runs:

```powershell
Start-Process 'C:\Program Files (x86)\Mission Planner\MissionPlanner.exe'
```

Expected on first launch:

- Splash screen, then the Flight Data tab opens with an artificial horizon
- Top-right dropdown shows available COM ports plus `TCP`, `UDP`, `UDPCl`, `AUTO`
- Baud dropdown defaults to `115200`
- Title bar shows `Mission Planner 1.3.8x build ...`

Close the window to end the check.

## Connect to SITL (deferred to P0.5)

This step is the end-to-end smoke test for Phase 0 — see `PROGRESS.md` task **P0.5**.
Two canonical connection paths:

### Option A — UDP 14550 via MAVProxy (recommended)

`sim_vehicle.py` in WSL already launches MAVProxy which forwards MAVLink to
UDP `14550`. To reach Mission Planner on the Windows side, add an explicit
out endpoint pointing at the Windows loopback:

```bash
# inside WSL
cd ~/boat_lending/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane \
    --out=udpout:$(ip route show | awk '/default/ {print $3}'):14550
```

Then in Mission Planner: top-right → `UDP` → `Connect` → port `14550`.

### Option B — TCP 5760 directly to the SITL binary

When running the raw `arduplane` binary (no MAVProxy wrapper), SITL exposes
MAVLink on TCP `5760`. WSL2 forwards `0.0.0.0:5760` listeners to the Windows
loopback, so Mission Planner connects to `127.0.0.1:5760`:

- Mission Planner → top-right → `TCP` → `Connect`
- Host: `127.0.0.1`
- Port: `5760`

This path is simpler but is one-client-at-a-time and does not expose the
`DISTANCE_SENSOR` / `LANDING_TARGET` packets the companion will later inject
via UDP `14551`. Use option A for phases 1+.

## MAVLink endpoints recap

| Port | Transport | Direction | Payload |
|------|-----------|-----------|---------|
| 5760 | TCP | SITL ⇄ GCS | raw serial-over-TCP (SITL binary default) |
| 14550 | UDP | SITL → GCS | Mission Planner HUD, telemetry |
| 14551 | UDP | companion → SITL | `LANDING_TARGET`, `DISTANCE_SENSOR` |
