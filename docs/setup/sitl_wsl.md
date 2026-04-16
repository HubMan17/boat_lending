# ArduPlane SITL in WSL2 Ubuntu 22.04

Reproducible install of ArduPlane 4.6.3 SITL for the QuadPlane frame.
All commands run inside WSL (`wsl -d Ubuntu-22.04`).

## Layout

```
~/boat_lending/
├── ardupilot/        # upstream ArduPilot clone at tag Plane-4.6.3
├── logs/             # SITL .BIN logs (local, not tracked)
└── scripts/          # WSL-side shell helpers (added per phase)
```

Kept separate from any other ArduPilot trees in the home directory.

## Prerequisites

Ubuntu 22.04 with the standard ArduPilot build prerequisites installed once via
`Tools/environment_install/install-prereqs-ubuntu.sh -y`. If the following imports
succeed, the step can be skipped:

```bash
python3 -c "import pymavlink, MAVProxy, pexpect, em"  # em == empy
```

Build toolchain required: `git build-essential ccache g++ gawk make ninja-build
libtool automake autoconf libexpat1-dev pkg-config python3-dev python3-pip
python3-setuptools`.

## Clone

```bash
mkdir -p ~/boat_lending
cd ~/boat_lending
git clone --branch Plane-4.6.3 --recurse-submodules --jobs 4 \
    https://github.com/ArduPilot/ardupilot.git ardupilot
```

Verify tag at HEAD:

```bash
cd ~/boat_lending/ardupilot
git tag --points-at HEAD | grep -q '^Plane-4.6.3$' && echo "ok: at Plane-4.6.3"
```

## Build

```bash
cd ~/boat_lending/ardupilot
./waf configure --board sitl
./waf plane
```

First build ≈ 8 minutes. Output binary: `build/sitl/bin/arduplane`.

## Smoke test

Start the binary directly (no MAVProxy), open TCP 5760, verify a heartbeat.

```bash
cd ~/boat_lending/ardupilot
./build/sitl/bin/arduplane -S -I0 --model quadplane --speedup 1 \
    --defaults Tools/autotest/default_params/quadplane.parm \
    >/tmp/arduplane_smoke.log 2>&1 &
sleep 10
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
hb = m.wait_heartbeat(timeout=15)
print('heartbeat type=%d autopilot=%d sys=%d mode=%d' % (
    hb.type, hb.autopilot, m.target_system, hb.custom_mode))
"
pkill -f 'build/sitl/bin/arduplane'
```

Expected: `heartbeat type=1 autopilot=3 sys=1 mode=16` (FIXED_WING vehicle class,
ArduPilotMega stack, QHOVER mode on boot).

## Interactive launch

For hands-on work use the standard wrapper:

```bash
cd ~/boat_lending/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane --console --map
```

UDP port 14550 is the default MAVProxy GCS output — this is where Mission Planner
on the Windows side will connect (WSL2 forwards localhost automatically).
