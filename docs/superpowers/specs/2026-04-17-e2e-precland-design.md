# P1.10 End-to-End PrecLand Test

## Goal

Automated orchestrator that flies a GUIDED takeoff → 5 m offset → LAND
mission and verifies PrecLand brings the copter back to the marker
within ±0.5 m XY error.

## Architecture

```
  Webots (Windows)          ArduCopter SITL (WSL)
  stage1_static.wbt  ◄──►  arducopter --model webots-python
       │                        │ TCP 5760 → bridge → MP UDP 14550
       │ camera TCP 5599        │ TCP 5763 → e2e_precland.py (serial2, -C flag)
       ▼                        │
  companion.py ──UDP 14551──►  SITL MAVLink input
  (ArUco→LANDING_TARGET)
```

SITL gets an extra serial port via `-C tcp:0.0.0.0:5763` so the e2e
script coexists with Mission Planner on TCP 5760/UDP 14550.

## Script: `scripts/e2e_precland.py`

Connects to `tcp:127.0.0.1:5763` (configurable via `--mav`).

### Sequence

1. Wait HEARTBEAT, verify QUADROTOR + ARDUPILOTMEGA
2. Set GUIDED mode, arm
3. MAV_CMD_NAV_TAKEOFF to 10 m
4. Wait altitude ≥ 9 m (LOCAL_POSITION_NED, timeout 30 s)
5. SET_POSITION_TARGET_LOCAL_NED: 5 m East of current position
6. Wait arrival within 1 m (timeout 30 s)
7. Set LAND mode
8. Wait disarmed (timeout 60 s)
9. Read final LOCAL_POSITION_NED
10. Compute XY error = sqrt(x² + y²) relative to home (0,0)
11. PASS if ≤ 0.5 m, FAIL otherwise

### CLI

```
python scripts/e2e_precland.py [--mav tcp:127.0.0.1:5763] [--alt 10] [--offset 5]
```

### Exit codes

- 0 = PASS
- 1 = FAIL (error > threshold or timeout)

## Changes to `start_arducopter.sh`

Add `-C tcp:0.0.0.0:5763` to the arducopter invocation for serial2.

## Success criterion

XY landing error ≤ 0.5 m from ArUco marker center.
