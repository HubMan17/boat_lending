# P1.9 — analyze_precland_log.py

BIN log analysis script for ArduCopter precision landing evaluation.

## Input

- CLI arg: path to `.BIN` file
- `--output-dir` (optional, default: same directory as BIN file)
- `--show` flag to open plots interactively (default: save only)

## Parsing (pymavlink DFReader)

| Message | Fields used | Purpose |
|---------|-------------|---------|
| PL | TimeUS, pX, pY, pZ, mX, mY, mZ, Heal | PrecLand estimator state |
| CTUN | TimeUS, Alt, DAlt, TAlt | Altitude control |
| XKF5 | TimeUS, NVI, NPI | EKF velocity/position innovations |

TimeUS converted to seconds relative to first PL message.

## Metrics (stdout)

1. **Final XY error** — distance from last healthy PL estimate to origin (marker)
2. **Max PL error** — max distance between PL estimated (pX,pY) and measured (mX,mY) while healthy
3. **Mean PL error** — mean of the same
4. **First acquisition altitude** — CTUN.Alt at time of first PL message with Heal > 0
5. **Landing duration** — time from first PL.Heal > 0 to last PL message
6. **EKF max velocity innovation** — max(abs(XKF5.NVI)) during landing window

## Plots (3 PNG files)

### 1. `trajectory_xy.png`
- Scatter/line of PL.pX vs PL.pY colored by time
- Marker position at (0, 0) as red X
- Equal aspect ratio
- Title: "XY Trajectory (PrecLand Estimator)"

### 2. `altitude_vs_time.png`
- CTUN.Alt (actual) and CTUN.DAlt (desired) vs time
- Vertical line at first PL acquisition
- Title: "Altitude vs Time"

### 3. `pl_error_vs_time.png`
- sqrt((pX-mX)^2 + (pY-mY)^2) vs time
- Background shading: green where Heal > 0, red where Heal == 0
- Title: "PrecLand Estimator Error"

## Dependencies

- `pymavlink` (already in requirements.txt) — DFReader for BIN parsing
- `matplotlib` — new dependency, add to requirements.txt

## Output structure

```
<output-dir>/
  trajectory_xy.png
  altitude_vs_time.png
  pl_error_vs_time.png
```

Metrics printed to stdout in key: value format for easy grep.

## Error handling

- Exit with message if BIN has no PL messages (no precision landing data)
- Skip XKF5 plot section if no XKF5 messages (older firmware)
- Skip CTUN metrics if no CTUN messages
