# Python companion environment on Windows

The companion process (perception + MAVLink bridge) runs on Windows next to
Webots. It lives in `companion/` and uses a dedicated virtualenv at
`companion/.venv` (gitignored).

## Python version

Python **3.14** 64-bit is used here. Any 3.11+ Python with working wheels for
`numpy`, `opencv-python`, `opencv-contrib-python` and `pymavlink` is fine —
the code does not use 3.14-specific features.

Verify the `py` launcher sees a 3.x interpreter:

```bash
py -0
```

Expected line:

```
 -V:3.14 *        Python 3.14 (64-bit)
```

## Create the virtualenv

From the repo root:

```bash
py -3.14 -m venv companion/.venv
companion/.venv/Scripts/python.exe -m pip install --upgrade pip
```

## Install dependencies

Pinned versions live in `companion/requirements.txt`:

```bash
companion/.venv/Scripts/python.exe -m pip install -r companion/requirements.txt
```

Top-level packages (transitive deps installed automatically):

| Package | Purpose |
|---------|---------|
| `numpy` | arrays / linear algebra for pixel→body frame math |
| `opencv-python` | core OpenCV (`cv2`) |
| `opencv-contrib-python` | ArUco / AprilTag detectors live in `contrib` |
| `pymavlink` | MAVLink packer for `LANDING_TARGET`, `DISTANCE_SENSOR`, `HEARTBEAT` |

`opencv-python` and `opencv-contrib-python` **must match versions** — mixing
them corrupts `cv2` imports. `requirements.txt` pins both.

## Smoke test

Confirms all packages import, ArUco is available, and a marker can be drawn:

```bash
companion/.venv/Scripts/python.exe -c "
import cv2, numpy, pymavlink
from pymavlink import mavutil
print('opencv:', cv2.__version__)
print('numpy:', numpy.__version__)
print('pymavlink:', pymavlink.__version__)
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
img = cv2.aruco.generateImageMarker(d, 0, 200)
print('aruco marker shape:', img.shape)
print('mavutil OK:', hasattr(mavutil, 'mavlink_connection'))
"
```

Pass criteria — non-empty versions, marker shape `(200, 200)`, `mavutil OK: True`.

## Activation (optional)

For interactive work the venv can be activated:

```bash
source companion/.venv/Scripts/activate        # Git Bash / MSYS
companion\.venv\Scripts\activate.bat            # cmd.exe
companion\.venv\Scripts\Activate.ps1            # PowerShell
```

Scripts and CI invoke `companion/.venv/Scripts/python.exe` directly, no
activation needed.
