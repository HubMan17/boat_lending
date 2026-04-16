# Webots R2025a on Windows

Reproducible install of Webots R2025a on the Windows side of the stack.
Webots runs natively; SITL runs in WSL2; both talk over localhost UDP.

## Install

Download the official installer from the Cyberbotics GitHub release:

- `webots-R2025a_setup.exe` (~251 MB)
- https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a_setup.exe

Silent install to the default location (`C:\Program Files\Webots`):

```powershell
Start-Process "$env:USERPROFILE\Downloads\webots-R2025a_setup.exe" `
    -ArgumentList '/S' -Verb RunAs -Wait
```

Interactive install is equivalent — run the `.exe`, accept UAC, keep defaults.

## Layout

```
C:\Program Files\Webots\
├── msys64\mingw64\bin\webots.exe    # main binary
├── projects\samples\                # shipped examples (read-only)
├── resources\                       # protos, shaders, plugins
└── include\, lib\, src\             # C/C++ controller SDK
```

## Sysinfo check

Confirms the binary runs and OpenGL is available:

```bash
"/c/Program Files/Webots/msys64/mingw64/bin/webots.exe" --sysinfo
```

Expected output — OS, CPU, OpenAL device, and OpenGL 3.3+:

```
System: Windows 10 ...
OpenAL device: OpenAL Soft
OpenGL vendor: ...
OpenGL renderer: ...
OpenGL version: 4.x ...
```

OpenGL ≥ 3.3 is the Webots hard requirement.

## Smoke test

Open a shipped world that exercises the renderer and a camera device:

```bash
"/c/Program Files/Webots/msys64/mingw64/bin/webots.exe" --mode=run \
    "C:/Program Files/Webots/projects/samples/devices/worlds/camera.wbt"
```

Pass criteria:

- Main 3D view opens and shows the scene (robot + textured ground + objects)
- Camera overlay in the top-left shows a live rendered RGB feed
- Simulation timer in the toolbar advances (not stuck at `0:00:00:000`)
- Process `webots-bin` is alive with a working set ≈ 400 MB

Close the window to end the test.

## MAVLink across WSL↔Windows

Webots and the Python companion run on Windows; ArduPlane SITL runs in WSL2.
WSL2 auto-forwards `127.0.0.1` traffic, so no extra network setup is needed:

- SITL → GCS (Mission Planner): UDP `127.0.0.1:14550`
- Companion → SITL (`LANDING_TARGET`, `DISTANCE_SENSOR`): UDP `127.0.0.1:14551`
