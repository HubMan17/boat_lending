# Gazebo Harmonic setup — Phase 3b

One-time host setup for the headless Gazebo + `ardupilot_gazebo` plugin stack
used by Phase 3b (QuadPlane approach/landing on Gazebo with full aerodynamics).

All work lives in WSL2 Ubuntu 22.04. Nothing is installed on the Windows host
except Mission Planner and the Claude Code session logs.

## 1. Install Gazebo Harmonic from OSRF apt repo

```bash
sudo apt-get install -y lsb-release gnupg curl ca-certificates
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic
```

Verify:

```bash
gz sim --version   # Gazebo Sim, version 8.11.0
```

## 2. Install build deps for ardupilot_gazebo plugin

```bash
sudo apt-get install -y \
  libgz-sim8-dev rapidjson-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
  gstreamer1.0-plugins-ugly gstreamer1.0-tools \
  libdebuginfod1 \
  cmake build-essential git
```

`gstreamer1.0-plugins-ugly` is **critical** — it contains `x264enc`, without
which `GstCameraPlugin` fails at `failed to create GStreamer elements`.
`libdebuginfod1` is a runtime dep of `libGstCameraPlugin.so`.

## 3. Build the ardupilot_gazebo plugin

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
export GZ_VERSION=harmonic
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

Output: four `.so` files in `build/`:

- `libArduPilotPlugin.so` — JSON SITL ↔ Gazebo FDM bridge (UDP 9002/9003)
- `libGstCameraPlugin.so` — camera sensor → H.264 RTP UDP 5600
- `libCameraZoomPlugin.so` — gimbal zoom control
- `libParachutePlugin.so` — parachute actuator

## 4. Environment variables

Append to `~/.bashrc`:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
```

`~/.bashrc` is **not** sourced by non-interactive scripts. All helper scripts
in `gazebo/scripts/` export these explicitly at the top.

## 5. External model tweaks (for pre-SITL visual testing)

The shipped `ardupilot_gazebo/models/iris_with_ardupilot/model.sdf` defaults
to `<lock_step>1</lock_step>`, which makes the simulator **pause** waiting for
SITL to send the first servo packet. To browse the world visually without a
running SITL, temporarily flip it:

```bash
sed -i 's|<lock_step>1</lock_step>|<lock_step>0</lock_step>|' \
  ~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf
```

Once SITL coupling is in place (P3b.2), revert to `lock_step=1` so physics
stays in sync with autopilot timing.

Optional performance tweaks for Intel UHD-class GPU via WSLg:

```bash
# disable sun shadows and sky dome in the runway world
sed -i 's|<sky></sky>||' ~/ardupilot_gazebo/worlds/iris_runway.sdf
sed -i '0,/<cast_shadows>true</{s|<cast_shadows>true</cast_shadows>|<cast_shadows>false</cast_shadows>|}' \
  ~/ardupilot_gazebo/worlds/iris_runway.sdf
```

Long-term these belong in a repo-hosted world file derived from upstream.

## 6. Running the headless sim + camera viewer

```bash
bash gazebo/scripts/run_iris_sim.sh   # gz sim -s -r + enable_streaming + gst window
bash gazebo/scripts/move_iris.sh      # teleport demo to prove camera streams live
bash gazebo/scripts/stop_sim.sh       # kill all
```

The gst viewer uses the WSLg display. Verify WSLg first with `xeyes`
(`sudo apt-get install -y x11-apps` if missing).

## 7. SITL ↔ Gazebo coupling (P3b.2)

Use `gazebo/scripts/run_sitl_gazebo.sh` to bring up ArduCopter JSON SITL
against a headless Gazebo world, with the UDP bridge to Mission Planner.
`gazebo/scripts/smoke_sitl_coupling.sh` runs a 4-check CI-style verification
(physics ticking, bidirectional UDP on 9002/9003, SITL parses FDM, MAVLink
on TCP:5760) and exits non-zero on failure.

Two details are easy to miss and were the actual cause of the "UDP deadlock"
P3b.2 originally described:

1. **ArduCopter's main loop waits for a TCP client on :5760** before ticking.
   Until any GCS/bridge opens that socket the vehicle loop never runs, so
   no servo packets are sent to the plugin and the FDM side looks silent
   on tcpdump. `run_sitl_gazebo.sh` launches `scripts/sitl_udp_bridge.py`
   which keeps :5760 connected.
2. **Camera sensors with Ogre2 block the main sim loop under headless WSL2.**
   `~/ardupilot_gazebo/worlds/iris_runway.sdf` (iris with gimbal camera)
   hangs at "Publishing pose messages" and never advances the clock, so
   `ArduPilotPlugin::PreUpdate` is never called and servo packets pile up
   unread in the kernel socket — matching the tcpdump-vs-no-receive
   symptom from the original P3b.2 note. Use a camera-less world for the
   coupling itself. Camera rendering is addressed separately in P3b.4.

The repo-hosted `gazebo/worlds/iris_coupling.sdf` is a trimmed-down world
(`iris_with_ardupilot` only, no gimbal, no Fuel downloads) that is the
canonical environment for P3b.2 / P3b.3 coupling tests.

## 8. Known limitations

- `gz sim -s` without a GUI client sometimes holds the world paused despite
  `-r`. Workaround: `gz service -s /world/<name>/control --reqtype
  gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req "pause: false"`.
- Camera sensors in Harmonic do **not** render until `enable_streaming`
  receives `data: true`. The camera-visual scripts handle this automatically;
  the coupling scripts avoid the camera altogether.
- `~/.bashrc` is not sourced by `bash -c` or `setsid`; all scripts under
  `gazebo/scripts/` export `GZ_SIM_SYSTEM_PLUGIN_PATH` and
  `GZ_SIM_RESOURCE_PATH` explicitly at the top.
