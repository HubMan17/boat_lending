# Phase 3b — Gazebo Harmonic migration for QuadPlane

## Context

После ~6 часов отладки Webots + ArduPlane QuadPlane (Phase 3.6 e2e)
упёрлись в фундаментальный барьер: **Webots не имеет fixed-wing lift
plugin**, а JSON-SITL backend через `ardupilot_vehicle_controller`
моделирует только мотор-тягу, без аэродинамики крыла. Следствия:
- QuadPlane в Webots ведёт себя как тяжёлый коптер с длинным фюзеляжем
- Hover/QLAND работают **marginally stable** — ~30-50% прогонов
  получают немотивированный attitude-кик (timing-sensitive физика при
  `--speedup 1`), диверсифицируется в 180° flip
- Transition в cruise (Phase 4) принципиально невозможен: без lift
  дрон при forward-тяге просто падает
- Ship landing (Phase 5) требует fixed-wing approach + hover transition

Roadmap `quadplane-ship-landing-roadmap.md` (2026-04) подтверждает: для
QuadPlane нужен **Gazebo Harmonic + ardupilot_gazebo plugin**. Там
полная аэродинамика, готовые примеры transition и ship-landing.

## Цель

Перенести Phase 3+ на Gazebo Harmonic в WSL2. Сохранить companion
pipeline (ArUco detection → LANDING_TARGET → PLND) как есть — он
simulator-agnostic. Переписать только launch-скрипты и world-файлы.

## Архитектура после миграции

```
Windows host
├── Mission Planner (UDP 14550)
└── companion/companion.py (Windows Python, TCP 5763 MAVLink)

WSL2 Ubuntu 22.04
├── ardupilot SITL (ArduPlane, --model JSON)
├── Gazebo Harmonic (headless, gz sim -s -r)
│   ├── world: ship_quadplane_landing.sdf
│   │   ├── QuadPlane model (alti_transition_runway derived)
│   │   ├── Ship model (из SITL_Models, box + ArUco тexture)
│   │   └── GstCameraPlugin (UDP 5600 H264 → companion)
│   └── ardupilot_gazebo plugin → JSON FDM на 127.0.0.1:9002
└── sitl_udp_bridge.py (TCP 5760 ↔ UDP 14550 для MP на Windows)
```

Companion подключается к камере через GStreamer UDP:5600 вместо
прямого TCP 5599 (Webots-специфика).

## Почему именно Gazebo Harmonic

- Официально поддерживается ArduPilot (JSON-SITL backend)
- Ogre2 renderer headless через `-s -r` флаги работает без GPU
  ускорения (WSL2 + Intel UHD N100 вытянет 10-15 FPS камеры)
- Готовый пример `SITL_Models/Gazebo/worlds/alti_transition_runway.sdf`
  — полный QuadPlane с аэродинамикой
- `SITL_Models PR #62 truck-quadplane-landing` — готовый landing на
  движущуюся платформу, меняем truck на ship
- `plane_ship_landing.lua` в ArduPilot — встроенный скрипт hold-off +
  approach + touchdown, активируется через `SHIP_ENABLE=1`

## Roadmap миграции

### P3b.1 Bringup Gazebo + SITL

Установка Gazebo Harmonic в WSL2, сборка `ardupilot_gazebo` plugin,
smoke-test с упрощённым миром (iris quadcopter) для проверки что
JSON-SITL → Gazebo работает:

```bash
# WSL2
sudo apt install libgz-sim8-dev rapidjson-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl

git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo && mkdir build && cd build
export GZ_VERSION=harmonic
cmake .. && make -j4
```

Smoke: `gz sim -s -r iris_runway.sdf` + `sim_vehicle.py -v ArduCopter
--model JSON` → HEARTBEAT на 14550. Критерий: mode GUIDED, arm,
takeoff 5, HEARTBEAT стабилен 60с.

### P3b.2 QuadPlane model + world

Склонировать `ArduPilot/SITL_Models`, вытащить
`Gazebo/worlds/alti_transition_runway.sdf` + QuadPlane model. Добавить
`GstCameraPlugin` downward-facing на UDP:5600.

Критерий: взлёт в QHOVER до 30м, стабильный hover без кика за 60с —
**это то чего нам не хватало в Webots**.

### P3b.3 Companion адаптация под GStreamer

Переключить `companion/camera_receive.py` с TCP 5599 на GStreamer UDP
5600:

```python
cap = cv2.VideoCapture(
    "udpsrc port=5600 caps=application/x-rtp,encoding-name=H264 "
    "! rtph264depay ! avdec_h264 ! videoconvert ! appsink",
    cv2.CAP_GSTREAMER)
```

Проверить что OpenCV собран с GStreamer (`cv2.getBuildInformation()`).
Если нет — поставить `opencv-contrib-python-headless` в WSL или
использовать stream relay через Python.

### P3b.4 Phase 3 e2e на Gazebo

Переписать `scripts/start_arduplane.sh` → `start_arduplane_gazebo.sh`,
`scripts/e2e_quadplane.py` — тот же pipeline (GUIDED takeoff → QLAND +
PLND) но против Gazebo. Маркер в ship-box со смещением 5м. Ожидаем:
XY error ≤ 0.5м reliable across 10/10 прогонов.

Параметры PLND оставляем как в roadmap:
```
PLND_ENABLED = 1
PLND_TYPE = 1
PLND_EST_TYPE = 1
PLND_LAG = 0.02
Q_LAND_SPEED = 30
Q_LAND_FINAL_ALT = 2
Q_LAND_FINAL_SPD = 15
```

### P3b.5 Transition test

Активировать `plane_ship_landing.lua` для hold-off логики, добавить
forward-flight waypoint. Тест: QHOVER takeoff → transition в
plane-mode → cruise 100м → transition обратно → QLAND на статичный
маркер.

### P3b.6 Ship landing (Phase 5 base)

Движущийся корабль 5 м/с, beacon (FOLL_*) + vision (LANDING_TARGET) =
hybrid подход. Референс — `SITL_Models PR #62`.

## Что сохраняется из текущей кодовой базы

- `companion/detector.py` — ArUco OpenCV, simulator-agnostic ✓
- `companion/mavlink_sender.py` — LANDING_TARGET + OPTICAL_FLOW +
  DISTANCE_SENSOR, simulator-agnostic ✓
- `companion/tracker.py` — PID tracker для GPS-denied Phase 2 ✓
- `scripts/e2e_precland.py` (Phase 1), `scripts/e2e_gps_denied.py`
  (Phase 2) — Webots + ArduCopter, **остаются рабочими** для базовых
  тестов ✓

## Что списывается / архивируется

- `webots/worlds/stage3_quadplane.wbt` — Webots QuadPlane мир (не
  удалять, держать как диагностический reference)
- `webots/protos/QuadPlane.proto` — кастомная модель с 5 моторами,
  больше не используется
- `scripts/start_arduplane.sh` — Webots-specific launcher
- `scripts/e2e_quadplane.py` в Webots-варианте — заменяется на
  Gazebo-версию

## Критичные ссылки

- [ardupilot_gazebo repo](https://github.com/ArduPilot/ardupilot_gazebo)
- [SITL_Models repo](https://github.com/ArduPilot/SITL_Models)
- [SITL_Models PR #62 truck-quadplane-landing](https://github.com/ArduPilot/SITL_Models/pull/62)
- [plane_ship_landing.lua](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/plane_ship_landing.lua)
- [Ship landing docs](https://ardupilot.org/plane/docs/common-ship-landing.html)
- [8OL-Robotics/precision-landing](https://github.com/8OL-Robotics/precision-landing)
- [rishabsingh3003/Precision_Landing_ArduPilot](https://github.com/rishabsingh3003/Precision_Landing_ArduPilot)

## Критерии готовности Phase 3b

1. Gazebo Harmonic + ardupilot_gazebo собраны в WSL2
2. QuadPlane стабильно висит 60с в QHOVER без кика (10/10 прогонов)
3. PLND замыкает контур в QLAND с XY error ≤ 0.5м (10/10, маркер 5м
   смещение)
4. Companion работает через GStreamer без изменений в логике детекции
5. Design doc в `docs/design/` — этот файл, обновлённый результатами
