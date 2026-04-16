# boat_lending

Отладочный стенд для автономной посадки QuadPlane на палубу корабля.

Полностью замкнутый симуляционный pipeline `perception → LANDING_TARGET → ArduPlane PRECLAND`, на котором можно вживую крутить параметры, логи и поведение — не выходя на реальный борт.

На старте детектор — **ArUco/AprilTag (OpenCV)**: детерминированная детекция изолирует ошибки контроля от ошибок восприятия. Реальная CNN на QR/прицел подключается позже к тому же MAVLink-интерфейсу без изменений в ArduPlane.

## Стек

- **Webots R2023b+** (Windows native) — симулятор сцены, камера, supervisor для ground truth
- **ArduPlane 4.6.3 SITL** (WSL2 Ubuntu 22.04) — автопилот с PRECLAND + Kalman target estimator
- **Python companion** (Windows) — perception + MAVLink bridge
- **Mission Planner / MAVProxy** — GCS, визуализация, настройка

## Архитектура pipeline

```
Webots (ship + ArUco-deck + QuadPlane + camera)
  │ camera frames via Webots controller
  ▼
companion.py
  ├── cv2.aruco.detectMarkers
  ├── pixel offset → body-frame metres (intrinsics + height)
  └── MAVLink LANDING_TARGET → UDP 127.0.0.1:14551
  ▼
ArduPlane SITL 4.6.3 (QuadPlane, QLAND + PRECLAND)
  │ MAVLink out → UDP 127.0.0.1:14550
  ▼
Mission Planner / MAVProxy (GCS)
```

Webots Supervisor API параллельно даёт ground truth позиций корабля и QuadPlane — отдельный канал в лог для изоляции ошибок восприятия `(gt_offset − detected_offset)` от ошибок контроля.

## Фазы

| Фаза | Сцена | Критерий успеха |
|------|-------|-----------------|
| 0 | Environment setup (SITL в WSL + Webots на Windows + Python) | smoke test каждого компонента по отдельности |
| 1 | Статичный ArUco на земле | посадка ±0.5 м от центра маркера, PL.target_x/y → 0 монотонно |
| 2 | Статичный корабль с ArUco-плитой на палубе | посадка на палубу, не в воду, PL-лог чистый |
| 3 | Движущийся корабль до 8 м/с | ≤1 м при 5 м/с; 8 м/с — допустим срыв, фиксируем лимит |
| 4 | Ветер до 8 м/с + движущийся корабль | ±1 м при ветре 8 м/с + корабль 5 м/с, NKF* без variance spikes |

## Структура репозитория

```
boat_lending/
├── companion/                    # Python perception + MAVLink bridge (Windows)
│   ├── companion.py              # основная петля: frames → detect → MAVLink
│   ├── detector.py               # OpenCV ArUco + pixel→body frame
│   ├── mavlink_sender.py         # LANDING_TARGET + DISTANCE_SENSOR пакеры
│   ├── camera_intrinsics.yaml    # fx, fy, cx, cy из Webots
│   └── groundtruth_logger.py     # параллельный лог gt vs detected
├── webots/
│   ├── worlds/                   # stage1_static.wbt, stage2_static_ship.wbt, stage3_moving_ship.wbt
│   ├── controllers/              # quadplane_camera/, ship_mover/
│   └── protos/                   # ArucoMarker.proto
├── params/
│   └── precland_quadplane.parm   # параметры ArduPlane для Mission Planner
├── scripts/
│   ├── start_sitl.sh             # запуск sim_vehicle.py (выполняется в WSL)
│   └── analyze_precland_log.py   # разбор .BIN на PL/CTUN/NKF
├── PROGRESS.md                   # статус задач между сессиями
└── README.md
```

## Разделение между Windows и WSL

| Компонент | Среда | Причина |
|-----------|-------|---------|
| Webots + мир + controllers | Windows native | GUI-симулятор, проще запускать на Windows |
| companion.py + OpenCV + pymavlink | Windows | рядом с Webots, общается по TCP/shmem |
| ArduPlane SITL + MAVProxy | WSL2 Ubuntu 22.04 | `install-prereqs-ubuntu.sh` — канонический путь |
| Mission Planner | Windows | только Windows-бинарник |

MAVLink UDP между WSL и Windows ходит через localhost без ручной настройки (WSL2 auto-forward).

## MAVLink endpoints

| Порт | Направление | Что |
|------|-------------|-----|
| 14550 | SITL → GCS | Mission Planner / MAVProxy HUD |
| 14551 | companion → SITL | `LANDING_TARGET`, `DISTANCE_SENSOR`, `HEARTBEAT` |

## Workflow

- Одна задача = одна ветка = одна сессия → коммит → merge
- Ветки: `phase0/env-setup`, `phase1/static-aruco-world`, `feat/companion-detector`, ...
- Прогресс и очередь задач — в [`PROGRESS.md`](./PROGRESS.md)
- `main` защищён, всё через PR/merge

## Явные не-цели

- Обучение CNN (отдельно после Phase 4)
- Физика воды / качка 6 DoF (Phase 5, возможно на Gazebo)
- Реальный борт
- GPS-denied терминал
