# PROGRESS

Трекер задач проекта `boat_lending`. Обновляется каждую сессию — одна задача на сессию, коммит, `/clear`, новая сессия.

Формат дат: `YYYY-MM-DD`.

## Текущая задача

_(нет активной задачи — ожидание запуска следующей сессии)_

## Завершённые задачи

| Дата | Задача | Ветка | Коммит |
|------|--------|-------|--------|
| 2026-04-16 | Bootstrap репозитория: структура, README, .gitignore, PROGRESS.md | `main` | _см. git log_ |

## Очередь задач

### Phase 0 — Environment setup
- [ ] **P0.1** Установка ArduPlane SITL 4.6.3 в WSL2 Ubuntu 22.04 (`install-prereqs-ubuntu.sh`, проверка `sim_vehicle.py -v ArduPlane -f quadplane`)
- [ ] **P0.2** Установка Webots R2023b+ на Windows, smoke test на `projects/samples/devices/worlds/camera.wbt`
- [ ] **P0.3** Python-окружение на Windows: `pymavlink`, `opencv-python`, `opencv-contrib-python`, `numpy`, virtualenv в `companion/.venv`
- [ ] **P0.4** Mission Planner для Windows
- [ ] **P0.5** End-to-end smoke test: SITL взлетает QHOVER, Mission Planner видит HUD, Webots рендерит мир

### Phase 1 — Статичный ArUco на земле
- [ ] **P1.1** `webots/worlds/stage1_static.wbt` (плоскость 50×50, ArUco-плита DICT_4X4_50 id=0, QuadPlane с downward-камерой)
- [ ] **P1.2** `webots/controllers/quadplane_camera/` — публикует кадры в companion
- [ ] **P1.3** `companion/detector.py` — OpenCV ArUco + pinhole pixel→body frame
- [ ] **P1.4** `companion/mavlink_sender.py` — пакер `LANDING_TARGET` + `DISTANCE_SENSOR` + `HEARTBEAT`
- [ ] **P1.5** `companion/companion.py` — основная петля 20 Hz, CLI-флаг `--stage`
- [ ] **P1.6** `params/precland_quadplane.parm` — набор параметров PLND_*, Q_LAND_SPEED, LOG_BITMASK
- [ ] **P1.7** `scripts/start_sitl.sh` — обёртка `sim_vehicle.py` с нужными флагами
- [ ] **P1.8** `companion/groundtruth_logger.py` + Webots supervisor API для gt-логов
- [ ] **P1.9** `scripts/analyze_precland_log.py` — разбор `.BIN` (PL/CTUN/NKF)
- [ ] **P1.10** End-to-end прогон: AUTO takeoff → GUIDED hover 5m offset → QLAND → критерий ±0.5 м

### Phase 2 — Статичный корабль
- [ ] **P2.1** `webots/worlds/stage2_static_ship.wbt` (корпус корабля 40×8×3 м, палуба, ArUco-плита 2×2 м, вода)
- [ ] **P2.2** Rangefinder через MAVLink `DISTANCE_SENSOR` от companion (не `SIM_SONAR`)
- [ ] **P2.3** End-to-end прогон: критерий — посадка на палубу, не в воду

### Phase 3 — Движущийся корабль
- [ ] **P3.1** `webots/controllers/ship_mover/` — supervisor, прямое движение 1/3/5/8 м/с
- [ ] **P3.2** Kalman-оценка скорости цели в companion (опционально) или проверка что ArduPlane сам дифференцирует
- [ ] **P3.3** AUTO rendezvous waypoint → GUIDED → QLAND + PRECLAND
- [ ] **P3.4** Критерий: ≤1 м при 5 м/с; фиксируем лимит на 8 м/с

### Phase 4 — Ветер
- [ ] **P4.1** Сценарии `SIM_WIND_SPD/DIR`: корабль 0 + ветер, корабль 5 по ветру, корабль 5 против ветра
- [ ] **P4.2** Тюнинг `Q_WVANE_ENABLE`, `Q_P_POSXY_P`, `PLND_LAG` при необходимости
- [ ] **P4.3** Критерий: ±1 м при ветре 8 м/с + корабль 5 м/с, `NKF*` без variance spikes

## Открытые вопросы и блокеры

_(пока нет)_

## Лог решений

| Дата | Решение | Причина |
|------|---------|---------|
| 2026-04-16 | SITL в WSL2, Webots на Windows | WSL2 — канонический путь для ArduPilot, Webots проще ставить native; UDP localhost auto-forward |
| 2026-04-16 | ArUco, не CNN, на старте | детерминированная детекция изолирует ошибки контура контроля от perception |
| 2026-04-16 | `LANDING_TARGET` (MAVLink #149), не прямое управление | все контуры (Kalman estimator, velocity damper, weathervane) уже в ArduPlane 4.6.3 — не дублируем |
| 2026-04-16 | Rangefinder через MAVLink `DISTANCE_SENSOR` от companion, не `SIM_SONAR` | на реальном борту будет так же — один канал, ноль расхождений sim↔real |
| 2026-04-16 | Одна задача на сессию + GitHub branch-per-task | запрос пользователя: минимизировать drift, атомарные коммиты, чистый history |
