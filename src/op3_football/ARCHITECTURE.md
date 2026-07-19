# OP3 Football — архитектура

Многоуровневый стек поверх штатного ROS2 OP3 (`op3_manager` + модули).  
ROS скрыт внутри L1; L2/L3/L4 — чистый Python API.

## Слои

```
L4  football        # стратегия (пока заглушка)
L3  motion / sense  # go(), подбор коэффициентов, углы/радианы, кадр камеры
L2  skills          # walk/stand/sit/turn/pivot/kick, joint 0..4095, e-stop
L1  bridge (C++)    # обёртка над /robotis/*, конвертация ticks↔rad
HW  op3_manager + Dynamixel + OpenCR + camera
```

| Уровень | Язык | Пакет | Что делает |
|--------|------|-------|------------|
| L1 | C++ | `op3_football_l1` + `op3_football_msgs` | Сервисы/топики `/op3_football/*`, перевод 0–4095 ↔ радианы ROS |
| L2 | Python | `op3_football.l2` | Скиллы без коэффициентов снаружи (коэфы приходят с L3) |
| L3 | Python | `op3_football.l3` | `go()`, пресеты коэффициентов, `write_deg`/`write_rad`, восприятие |
| L4 | Python | `op3_football.l4` | Заглушка футбола |
| walking fork | C++ | `op3_football_walking` | Копия `op3_walking_module` для правок походки (пока не воткнута в manager) |

## L1 API (`/op3_football/...`)

- `joint/write` `(id, value 0..4095)` → `/robotis/direct_control/set_joint_states`
- `joint/read` `(id)` ← `/robotis/present_joint_states`
- `module/set` → `/robotis/set_present_ctrl_modules` / `enable_ctrl_module`
- `walking/command` `start|stop` → `/robotis/walking/command`
- `walking/set_params` → `/robotis/walking/set_params`
- `base/ini_pose`, `torque`, `led`, `emergency_stop`
- топики: `imu`, `button`, `joint_ticks`

Конвертация XM430: `0 rad ↔ 2048`, `±π ↔ 0/4095`.

## L2 API (Python)

```python
from op3_football import Robot
r = Robot(); r.start()
r.joint.write(1, 2048)       # id, ticks
r.joint.read(1)
r.walk.start(coefs)          # coefs с L3
r.walk.stop()
r.stand(); r.sit()
r.turn(coefs); r.pivot(coefs)
r.kick.right() / r.kick.left()   # свои траектории
r.estop()                        # + кнопка OpenCR
r.imu; r.led; r.button; r.torque_on()
```

## L3 API (Python)

```python
r.go()                       # уверенная ходьба вперёд с пресетом
r.go(mode="turn_left")
r.joint.write_deg(1, 10.0)
r.joint.write_rad(1, 0.1)
r.sense.ball_in_image()      # кадр / угол головы (обёртка)
# коэффициенты живут в l3/coefs.py — отдельно от стратегии
```

## L4 (заглушка)

```python
if ball.seen():
    go_to_goal()
else:
    find_ball()
```

## Арбитраж модулей

Переключение `direct_control` / `walking_module` / `base_module` / `head_control`  
делается внутри L2-скиллов и собирается на L3 (`go` включает walking и т.д.).  
L4 модули не трогает.

## Запуск

```bash
# штатный manager (пока)
ros2 launch op3_manager op3_manager.launch.py
ros2 run op3_football_l1 bridge
ros2 run op3_football l3_smoke   # простой тест L3
```

## Дальше

1. Подключить `op3_football_walking` в форк manager  
2. Доработать kick-траектории на железе  
3. Симулятор (Gazebo/Webots)  
4. YOLO вместо color ball detector  
5. Настоящий L4
