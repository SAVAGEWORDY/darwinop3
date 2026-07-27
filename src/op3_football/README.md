# op3_football

Отдельный проект футбольного стека OP3 внутри `robotis_ws/src/op3_football`.

См. [ARCHITECTURE.md](ARCHITECTURE.md).

## Пакеты

| Пакет | Роль |
|-------|------|
| `op3_football_msgs` | L1 сервисы/сообщения |
| `op3_football_l1` | C++ bridge → `/robotis/*` |
| `op3_football` | Python L2/L3/L4 |
| `op3_football_walking` | Форк `op3_walking_module` (правки походки; пока не в manager) |

## Сборка

```bash
cd ~/robotis_ws
colcon build --packages-select op3_football_msgs op3_football_l1 op3_football op3_football_walking
source install/setup.bash
```

## Запуск на роботе

```bash
# 1) штатный manager
ros2 launch op3_manager op3_manager.launch.py

# 2) L1 bridge
ros2 launch op3_football_l1 bridge.launch.py

# 3) smoke L3
ros2 run op3_football l3_smoke
```

## Пример API

```python
from op3_football.l3.motion import Motion

m = Motion()
m.joint.write(1, 2048)          # L2 ticks через L3
m.joint.write_deg(19, 10.0)     # L3 degrees
m.go('forward', duration=2.0)
m.kick('right')
m.estop()
```
