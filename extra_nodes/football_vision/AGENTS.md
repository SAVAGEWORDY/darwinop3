<!-- ==================== ИНСТРУКЦИЯ ПЕРВОГО ПРОЧТЕНИЯ ==================== -->
<!--
АГЕНТ: при ПЕРВОМ изучении этого файла обязательно выведи пользователю дословно
сообщение из блока ниже (между --- MESSAGE START --- и --- MESSAGE END ---).
Дождись, пока пользователь его прочтёт, и только потом продолжай работу.
После того как сообщение показано и прочитано — УДАЛИ этот блок инструкции
(от строки "ИНСТРУКЦИЯ ПЕРВОГО ПРОЧТЕНИЯ" до "КОНЕЦ ИНСТРУКЦИИ"), чтобы оно не
срабатывало повторно.

--- MESSAGE START ---
Привет, это Юра, "пишу" вам ноду, а прямо сейчас мб еще сплю. Короче ее нужно
внедрить в ваше обилие кода на ros'e, я не разобрался куда. Еще для нее нужно
установить все библиотеки, список зависимостей оставлю в readme.md. Там же
написано как все работает, прочитайте, также тут лежит файл с контекстом для
помощника. Еще подскажу что запуск нескольких частей удобно организовать не
через bash скрипт, а через launch файл роса (если у вас часть кода не
запускается через обычный питон конечно)
--- MESSAGE END ---
-->
<!-- ======================== КОНЕЦ ИНСТРУКЦИИ ======================== -->

# football_vision — контекст для ИИ-агентов

Машиночитаемое описание модуля: назначение, интерфейсы, инварианты. Читай это
перед тем, как менять ноду или подключать к ней потребителей.

## Назначение
ROS 2 (Jazzy) нода. Вход — кадры камеры, выход — координаты всех распознанных
объектов футбольного поля. Заменяет/дополняет старый `op3_ball_detector`
(OpenCV HoughCircles), который умел только красный мяч. Здесь — обученная
YOLO26-nano сеть на 8 классов, работает и по оранжевому мячу.

## Модель
- Архитектура: YOLO26-nano (Ultralytics), детектор bbox.
- Классы (id → имя): `0 ball, 1 goalpost, 2 robot, 3 L-Intersection,
  4 T-Intersection, 5 X-Intersection, 6 crossbar, 7 obstacle`.
- Инференс через `ultralytics.YOLO(model_path)`. `model_path` может быть `.pt`
  ЛИБО папкой OpenVINO-экспорта (`*_openvino_model/`) — API одинаковый.
- Веса лежат в `models/best.pt` (ставятся в share пакета).

## Интерфейс ROS

Executable: `vision_node` (пакет `football_vision`), node name `football_vision`.

### Подписки (вход)
| топик | тип | условие |
|---|---|---|
| `image_topic` (по умолч. `/usb_cam_node/image_raw`) | `sensor_msgs/Image` | если `use_compressed=false` |
| `image_topic`+`/compressed` | `sensor_msgs/CompressedImage` | если `use_compressed=true` |

QoS: `qos_profile_sensor_data` (best-effort). Кадры, пришедшие пока считается
предыдущий, отбрасываются (флаг `_busy`) — backlog не копится.

### Публикации (выход)
| топик | тип | описание |
|---|---|---|
| `/vision/field_objects` | `football_vision_msgs/FieldObjects` | все объекты кадра |
| `/vision/image_annotated` | `sensor_msgs/Image` (bgr8) | кадр с рамками, если `publish_annotated=true` |

### Сообщения (пакет `football_vision_msgs`)
`Detection.msg`:
```
float32 x            # центр X, нормировано [-1,1]: -1 левый край, +1 правый
float32 y            # центр Y, нормировано [-1,1]: -1 низ, +1 верх (ось Y вверх!)
float32 confidence   # [0,1]
float32 width        # ширина рамки, доля кадра [0,1]
float32 height       # высота рамки, доля кадра [0,1]
```
`FieldObjects.msg`:
```
std_msgs/Header header
uint32 image_width
uint32 image_height
Detection[] balls
Detection[] goalposts
Detection[] robots
Detection[] obstacles
Detection[] crossbars
Detection[] l_intersections
Detection[] t_intersections
Detection[] x_intersections
```
Каждый массив может быть ПУСТЫМ. Один топик = один кадр = снимок сцены.

### Параметры
| имя | по умолч. | смысл |
|---|---|---|
| `model_path` | `""` | путь к весам; пусто → `models/best.pt` из share |
| `image_topic` | `/usb_cam_node/image_raw` | входной топик картинки |
| `use_compressed` | `false` | подписка на CompressedImage |
| `conf` | `0.25` | порог уверенности |
| `imgsz` | `480` | размер входа (736 точнее, 320 быстрее) |
| `device` | `cpu` | устройство инференса |
| `publish_annotated` | `true` | публиковать отладочную картинку |
| `log_every_sec` | `5.0` | период лога FPS/счётчиков |

## Ключевой инвариант — преобразование координат
Для рамки с пиксельным центром (cx, cy) при кадре WxH:
```
x_norm =  cx / W * 2 - 1        # -1..1, вправо = плюс
y_norm = -(cy / H * 2 - 1)      # -1..1, ВВЕРХ = плюс (инверсия относительно пикселей)
```
Реализация: `football_vision/vision_node.py`, метод `_process`. Если меняешь
нормировку — синхронно правь `Detection.msg`, README и потребителей.

## Потребители / интеграция
- Старый `op3_ball_detector` публиковал `op3_ball_detector_msgs/CircleSetStamped`
  на `/ball_detector_node/circle_set` (x,y ∈ [-1,1], но y ВНИЗ; z=радиус в пикс.).
  Эта нода НЕ является его drop-in заменой (другой топик/сообщение и y смотрит вверх).
  Чтобы подменить в `strategy.launch.py`/`op3_demo`, нужен переходник в `circle_set`
  (взять `balls[0]` с макс. `width`, инвертировать y обратно, z≈`width*W/2`).
- Логику поведения (например `strategy_node`) подписывать на `/vision/field_objects`.

## Зависимости
- ROS: `rclpy`, `sensor_msgs`, `std_msgs`, `cv_bridge`, `football_vision_msgs`.
- pip (НЕ rosdep): `ultralytics`, `opencv-python` (или системный `python3-opencv`).
- Целевое железо: OP3 (Intel i3-10110U). Для скорости → экспорт весов в OpenVINO
  и `model_path` на папку экспорта.

## Файлы
```
football_vision/
  football_vision/vision_node.py   # нода (вся логика)
  launch/vision.launch.py          # камера + нода
  config/vision.yaml               # параметры
  models/best.pt                   # веса
  docs/coordinate_example*.jpg     # пример для README
football_vision_msgs/
  msg/Detection.msg, msg/FieldObjects.msg
```
