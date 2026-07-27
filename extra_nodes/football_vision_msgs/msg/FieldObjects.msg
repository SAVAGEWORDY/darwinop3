# Всё, что распознала сеть на одном кадре.
#
# Каждое поле — МАССИВ объектов данного класса, который может быть ПУСТЫМ.
# Например goalposts: 0 элементов (не видно), 1 (одна штанга), 2 (обе) и т.д.
# Координаты каждого объекта см. в Detection.msg (центр, нормировка [-1, 1]).

std_msgs/Header header      # метка времени и frame_id исходного кадра камеры

# Размер исходного кадра в пикселях — если понадобится восстановить пиксельные координаты.
uint32 image_width
uint32 image_height

Detection[] balls               # мяч
Detection[] goalposts           # штанги
Detection[] robots              # роботы
Detection[] obstacles           # препятствия
Detection[] crossbars           # перекладина ворот
Detection[] l_intersections     # L-пересечения линий разметки
Detection[] t_intersections     # T-пересечения
Detection[] x_intersections     # X-пересечения
