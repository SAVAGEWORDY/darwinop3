#!/usr/bin/env python3
"""
football_vision — нода распознавания объектов футбольного поля.

Берёт кадры с камеры (топик sensor_msgs/Image), прогоняет их через обученную
YOLO-сеть и публикует в один топик всё, что видит: мяч, штанги, роботов,
препятствия, перекладину и пересечения линий разметки.

Координаты центра каждого объекта нормированы в [-1, 1]:
    x: -1 = левый край,  +1 = правый край
    y: -1 = нижний край, +1 = верхний край   (ось Y направлена ВВЕРХ)
    (0, 0) = центр кадра.

Подробности интерфейса — в AGENTS.md рядом с этим файлом.
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
import cv2

from football_vision_msgs.msg import FieldObjects, Detection

# Имя класса модели -> имя поля-массива в FieldObjects.msg
CLASS_TO_FIELD = {
    "ball": "balls",
    "goalpost": "goalposts",
    "robot": "robots",
    "obstacle": "obstacles",
    "crossbar": "crossbars",
    "L-Intersection": "l_intersections",
    "T-Intersection": "t_intersections",
    "X-Intersection": "x_intersections",
}


class FootballVisionNode(Node):
    def __init__(self):
        super().__init__("football_vision")

        # --- параметры ---
        self.declare_parameter("model_path", "")            # пусто -> модель из share пакета
        self.declare_parameter("image_topic", "/usb_cam_node/image_raw")
        self.declare_parameter("use_compressed", False)     # True -> подписка на CompressedImage
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 480)                # меньше = быстрее на слабом CPU
        self.declare_parameter("device", "cpu")
        self.declare_parameter("publish_annotated", True)   # отладочная картинка с рамками
        self.declare_parameter("log_every_sec", 5.0)

        self.image_topic = self.get_parameter("image_topic").value
        self.use_compressed = self.get_parameter("use_compressed").value
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = self.get_parameter("device").value
        self.publish_annotated = self.get_parameter("publish_annotated").value
        self.log_every = float(self.get_parameter("log_every_sec").value)

        model_path = self.get_parameter("model_path").value
        if not model_path:
            from ament_index_python.packages import get_package_share_directory
            import os
            model_path = os.path.join(
                get_package_share_directory("football_vision"), "models", "best.pt")
        self.get_logger().info(f"Загружаю модель: {model_path}")
        from ultralytics import YOLO
        self.model = YOLO(model_path)
        self.get_logger().info(f"Классы модели: {self.model.names}")

        # --- ROS I/O ---
        self.bridge = CvBridge()
        self.pub = self.create_publisher(FieldObjects, "/vision/field_objects", 10)
        self.annot_pub = (self.create_publisher(Image, "/vision/image_annotated", 1)
                          if self.publish_annotated else None)

        if self.use_compressed:
            topic = self.image_topic
            if not topic.endswith("/compressed"):
                topic = topic + "/compressed"
            self.sub = self.create_subscription(
                CompressedImage, topic, self.on_compressed, qos_profile_sensor_data)
        else:
            self.sub = self.create_subscription(
                Image, self.image_topic, self.on_image, qos_profile_sensor_data)

        self._busy = False           # пропускаем кадры, пока считается предыдущий
        self._frames = 0
        self._last_log = time.time()
        self.get_logger().info(
            f"Готов. Вход: {self.image_topic}"
            f"{'/compressed' if self.use_compressed else ''} | "
            f"выход: /vision/field_objects | imgsz={self.imgsz} conf={self.conf}")

    # ------------------------------------------------------------------ #
    def on_compressed(self, msg: CompressedImage):
        if self._busy:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)   # BGR
        if frame is None:
            self.get_logger().warn("Не удалось декодировать CompressedImage")
            return
        self._process(frame, msg.header)

    def on_image(self, msg: Image):
        if self._busy:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._process(frame, msg.header)

    # ------------------------------------------------------------------ #
    def _process(self, frame_bgr, header):
        self._busy = True
        try:
            h, w = frame_bgr.shape[:2]
            result = self.model.predict(
                frame_bgr, conf=self.conf, imgsz=self.imgsz,
                device=self.device, verbose=False)[0]

            out = FieldObjects()
            out.header = header
            out.image_width = int(w)
            out.image_height = int(h)

            for box in result.boxes:
                cls_id = int(box.cls[0])
                name = self.model.names[cls_id]
                field = CLASS_TO_FIELD.get(name)
                if field is None:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) * 0.5
                cy = (y1 + y2) * 0.5

                det = Detection()
                det.x = float(cx / w * 2.0 - 1.0)          # -1..1, вправо = +
                det.y = float(-(cy / h * 2.0 - 1.0))       # -1..1, вверх = +
                det.confidence = float(box.conf[0])
                det.width = float((x2 - x1) / w)
                det.height = float((y2 - y1) / h)
                getattr(out, field).append(det)

            self.pub.publish(out)

            if self.annot_pub is not None:
                annotated = result.plot()                  # BGR c рамками
                img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                img_msg.header = header
                self.annot_pub.publish(img_msg)

            self._frames += 1
            now = time.time()
            if now - self._last_log >= self.log_every:
                fps = self._frames / (now - self._last_log)
                nballs = len(out.balls)
                ngoals = len(out.goalposts)
                self.get_logger().info(
                    f"{fps:.1f} FPS | мяч:{nballs} штанги:{ngoals} "
                    f"роботы:{len(out.robots)} линии(L/T/X):"
                    f"{len(out.l_intersections)}/{len(out.t_intersections)}/"
                    f"{len(out.x_intersections)}")
                self._frames = 0
                self._last_log = now
        finally:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = FootballVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
