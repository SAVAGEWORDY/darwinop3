#!/usr/bin/env python3
"""
HTTP MJPEG стрим с аннотированным видео из football_vision.

По умолчанию:
  - подписка: /vision/image_annotated
  - адрес:    0.0.0.0
  - порт:     8080

Открыть в браузере:
  http://<LOCAL_IP>:8080/
или напрямую поток:
  http://<LOCAL_IP>:8080/stream.mjpg
"""
import threading
from http import server
from socketserver import ThreadingMixIn
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class SharedFrame:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._seq = 0

    def update(self, jpeg_bytes: bytes) -> None:
        with self._lock:
            self._jpeg = jpeg_bytes
            self._seq += 1

    def get(self):
        with self._lock:
            return self._jpeg, self._seq


class ThreadedHTTPServer(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class StreamNode(Node):
    def __init__(self) -> None:
        super().__init__("football_vision_stream")
        self.declare_parameter("image_topic", "/vision/image_annotated")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("jpeg_quality", 80)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.bridge = CvBridge()
        self.shared = SharedFrame()
        self.client_count = 0
        self.client_count_lock = threading.Lock()

        self.sub = self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data
        )

        handler_cls = self._make_handler()
        self.httpd = ThreadedHTTPServer((self.host, self.port), handler_cls)
        self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.server_thread.start()

        self.get_logger().info(
            f"HTTP stream started: http://{self.host}:{self.port}/ (topic: {self.image_topic})"
        )

    def destroy_node(self):
        try:
            if hasattr(self, "httpd") and self.httpd is not None:
                self.httpd.shutdown()
                self.httpd.server_close()
        finally:
            super().destroy_node()

    def _on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, enc = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if ok:
            self.shared.update(enc.tobytes())

    def _make_handler(self):
        node = self
        boundary = b"--frame"

        class StreamHandler(server.BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                # Не шумим в stdout, логирование остаётся в ROS.
                return

            def do_GET(self):
                if self.path == "/" or self.path == "/index.html":
                    body = (
                        "<html><head><title>OP3 YOLO Stream</title></head>"
                        "<body style='margin:0;background:#111;color:#eee;'>"
                        "<div style='padding:10px;font-family:sans-serif;'>"
                        "<h3 style='margin:0 0 8px 0;'>OP3 YOLO Stream</h3>"
                        "<p style='margin:0 0 8px 0;'>"
                        "Open stream: <a style='color:#8cf' href='/stream.mjpg'>/stream.mjpg</a>"
                        "</p></div>"
                        "<img src='/stream.mjpg' style='max-width:100%;height:auto;display:block;'/>"
                        "</body></html>"
                    ).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path == "/health":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(b"ok")
                    return

                if self.path != "/stream.mjpg":
                    self.send_error(404)
                    return

                with node.client_count_lock:
                    node.client_count += 1
                    clients_now = node.client_count
                node.get_logger().info(f"Client connected to stream ({clients_now} total)")

                try:
                    self.send_response(200)
                    self.send_header(
                        "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                    )
                    self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                    self.send_header("Pragma", "no-cache")
                    self.end_headers()

                    last_seq = -1
                    while rclpy.ok():
                        jpeg, seq = node.shared.get()
                        if jpeg is None or seq == last_seq:
                            # Если нового кадра нет — не забиваем CPU.
                            threading.Event().wait(0.02)
                            continue

                        last_seq = seq
                        self.wfile.write(boundary + b"\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    with node.client_count_lock:
                        node.client_count = max(0, node.client_count - 1)
                        clients_left = node.client_count
                    node.get_logger().info(f"Client disconnected ({clients_left} total)")

        return StreamHandler


def main(args=None):
    rclpy.init(args=args)
    node = StreamNode()
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
