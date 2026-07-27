"""Hidden ROS client for /op3_football/* L1 services."""

from __future__ import annotations

import threading
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, String

from op3_football_msgs.srv import (
    EmptyTrigger,
    JointRead,
    JointWrite,
    JointWriteMany,
    SetLed,
    SetModule,
    SetTorque,
    SetWalkingParams,
    WalkingCommand,
)
from op3_walking_module_msgs.msg import WalkingParam


class BridgeClient:
    """Singleton-ish runtime: one node + spinner thread for the whole process."""

    _instance: Optional['BridgeClient'] = None

    def __init__(self) -> None:
        if not rclpy.ok():
            rclpy.init()
        self._node = Node('op3_football_py')
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._imu: Optional[Imu] = None
        self._button: str = ''
        self._node.create_subscription(Imu, '/op3_football/imu', self._on_imu, 10)
        self._node.create_subscription(String, '/op3_football/button', self._on_button, 10)
        self._action_pub = self._node.create_publisher(Int32, '/robotis/action/page_num', 10)

        self._cli_write = self._node.create_client(JointWrite, '/op3_football/joint/write')
        self._cli_read = self._node.create_client(JointRead, '/op3_football/joint/read')
        self._cli_write_many = self._node.create_client(JointWriteMany, '/op3_football/joint/write_many')
        self._cli_module = self._node.create_client(SetModule, '/op3_football/module/set')
        self._cli_walk_cmd = self._node.create_client(WalkingCommand, '/op3_football/walking/command')
        self._cli_walk_params = self._node.create_client(SetWalkingParams, '/op3_football/walking/set_params')
        self._cli_ini = self._node.create_client(EmptyTrigger, '/op3_football/base/ini_pose')
        self._cli_estop = self._node.create_client(EmptyTrigger, '/op3_football/emergency_stop')
        self._cli_led = self._node.create_client(SetLed, '/op3_football/led/set')
        self._cli_torque = self._node.create_client(SetTorque, '/op3_football/torque/set')

    @classmethod
    def instance(cls) -> 'BridgeClient':
        if cls._instance is None:
            cls._instance = BridgeClient()
        return cls._instance

    def _on_imu(self, msg: Imu) -> None:
        self._imu = msg

    def _on_button(self, msg: String) -> None:
        self._button = msg.data

    def _call(self, client, request, timeout: float = 2.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Service not available: {client.srv_name}')
        future = client.call_async(request)
        # spin is in background thread; wait on future
        event = threading.Event()

        def _done(_):
            event.set()

        future.add_done_callback(_done)
        if not event.wait(timeout):
            raise TimeoutError(f'Service call timed out: {client.srv_name}')
        return future.result()

    def joint_write(self, joint_id: int, value: int) -> None:
        req = JointWrite.Request()
        req.id = int(joint_id)
        req.value = int(value)
        res = self._call(self._cli_write, req)
        if not res.success:
            raise RuntimeError(res.message)

    def joint_read(self, joint_id: int) -> int:
        req = JointRead.Request()
        req.id = int(joint_id)
        res = self._call(self._cli_read, req)
        if not res.success:
            raise RuntimeError(res.message)
        return int(res.value)

    def joint_write_many(self, ids: Sequence[int], values: Sequence[int]) -> None:
        req = JointWriteMany.Request()
        req.ids = [int(i) for i in ids]
        req.values = [int(v) for v in values]
        res = self._call(self._cli_write_many, req)
        if not res.success:
            raise RuntimeError(res.message)

    def set_module(self, module_name: str) -> None:
        req = SetModule.Request()
        req.module_name = module_name
        res = self._call(self._cli_module, req)
        if not res.success:
            raise RuntimeError(res.message)

    def walking_command(self, command: str) -> None:
        req = WalkingCommand.Request()
        req.command = command
        res = self._call(self._cli_walk_cmd, req)
        if not res.success:
            raise RuntimeError(res.message)

    def set_walking_params(self, params: WalkingParam) -> None:
        req = SetWalkingParams.Request()
        req.params = params
        res = self._call(self._cli_walk_params, req)
        if not res.success:
            raise RuntimeError(res.message)

    def ini_pose(self) -> None:
        res = self._call(self._cli_ini, EmptyTrigger.Request())
        if not res.success:
            raise RuntimeError(res.message)

    def emergency_stop(self) -> None:
        res = self._call(self._cli_estop, EmptyTrigger.Request())
        if not res.success:
            raise RuntimeError(res.message)

    def set_led(self, red: int, green: int, blue: int) -> None:
        req = SetLed.Request()
        req.red = int(red)
        req.green = int(green)
        req.blue = int(blue)
        res = self._call(self._cli_led, req)
        if not res.success:
            raise RuntimeError(res.message)

    def set_torque(self, command: str) -> None:
        req = SetTorque.Request()
        req.command = command
        res = self._call(self._cli_torque, req)
        if not res.success:
            raise RuntimeError(res.message)

    def play_action_page(self, page: int, wait_s: float = 0.0) -> None:
        self.set_module('action_module')
        msg = Int32()
        msg.data = int(page)
        self._action_pub.publish(msg)
        if wait_s > 0.0:
            import time
            time.sleep(wait_s)

    @property
    def imu(self) -> Optional[Imu]:
        return self._imu

    @property
    def button(self) -> str:
        return self._button

    def shutdown(self) -> None:
        self._executor.shutdown()
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        BridgeClient._instance = None
