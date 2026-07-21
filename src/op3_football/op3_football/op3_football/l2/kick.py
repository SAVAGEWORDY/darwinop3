"""Kick API via OP3 action pages (demo-compatible)."""

from __future__ import annotations

import time
from std_msgs.msg import Int32

from op3_football.l1.bridge_client import BridgeClient
from op3_football.l2.joint import JointAPI


class KickAPI:
    def __init__(self, bridge: BridgeClient, joint: JointAPI) -> None:
        self._bridge = bridge
        self._joint = joint
        self._page_pub = self._bridge._node.create_publisher(Int32, "/robotis/action/page_num", 10)
        self._right_kick_page = 121
        self._left_kick_page = 120
        self._kick_wait_s = 2.0
        self._post_kick_stand_page = 9
        self._post_stand_wait_s = 1.2

    def right(self) -> None:
        self._play_page(self._right_kick_page)

    def left(self) -> None:
        self._play_page(self._left_kick_page)

    def _play_page(self, page: int) -> None:
        self._bridge.walking_command("stop")
        self._bridge.set_module("action_module")
        time.sleep(0.15)
        msg = Int32()
        msg.data = int(page)
        self._page_pub.publish(msg)
        time.sleep(self._kick_wait_s)

        # Return to stable standing page used by OP3 demo.
        if self._post_kick_stand_page > 0:
            msg.data = int(self._post_kick_stand_page)
            self._page_pub.publish(msg)
            time.sleep(self._post_stand_wait_s)
