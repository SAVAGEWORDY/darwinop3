"""L2 facade — skills API without exposing ROS."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from op3_football.l1.bridge_client import BridgeClient
from op3_football.l2.joint import JointAPI
from op3_football.l2.kick import KickAPI
from op3_football.l2.walk import PostureAPI, TurnAPI, WalkAPI

if TYPE_CHECKING:
    from op3_football.l3.coefs import WalkCoefs


class Robot:
    def __init__(self) -> None:
        self._bridge: Optional[BridgeClient] = None
        self.joint: Optional[JointAPI] = None
        self.walk: Optional[WalkAPI] = None
        self.kick: Optional[KickAPI] = None
        self._posture: Optional[PostureAPI] = None
        self._turn: Optional[TurnAPI] = None

    def start(self) -> 'Robot':
        self._bridge = BridgeClient.instance()
        self.joint = JointAPI(self._bridge)
        self.walk = WalkAPI(self._bridge)
        self.kick = KickAPI(self._bridge, self.joint)
        self._posture = PostureAPI(self._bridge, self.joint)
        self._turn = TurnAPI(self.walk)
        return self

    def close(self) -> None:
        if self._bridge is not None:
            self._bridge.shutdown()
            self._bridge = None

    # --- posture / locomotion thin wrappers ---

    def stand(self) -> None:
        assert self._posture
        self._posture.stand()

    def sit(self) -> None:
        assert self._posture
        self._posture.sit()

    def turn(self, coefs: 'WalkCoefs') -> None:
        assert self._turn
        self._turn.turn(coefs)

    def pivot(self, coefs: 'WalkCoefs') -> None:
        assert self._turn
        self._turn.pivot(coefs)

    def estop(self) -> None:
        assert self._bridge
        self._bridge.emergency_stop()

    def torque_on(self) -> None:
        assert self._bridge
        self._bridge.set_torque('on')

    def torque_off(self) -> None:
        assert self._bridge
        self._bridge.set_torque('off')

    def set_led(self, r: int, g: int, b: int) -> None:
        assert self._bridge
        self._bridge.set_led(r, g, b)

    def set_module(self, name: str) -> None:
        assert self._bridge
        self._bridge.set_module(name)

    @property
    def imu(self):
        assert self._bridge
        return self._bridge.imu

    @property
    def button(self) -> str:
        assert self._bridge
        return self._bridge.button

    @property
    def bridge(self) -> BridgeClient:
        assert self._bridge
        return self._bridge
