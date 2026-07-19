"""L3 perception helpers — image frame / head angles (not global field yet)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from op3_football.l1.bridge_client import BridgeClient
from op3_football.l2.robot import Robot


@dataclass
class BallInImage:
    """Normalized ball position in camera frame (approx. from detector)."""
    x: float  # [-1, 1] left..right
    y: float  # [-1, 1] top..bottom
    radius: float
    seen: bool


@dataclass
class HeadAngles:
    pan_rad: float
    tilt_rad: float


class Sense:
    def __init__(self, robot: Robot) -> None:
        self._robot = robot
        self._bridge: BridgeClient = robot.bridge
        self._last_ball = BallInImage(0.0, 0.0, 0.0, False)
        self._ball_sub = None
        self._try_subscribe_ball()

    def _try_subscribe_ball(self) -> None:
        try:
            from op3_ball_detector_msgs.msg import CircleSetStamped

            def _cb(msg: CircleSetStamped) -> None:
                if not msg.circles:
                    self._last_ball = BallInImage(0.0, 0.0, 0.0, False)
                    return
                c = msg.circles[0]
                # Point: (x, y) center in image pixels, z = radius
                self._last_ball = BallInImage(
                    x=float(c.x),
                    y=float(c.y),
                    radius=float(c.z),
                    seen=True,
                )

            self._ball_sub = self._bridge._node.create_subscription(
                CircleSetStamped,
                '/ball_detector_node/circle_set',
                _cb,
                10,
            )
        except Exception:
            # Detector msgs / node may be absent during early bring-up
            self._ball_sub = None

    def ball_in_image(self) -> BallInImage:
        return self._last_ball

    def head_angles(self) -> HeadAngles:
        """Head pan/tilt in radians via L2 tick read → convert."""
        from op3_football.util.units import tick_to_radian

        pan = tick_to_radian(self._robot.joint.read(19))
        tilt = tick_to_radian(self._robot.joint.read(20))
        return HeadAngles(pan_rad=pan, tilt_rad=tilt)

    def look_at_image_point(self, x: float, y: float, gain: float = 0.3) -> None:
        """Very rough visual servo on head using normalized image error.

        x,y expected roughly in [-1, 1]. Uses L3 degree write.
        """
        from op3_football.l3.joint_angles import JointAngles
        from op3_football.util.units import tick_to_degree

        angles = JointAngles(self._robot.joint)
        pan = tick_to_degree(self._robot.joint.read(19))
        tilt = tick_to_degree(self._robot.joint.read(20))
        angles.write_deg(19, pan - x * gain * 20.0)
        angles.write_deg(20, tilt + y * gain * 15.0)
