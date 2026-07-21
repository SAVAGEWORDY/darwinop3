"""L4 football helpers with local 2D coordinates and GoToPoint."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from op3_football.l3.motion import Motion


def _normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class Pose2D:
    """Robot pose in L4 local coordinates.

    Axes convention:
    - +Y: forward from robot start heading
    - +X: right side of robot
    - heading_rad = 0 means looking along +Y
    - positive heading turns toward +X (clockwise on XY plane)
    """

    x: float = 0.0
    y: float = 0.0
    heading_rad: float = 0.0


class Navigator:
    """Dead-reckoning navigator over L3 motion primitives."""
    _global_pose = Pose2D()
    _global_pose_initialized = False

    def __init__(
        self,
        motion: Motion,
        start_x: float = 0.0,
        start_y: float = 0.0,
        start_heading_rad: float = 0.0,
        forward_speed_mps: float = 0.035,
        turn_speed_radps: float = 0.75,
        use_global_pose: bool = True,
    ) -> None:
        self.motion = motion
        self.use_global_pose = use_global_pose
        if self.use_global_pose:
            if not Navigator._global_pose_initialized:
                Navigator._global_pose = Pose2D(start_x, start_y, start_heading_rad)
                Navigator._global_pose_initialized = True
            self.pose = Pose2D(
                Navigator._global_pose.x,
                Navigator._global_pose.y,
                Navigator._global_pose.heading_rad,
            )
        else:
            self.pose = Pose2D(start_x, start_y, start_heading_rad)
        self.forward_speed_mps = forward_speed_mps
        self.turn_speed_radps = turn_speed_radps

    def set_pose(self, x: float, y: float, heading_rad: float = 0.0) -> None:
        self.pose = Pose2D(x=x, y=y, heading_rad=heading_rad)
        self._sync_global_pose()

    def reset_origin(self, x: float = 0.0, y: float = 0.0, heading_rad: float = 0.0) -> None:
        """Reset global/local origin explicitly."""
        self.pose = Pose2D(x=x, y=y, heading_rad=heading_rad)
        Navigator._global_pose = Pose2D(x=x, y=y, heading_rad=heading_rad)
        Navigator._global_pose_initialized = True

    def _sync_global_pose(self) -> None:
        if self.use_global_pose:
            Navigator._global_pose = Pose2D(
                self.pose.x,
                self.pose.y,
                self.pose.heading_rad,
            )

    def go_to_point(self, target_x: float, target_y: float) -> None:
        """Rotate to target bearing, then move straight to target point."""
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.hypot(dx, dy)
        if distance < 1e-4:
            return

        # Bearing from +Y axis with + to right side:
        # atan2(x, y) instead of atan2(y, x).
        desired_heading = math.atan2(dx, dy)
        turn_delta = _normalize_angle(desired_heading - self.pose.heading_rad)

        # Phase 1: initial turn
        if abs(turn_delta) > 0.02:
            turn_duration = abs(turn_delta) / max(self.turn_speed_radps, 1e-6)
            # Project currently has left/right turn primitives swapped at L3 level.
            if turn_delta > 0.0:
                self.motion.go("turn_left", duration=turn_duration)
            else:
                self.motion.go("turn_right", duration=turn_duration)
            self.pose.heading_rad = desired_heading

        # Phase 2: straight line motion
        move_duration = distance / max(self.forward_speed_mps, 1e-6)
        self.motion.go("forward", duration=move_duration)
        self.pose.x = target_x
        self.pose.y = target_y
        self._sync_global_pose()


class Ball:
    def __init__(self, motion: Motion) -> None:
        self._motion = motion
        self._last_seen_ts: float = 0.0
        self._last_x: float = 0.0
        self._last_y: float = 0.0
        self._last_width: float = 0.0
        self._last_confidence: float = 0.0
        self._vision_sub = None
        self._subscribe_football_vision()

    def seen(self) -> bool:
        # Prefer fresh detections from football_vision topic.
        if self._vision_sub is not None:
            return (time.time() - self._last_seen_ts) <= 0.7
        # Fallback to legacy detector if football_vision is not available.
        return self._motion.sense.ball_in_image().seen

    @property
    def x(self) -> float:
        if self._vision_sub is not None:
            return self._last_x
        return float(self._motion.sense.ball_in_image().x)

    @property
    def y(self) -> float:
        if self._vision_sub is not None:
            return self._last_y
        return float(self._motion.sense.ball_in_image().y)

    @property
    def width(self) -> float:
        if self._vision_sub is not None:
            return self._last_width
        # Legacy radius has other meaning; keep 0 to disable distance logic there.
        return 0.0

    @property
    def confidence(self) -> float:
        if self._vision_sub is not None:
            return self._last_confidence
        return 0.0

    def _subscribe_football_vision(self) -> None:
        try:
            from football_vision_msgs.msg import FieldObjects

            def _cb(msg: FieldObjects) -> None:
                if not msg.balls:
                    return
                # Pick the best candidate: higher confidence and visibly larger box.
                best = max(msg.balls, key=lambda d: float(d.confidence) + float(d.width) * 0.25)
                self._last_x = float(best.x)
                self._last_y = float(best.y)
                self._last_width = float(best.width)
                self._last_confidence = float(best.confidence)
                self._last_seen_ts = time.time()

            node = self._motion.robot.bridge._node
            self._vision_sub = node.create_subscription(
                FieldObjects,
                "/vision/field_objects",
                _cb,
                10,
            )
        except Exception:
            self._vision_sub = None


def go_to_goal(nav: Navigator) -> None:
    # Placeholder goal point in local frame.
    nav.go_to_point(0.0, 1.0)


def find_ball(motion: Motion) -> None:
    # placeholder: turn and scan head
    motion.go('turn_left', duration=0.8)
    motion.stop()
    try:
        motion.sense.look_at_image_point(0.0, 0.0)
    except Exception:
        pass


def main() -> None:
    motion = Motion()
    ball = Ball(motion)
    print("L4 football: turning to ball and approaching while visible. Ctrl+C to stop.")
    # Control thresholds in normalized image coordinates.
    center_x_tol = 0.10
    close_ball_width = 0.20  # bbox width fraction; larger means ball is near.
    search_direction = 1

    try:
        while True:
            if ball.seen():
                x_err = ball.x

                if abs(x_err) > center_x_tol:
                    # Note: in this project turn primitives are swapped at L3 level.
                    # Object on the right (x>0) -> use turn_left primitive.
                    turn_duration = 0.10 + min(0.55, abs(x_err) * 0.60)
                    if x_err > 0.0:
                        motion.go("turn_left", duration=turn_duration)
                    else:
                        motion.go("turn_right", duration=turn_duration)
                    continue

                # Ball is roughly centered horizontally: walk to it.
                if ball.width < close_ball_width:
                    motion.go("forward", duration=0.28)
                else:
                    motion.stop()
                    time.sleep(0.12)
            else:
                # If ball is lost, do short search turns; alternate directions.
                if search_direction > 0:
                    motion.go("turn_left", duration=0.25)
                else:
                    motion.go("turn_right", duration=0.25)
                search_direction *= -1
                try:
                    motion.sense.look_at_image_point(0.0, 0.0, gain=0.2)
                except Exception:
                    pass
            time.sleep(0.03)
    except KeyboardInterrupt:
        motion.estop()
        motion.robot.close()


if __name__ == '__main__':
    main()
