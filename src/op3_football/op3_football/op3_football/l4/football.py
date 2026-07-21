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

    def seen(self) -> bool:
        return self._motion.sense.ball_in_image().seen


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
    nav = Navigator(motion=motion, start_x=0.0, start_y=0.0, start_heading_rad=0.0)
    ball = Ball(motion)
    print('L4 stub running. Ctrl+C to stop.')
    try:
        while True:
            if ball.seen():
                go_to_goal(nav)
            else:
                find_ball(motion)
            time.sleep(0.1)
    except KeyboardInterrupt:
        motion.estop()
        motion.robot.close()


if __name__ == '__main__':
    main()
