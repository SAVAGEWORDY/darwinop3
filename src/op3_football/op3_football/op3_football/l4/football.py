"""L4 football stub — replace with real strategy later."""

from __future__ import annotations

import time

from op3_football.l3.motion import Motion


class Ball:
    def __init__(self, motion: Motion) -> None:
        self._motion = motion

    def seen(self) -> bool:
        return self._motion.sense.ball_in_image().seen


def go_to_goal(motion: Motion) -> None:
    # placeholder: walk forward a bit
    motion.go('forward', duration=1.0)


def find_ball(motion: Motion) -> None:
    # placeholder: pivot and scan head
    motion.go('pivot_left', duration=0.8)
    motion.stop()
    try:
        motion.sense.look_at_image_point(0.0, 0.0)
    except Exception:
        pass


def main() -> None:
    motion = Motion()
    ball = Ball(motion)
    print('L4 stub running. Ctrl+C to stop.')
    try:
        while True:
            if ball.seen():
                go_to_goal(motion)
            else:
                find_ball(motion)
            time.sleep(0.1)
    except KeyboardInterrupt:
        motion.estop()
        motion.robot.close()


if __name__ == '__main__':
    main()
