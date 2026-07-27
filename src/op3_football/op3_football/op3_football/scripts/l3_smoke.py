"""Manual smoke test for L3 (requires op3_manager + op3_football_l1 bridge)."""

from __future__ import annotations

import time

from op3_football.l3.motion import Motion


def main() -> None:
    print('L3 smoke: stand -> go(forward) 2s -> stop -> kick.right placeholder')
    m = Motion()
    try:
        m.stand()
        time.sleep(3.0)
        m.go('forward', duration=2.0)
        time.sleep(0.5)
        # m.kick('right')  # enable when robot is held / soft floor
        print('done')
    except KeyboardInterrupt:
        m.estop()
    finally:
        m.stop()
        m.robot.close()


if __name__ == '__main__':
    main()
