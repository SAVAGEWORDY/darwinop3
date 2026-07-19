"""L3 motion prep — go() + module wiring + angle helpers."""

from __future__ import annotations

import time
from typing import Optional

from op3_football.l2.robot import Robot
from op3_football.l3.coefs import WalkCoefs, presets
from op3_football.l3.joint_angles import JointAngles
from op3_football.l3.sense import Sense


class Motion:
    """High-ish API for L4: walk modes, angles, sensing. No strategy here."""

    def __init__(self, robot: Optional[Robot] = None) -> None:
        self.robot = robot or Robot().start()
        self.joint = JointAngles(self.robot.joint)
        self.sense = Sense(self.robot)
        self._presets = presets()

    def go(self, mode: str = 'forward', duration: Optional[float] = None) -> None:
        """Start prepared walking. Coefficients chosen here, not in L4.

        mode: key from presets() — forward, turn_left, pivot_right, ...
        duration: if set, walk then stop; else leave walking on.
        """
        if mode not in self._presets:
            raise KeyError(f'Unknown go mode {mode!r}. Available: {list(self._presets)}')

        # Connect modules at L3
        self.robot.set_module('walking_module')
        time.sleep(0.05)
        coefs = self._presets[mode]
        self.robot.walk.start(coefs)

        if duration is not None:
            time.sleep(duration)
            self.stop()

    def go_coefs(self, coefs: WalkCoefs, duration: Optional[float] = None) -> None:
        self.robot.set_module('walking_module')
        time.sleep(0.05)
        self.robot.walk.start(coefs)
        if duration is not None:
            time.sleep(duration)
            self.stop()

    def stop(self) -> None:
        self.robot.walk.stop()

    def stand(self) -> None:
        self.robot.stand()

    def sit(self) -> None:
        self.robot.sit()

    def kick(self, side: str = 'right') -> None:
        if side == 'left':
            self.robot.kick.left()
        else:
            self.robot.kick.right()
        # return control path to walking-ready posture
        self.robot.set_module('walking_module')

    def estop(self) -> None:
        self.robot.estop()

    def get_preset(self, name: str) -> WalkCoefs:
        return self._presets[name]
