"""L2 locomotion skills. Coefficients always come from L3."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from op3_football.l3.coefs import WalkCoefs

from op3_football.l1.bridge_client import BridgeClient


class WalkAPI:
    def __init__(self, bridge: BridgeClient) -> None:
        self._bridge = bridge
        self._running = False

    def start(self, coefs: 'WalkCoefs') -> None:
        self._bridge.set_module('walking_module')
        time.sleep(0.05)
        self._bridge.set_walking_params(coefs.to_msg())
        self._bridge.walking_command('start')
        self._running = True

    def update(self, coefs: 'WalkCoefs') -> None:
        """Update amplitudes while walking."""
        self._bridge.set_walking_params(coefs.to_msg())

    def stop(self) -> None:
        self._bridge.walking_command('stop')
        self._running = False

    @property
    def running(self) -> bool:
        return self._running


class PostureAPI:
    def __init__(self, bridge: BridgeClient, joint) -> None:
        self._bridge = bridge
        self._joint = joint

    def stand(self) -> None:
        """Unified stand pose via action page 50."""
        self._bridge.walking_command('stop')
        self._bridge.play_action_page(50, wait_s=1.2)

    def sit(self) -> None:
        """Simple sit trajectory in ticks (placeholder — tune on hardware)."""
        self._bridge.set_module('direct_control_module')
        # Rough crouch: bend hips/knees/ankles around center 2048
        goals = {
            11: 1800, 12: 2296,  # hip pitch
            13: 1600, 14: 2496,  # knee
            15: 1900, 16: 2196,  # ankle pitch
            3: 1800, 4: 2296,    # shoulder roll slightly in
            5: 2400, 6: 1696,    # elbows
        }
        self._joint.write_many(goals)


class TurnAPI:
    def __init__(self, walk: WalkAPI) -> None:
        self._walk = walk

    def turn(self, coefs: 'WalkCoefs') -> None:
        """Walk with yaw amplitude from L3 coefs."""
        self._walk.start(coefs)

    def pivot(self, coefs: 'WalkCoefs') -> None:
        """In-place rotation; L3 should zero x/y amplitudes."""
        self._walk.start(coefs)
