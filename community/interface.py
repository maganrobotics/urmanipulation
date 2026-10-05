"""Transport-independent task contract shared by offline and ROS2 examples.

Distances are metres, angles radians. Base goals use map; arm trajectories use
the robot's own joint names. Navigation and manipulation are sequential in v0.1.
"""
from dataclasses import dataclass
from enum import Enum
import math


class Phase(str, Enum):
    IDLE = 'idle'
    NAVIGATING = 'navigating'
    READY = 'ready'
    MANIPULATING = 'manipulating'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    CANCELED = 'canceled'


JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


def joint_names(side):
    if side not in ('left', 'right'):
        raise ValueError('side must be left or right')
    return [side + '_' + name for name in JOINTS]


def validate_trajectory(names, points):
    """Validate the teaching bridge's exact 12-joint, position-only contract."""
    if list(names) != joint_names('left') + joint_names('right'):
        raise ValueError('Expected the documented left-then-right 12-joint order')
    if not points or len(points) > 10000:
        raise ValueError('Trajectory must contain 1..10000 points')
    last = -1.0
    for point in points:
        t, q = point['time_s'], point['positions']
        if not math.isfinite(t) or t <= last or t < 0:
            raise ValueError('Trajectory times must be finite, nonnegative and increasing')
        if len(q) != 12 or not all(math.isfinite(v) for v in q):
            raise ValueError('Expected 12 finite joint positions per point')
        last = t
    return points


@dataclass(frozen=True)
class BaseGoal:
    x: float
    y: float
    yaw: float
    frame_id: str = 'map'

    def __post_init__(self):
        if self.frame_id != 'map' or not all(math.isfinite(v) for v in (self.x, self.y, self.yaw)):
            raise ValueError('Base goal must be finite and in map')


class TaskGate:
    """Only explicit successful results advance the task. Failure never falls through."""
    def __init__(self):
        self.phase = Phase.IDLE
        self.history = [self.phase.value]

    def _set(self, phase):
        self.phase = phase
        self.history.append(phase.value)

    def start_navigation(self):
        if self.phase != Phase.IDLE:
            raise RuntimeError('Navigation requires an idle task')
        self._set(Phase.NAVIGATING)

    def navigation_result(self, success, stopped):
        if self.phase != Phase.NAVIGATING:
            raise RuntimeError('Unexpected navigation result')
        self._set(Phase.READY if success and stopped else Phase.FAILED)

    def start_manipulation(self):
        if self.phase != Phase.READY:
            raise RuntimeError('Manipulation requires successful navigation and a stopped base')
        self._set(Phase.MANIPULATING)

    def manipulation_result(self, success):
        if self.phase != Phase.MANIPULATING:
            raise RuntimeError('Unexpected manipulation result')
        self._set(Phase.SUCCEEDED if success else Phase.FAILED)

    def cancel(self):
        if self.phase in (Phase.SUCCEEDED, Phase.FAILED, Phase.CANCELED):
            raise RuntimeError('Task is already terminal')
        self._set(Phase.CANCELED)

    def fail(self):
        if self.phase not in (Phase.SUCCEEDED, Phase.FAILED, Phase.CANCELED):
            self._set(Phase.FAILED)
