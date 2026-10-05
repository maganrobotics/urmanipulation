"""Regression checks for failure handling and independently sampled paths."""
import unittest
from community.interface import BaseGoal, TaskGate, Phase, joint_names, validate_trajectory


class ContractTests(unittest.TestCase):
    def test_success_order(self):
        task = TaskGate()
        task.start_navigation()
        task.navigation_result(True, True)
        task.start_manipulation()
        task.manipulation_result(True)
        self.assertEqual(task.phase, Phase.SUCCEEDED)

    def test_failed_or_moving_base_blocks_arm(self):
        for success, stopped in [(False, True), (True, False), (False, False)]:
            task = TaskGate()
            task.start_navigation()
            task.navigation_result(success, stopped)
            with self.assertRaises(RuntimeError):
                task.start_manipulation()

    def test_cancel_blocks_late_result(self):
        task = TaskGate()
        task.start_navigation()
        task.cancel()
        with self.assertRaises(RuntimeError):
            task.navigation_result(True, True)
        with self.assertRaises(RuntimeError):
            task.start_manipulation()

    def test_goal_rejects_invalid_frame_and_nan(self):
        for args in [(0, 0, 0, 'odom'), (float('nan'), 0, 0, 'map')]:
            with self.assertRaises(ValueError):
                BaseGoal(*args)

    def test_trajectory_rejects_reordered_joints_nan_and_time_reversal(self):
        names = joint_names('left') + joint_names('right')
        good = [dict(time_s=0, positions=[0] * 12), dict(time_s=1, positions=[.1] * 12)]
        self.assertEqual(validate_trajectory(names, good), good)
        for n, points in [(names[::-1], good), (names, good[::-1]),
                          (names, [dict(time_s=0, positions=[float('nan')] * 12)]),
                          (names, [dict(time_s=0, positions=[0] * 6)]), (names, [])]:
            with self.assertRaises(ValueError):
                validate_trajectory(n, points)


if __name__ == '__main__':
    unittest.main()
