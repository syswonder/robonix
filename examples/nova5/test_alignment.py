# SPDX-License-Identifier: MulanPSL-2.0
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from run import alignment_targets


class AlignmentTest(unittest.TestCase):
    def check_alignment(self, start_angles, target_angles):
        start, target = Rotation.from_euler(
            "xyz", [start_angles, target_angles], degrees=True).as_matrix()
        tcp_position = np.array([600., -20., 90.])
        start_tcp = np.eye(4)
        start_tcp[:3, :3], start_tcp[:3, 3] = start, tcp_position
        targets = alignment_targets(start_tcp, target)
        offset = np.array([0., 0., 165.])
        previous = start
        for target_tcp in targets:
            np.testing.assert_array_equal(target_tcp[:3, 3], tcp_position)
            rotation = target_tcp[:3, :3]
            angle = np.degrees(Rotation.from_matrix(previous.T @ rotation).magnitude())
            previous_flange = tcp_position - previous @ offset
            flange = tcp_position - rotation @ offset
            self.assertLessEqual(angle, 70)
            self.assertLessEqual(np.linalg.norm(flange - previous_flange), 100)
            np.testing.assert_allclose(flange + rotation @ offset, tcp_position)
            previous = rotation
        np.testing.assert_allclose(targets[-1][:3, :3], target, atol=1e-12)
        return targets

    def test_return_to_home_orientation(self):
        rotations = self.check_alignment(
            [-179.999755859375, 0.00026062954567102704, 111.09854125976564],
            [170.8342, 25.7119, -171.0346])
        self.assertGreater(len(rotations), 1)

    def test_tilt_keeps_flange_steps_within_distance_bound(self):
        self.check_alignment([180, 0, 0], [0, 0, 0])

    def test_shortest_rotation_and_unchanged_orientation(self):
        for start, target in (([180, 0, 179], [180, 0, -179]),
                              ([180, 0, 90], [180, 0, 90])):
            with self.subTest(start=start, target=target):
                self.assertEqual(len(self.check_alignment(start, target)), 1)


if __name__ == "__main__":
    unittest.main()
