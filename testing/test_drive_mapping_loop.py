# SPDX-License-Identifier: MulanPSL-2.0
"""The mapping loop follows a clear closed square and rejects failed motion."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from drive_mapping_loop import drive_square, main, mapping_square


def grid(size=60):
    data = [-1 if x in (0, size - 1) or y in (0, size - 1) else 0
            for y in range(size) for x in range(size)]
    return SimpleNamespace(info=SimpleNamespace(width=size, height=size, resolution=.05,
        origin=SimpleNamespace(position=SimpleNamespace(x=-1., y=-1.))), data=data)


class MappingLoopTests(unittest.TestCase):
    def test_square_closes_with_four_full_length_legs(self):
        route = mapping_square(grid())
        self.assertEqual(route[0], route[-1])
        self.assertEqual(len(route), 5)
        for a, b in zip(route, route[1:]):
            self.assertAlmostEqual(math.dist(a[:2], b[:2]), 1.5)

    def test_blocked_perimeter_cannot_be_hidden_by_clear_corners(self):
        world = grid()
        for y in range(world.info.height):
            world.data[y * world.info.width + 30] = 100
        with self.assertRaisesRegex(RuntimeError, "no cleared"):
            mapping_square(world)

    def test_diagonal_obstacle_retains_euclidean_clearance(self):
        world = grid()
        world.data[2 * world.info.width + 2] = 100
        route = mapping_square(world)
        obstacle = (-.875, -.875)
        for a, b in zip(route, route[1:]):
            for step in range(31):
                point = tuple(a[i] + (b[i] - a[i]) * step / 30 for i in (0, 1))
                self.assertGreaterEqual(math.dist(point, obstacle), .2)

    def test_failed_navigation_is_cancelled_and_stops_the_loop(self):
        calls = []
        def nav(tool, **args):
            calls.append(tool)
            return {"accepted": True, "run_id": "run"}
        world = SimpleNamespace(count=0, collisions=[])
        with patch("drive_mapping_loop.wait_terminal", return_value={"state": "TIMEOUT_LOCAL"}):
            with self.assertRaisesRegex(RuntimeError, "waypoint 0 failed"):
                drive_square(world, nav, mapping_square(grid()))
        self.assertEqual(calls, ["navigate", "cancel"])

    def test_collision_fails_even_when_navigation_reports_success(self):
        world = SimpleNamespace(count=0, collisions=[])
        def nav(tool, **args):
            world.count += 1
            return {"accepted": True, "run_id": "run"}
        with patch("drive_mapping_loop.wait_terminal", return_value={"state": "SUCCEEDED"}), \
                patch("drive_mapping_loop.read_collision_count", side_effect=lambda _: world.count, create=True):
            with self.assertRaisesRegex(RuntimeError, "collision"):
                drive_square(world, nav, mapping_square(grid()))

    def test_final_collision_is_detected_while_old_subscriber_is_stale(self):
        world = SimpleNamespace(count=0, collisions=[])
        def nav(tool, **args):
            return {"accepted": True, "run_id": "run"}
        with patch("drive_mapping_loop.wait_terminal", return_value={"state": "SUCCEEDED"}), \
                patch("drive_mapping_loop.read_collision_count", side_effect=[0, 0, 0, 0, 1], create=True):
            with self.assertRaisesRegex(RuntimeError, "collision.*4"):
                drive_square(world, nav, mapping_square(grid()))
        self.assertEqual(world.count, 0)

    def test_preparation_requires_clear_space_and_no_collision(self):
        for outcome in ("expanded", "budget", "uncleared", "collision", "timeout", "failure"):
            with self.subTest(outcome=outcome):
                world = SimpleNamespace(costmap=grid(30), count=0, collisions=[], close=Mock())
                preparation, navigation = [], []
                def explore(tool, **args):
                    preparation.append((tool, args))
                    if tool == "explore":
                        return {"accepted": True, "run_id": "prepare"}
                    if tool == "status":
                        if outcome in ("expanded", "budget", "collision", "failure"):
                            world.costmap = grid()
                        if outcome == "collision":
                            world.count = 1
                    return {"state": "TIMEOUT" if outcome == "budget" else "FAILED" if outcome == "failure" else "SUCCEEDED"}
                def nav(tool, **args):
                    if tool == "navigate":
                        navigation.append(args["goal"])
                        return {"accepted": True, "run_id": "navigate"}
                    return {"state": "SUCCEEDED"}
                def terminal(status, run_id, timeout_s):
                    if outcome == "timeout" and run_id == "prepare":
                        return {"state": "TIMEOUT_LOCAL"}
                    return status(run_id=run_id)
                with patch("drive_mapping_loop.World", return_value=world), \
                        patch("drive_mapping_loop.signal.signal"), \
                        patch("drive_mapping_loop.endpoint", side_effect=lambda contract, _: ("provider", contract)), \
                        patch("drive_mapping_loop.activate"), \
                        patch("drive_mapping_loop.Tools", side_effect=lambda url: explore if "/explore/" in url else nav), \
                        patch("drive_mapping_loop.read_collision_count", side_effect=lambda _: world.count), \
                        patch("drive_mapping_loop.wait_terminal", side_effect=terminal):
                    if outcome in ("expanded", "budget"):
                        main()
                        self.assertEqual(len(navigation), 5)
                        self.assertEqual(navigation[0], navigation[-1])
                    else:
                        with self.assertRaises(RuntimeError):
                            main()
                        self.assertEqual(navigation, [])
                self.assertTrue(preparation)
                self.assertEqual(preparation[0][1]["timeout_s"], 120.)
                if outcome == "timeout":
                    self.assertEqual(preparation[-1][0], "cancel")
                world.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
