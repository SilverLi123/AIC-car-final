"""终点航向门控：停车优先和几何符号是独立于实车的必要条件。"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                     'src/line_follow_ws/src/line_follower/scripts'))
from precision_parking.finish_heading import FinishHeading, heading_error


class FinishHeadingTests(unittest.TestCase):
    def ready(self):
        gate = FinishHeading()
        gate.set_started(True)
        for n in (1, 2, 3):
            gate.on_count(n, 4)
        gate.on_cleared()
        for i in range(7):
            gate.feedback(i * .1, .5)
        gate.feedback(.64, .12)
        return gate

    def observe(self, gate, angle=20., distance=.65):
        for i in range(3):
            gate.observe(.62 + .02*i, i, (math.sin(math.radians(-angle)),
                         math.cos(math.radians(-angle)), -distance),
                         distance, .02, True, True)

    def test_ground_right_normal_requires_ros_right_turn(self):
        self.assertAlmostEqual(heading_error((.5, .866025403784, -.5)),
                               -math.pi/6, places=8)
        self.assertAlmostEqual(heading_error((-.5, -.866025403784, .5)),
                               -math.pi/6, places=8)

    def test_first_three_requests_keep_original_steering_and_speed(self):
        gate = FinishHeading()
        gate.set_started(True)
        for n in (1, 2):
            gate.on_count(n, 4)
            self.assertEqual(gate.step(1., .62, -.4)[:2], (.62, -.4))

    def test_terminal_cap_does_not_raise_a_lower_upstream_speed(self):
        gate = self.ready()
        self.assertEqual(gate.step(.65, .6, .1)[0], .12)
        self.assertAlmostEqual(gate.step(.65, .6, .1)[1], .02)
        self.assertEqual(gate.step(.65, .07, .1)[:2], (.07, .1))

    def test_third_line_cannot_be_used_before_clearing_and_travelling(self):
        gate = FinishHeading()
        gate.set_started(True)
        for n in (1, 2, 3): gate.on_count(n, 4)
        gate.feedback(.6, .12)
        self.observe(gate)
        self.assertAlmostEqual(gate.step(.67, .6, .1)[1], .02)

    def test_fresh_supported_heading_corrects_left_with_bounded_curvature(self):
        gate = self.ready()
        self.observe(gate)
        v, w, reason = gate.step(.67, .62, .1)
        self.assertEqual(v, .12)
        self.assertGreater(w, 0)
        self.assertLessEqual(w, .18)
        self.assertEqual(reason, 'aligning')

    def test_zero_request_pause_and_fourth_lock_never_restart(self):
        gate = self.ready(); self.observe(gate)
        self.assertEqual(gate.step(.67, 0., .3)[:2], (0., 0.))
        gate.set_started(False)
        self.assertEqual(gate.step(.67, .6, .3)[:2], (0., 0.))
        gate.set_started(True); gate.on_count(4, 4)
        gate.set_started(False); gate.set_started(True)
        self.assertEqual(gate.step(.67, .6, .3)[:2], (0., 0.))

    def test_insufficient_distance_and_stale_geometry_preserve_original_yaw(self):
        gate = self.ready(); self.observe(gate, distance=.2)
        self.assertAlmostEqual(gate.step(.67, .6, -.1)[1], -.02)
        gate = self.ready(); self.observe(gate)
        gate.feedback(.9, .12)
        self.assertAlmostEqual(gate.step(.91, .6, -.1)[1], -.02)

    def test_room_budget_includes_proportional_steering_tail(self):
        gate = self.ready(); self.observe(gate, angle=20., distance=.50)
        self.assertAlmostEqual(gate.step(.67, .6, -.1)[1], -.02)

    def test_stale_odom_and_failed_observation_cancel_heading(self):
        gate = self.ready(); self.observe(gate)
        self.assertAlmostEqual(gate.step(.9, .6, .1)[1], .02)
        gate.observe(.67, 9, None, None, 0., False, False)
        self.assertAlmostEqual(gate.step(.68, .6, .1)[1], .02)

    def test_large_original_turn_is_preserved_to_avoid_cutting_curve(self):
        gate = self.ready(); self.observe(gate)
        self.assertAlmostEqual(gate.step(.67, .6, -.7)[1], -.14)

    def test_lost_road_never_replaces_original_steering(self):
        gate = self.ready(); self.observe(gate)
        self.assertAlmostEqual(gate.step(.67, .6, -.1, tracking=False)[1], -.02)

    def test_nonsequential_count_and_nonfinite_requests_stop(self):
        gate = FinishHeading(); gate.set_started(True); gate.on_count(3, 4)
        self.assertEqual(gate.step(1., .6, .1)[:2], (0., 0.))
        gate = self.ready()
        for v, w in ((float('nan'), 0), (.6, float('inf')), (-.1, 0)):
            self.assertEqual(gate.step(.67, v, w)[:2], (0., 0.))

    def test_duplicates_and_future_observations_cannot_confirm(self):
        gate = self.ready()
        for _ in range(3):
            gate.observe(.65, 1, (-.3, 1., -.6), .6, .02, True, True)
        self.assertAlmostEqual(gate.step(.67, .6, .1)[1], .02)
        self.observe(gate)
        self.assertAlmostEqual(gate.step(.60, .6, .1)[1], .02)

    def test_terminal_slowdown_preserves_requested_curvature(self):
        gate = self.ready()
        v, w, _ = gate.step(.67, .6, .3)
        self.assertEqual(v, .12)
        self.assertAlmostEqual(w, .06)
        self.assertAlmostEqual(w/v, .5)

    def test_alignment_budget_uses_actual_speed_during_deceleration(self):
        gate = self.ready(); gate.feedback(.65, .18); self.observe(gate)
        self.assertNotEqual(gate.step(.67, .6, .1)[2], 'aligning')


if __name__ == '__main__': unittest.main()
