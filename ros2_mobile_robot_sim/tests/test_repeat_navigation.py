"""Offline checks only; ROS transport and Gazebo still require guest testing."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/repeat_navigation.py'
spec = importlib.util.spec_from_file_location('repeat_navigation', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def pose_message():
    return NS(header=NS(frame_id=''), pose=NS(position=NS(x=0., y=0.),
                                             orientation=NS(z=0., w=1.)))


class FakeRunner(module.Runner):
    def __init__(self, statuses=None, stale_plan=False, arrival_error=0.):
        self.args = NS(rounds=3, timeout=120)
        self.rows, self.statuses, self.sent, self.logged = [], [], [], []
        self.pose = [1., -1., 0.]
        self.responses = iter(statuses if statuses is not None else [4] * 6)
        self.stale_plan, self.arrival_error = stale_plan, arrival_error
        self.planner = NS(wait_for_server=lambda **_: True)
        self.navigator = NS(wait_for_server=lambda **_: True)
        self.PlanGoal = lambda: NS(goal=pose_message())
        self.NavGoal = lambda: NS(pose=pose_message())
        self.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=100000000000)))

    def event(self, kind, **values):
        self.logged.append((kind, values))

    def spin(self, seconds):
        pass

    def healthy_snapshot(self):
        return {'pose': self.pose[:], 'sim_time': 100.}

    def save_rows(self):
        pass

    def request(self, client, goal, label, timeout, monitor=False):
        if client is self.planner:
            start = self.pose[:2]
            # Recreate a far-away stale path, independent of starting test pose.
            if self.stale_plan:
                start = (10., 10.)
            target = (goal.goal.pose.position.x, goal.goal.pose.position.y)
            poses = [NS(pose=NS(position=NS(x=p[0], y=p[1]))) for p in (start, target)]
            return NS(status=4, result=NS(path=NS(header=NS(frame_id='map'), poses=poses)))
        target = (goal.pose.pose.position.x, goal.pose.pose.position.y)
        self.sent.append(target)
        status = next(self.responses)
        if status == 4:
            self.pose = [target[0] + self.arrival_error, target[1], 0.]
        return NS(status=status)


class RepeatNavigationTests(unittest.TestCase):
    def test_recorded_return_error_and_angle_wrap(self):
        distance, angle = module.pose_errors((.899, -.906, math.radians(-8.064)), (1., -1., 0.))
        self.assertAlmostEqual(distance, .13797463535)
        self.assertLess(angle, .15)
        _, wrapped = module.pose_errors((0., 0., math.radians(-179)), (0., 0., math.radians(179)))
        self.assertAlmostEqual(wrapped, math.radians(2))

    def test_actual_old_short_path_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'possible stale automatic start'):
            module.plan_metrics([(.95, -.92), (1., -1.)], (1.4385, 1.6884, 0.), (1., -1., 0.))

    def test_bad_endpoint_empty_and_nonfinite_plans_rejected(self):
        for points in ([], [(float('nan'), 0)], [(0., 0.), (0., .1)]):
            with self.subTest(points=points), self.assertRaises(RuntimeError):
                module.plan_metrics(points, (0., 0., 0.), (1., -1., 0.))

    def test_fresh_publication_with_old_pose_stamp_is_rejected(self):
        snapshot = dict(sim_time=177154., tf_stamp=177154., foot_stamp=1198.285,
                        foot_rx_age=.01, pose_gap_m=2.65, odom_tf_stamp=177154.,
                        local_foot_stamp=177154., local_foot_rx_age=.01, local_pose_gap_m=.01)
        self.assertIn('footprint timestamp', module.observation_problem(snapshot))
        snapshot.update(foot_stamp=177154., pose_gap_m=.02)
        self.assertIsNone(module.observation_problem(snapshot))

    def test_local_stale_pose_is_rejected_with_fresh_global_pose(self):
        snapshot = dict(sim_time=53528., tf_stamp=53528., foot_stamp=53528.,
                        foot_rx_age=.01, pose_gap_m=.01, odom_tf_stamp=53528.,
                        local_foot_stamp=374.16, local_foot_rx_age=.01, local_pose_gap_m=.01)
        self.assertIn('Local footprint timestamp', module.observation_problem(snapshot))

    def test_finish_diagnostics_does_not_replace_failed_half_second_measurement(self):
        runner = FakeRunner()
        runner.args.finish_diagnostics = True
        snapshots = [{'pose': [1., -1., yaw]} for yaw in (0.1, -.185, -.17, -.12, -.1)]
        runner.healthy_snapshot = Mock(side_effect=snapshots)
        baseline = runner.finish_observation(1, (1., -1., 0.))
        self.assertIs(baseline, snapshots[1])
        self.assertGreater(module.pose_errors(baseline['pose'], (1., -1., 0.))[1], .15)
        self.assertEqual([entry[1]['nominal_delay_s'] for entry in runner.logged], [0., .5, 1., 2., 3.])

    def test_three_rounds_send_exactly_six_goals(self):
        runner = FakeRunner()
        runner.run()
        self.assertEqual(runner.sent, [(1.5, 2.), (1., -1.)] * 3)
        self.assertEqual(len(runner.rows), 6)
        self.assertTrue(all(row['passed'] for row in runner.rows))

    def test_aborted_second_goal_prevents_third_goal(self):
        runner = FakeRunner(statuses=[4, 6])
        with self.assertRaisesRegex(RuntimeError, 'Action failed'):
            runner.run()
        self.assertEqual(len(runner.sent), 2)
        self.assertEqual(runner.rows[-1]['status'], 'ABORTED')
        self.assertFalse(runner.rows[-1]['passed'])

    def test_bad_plan_never_sends_motion_goal(self):
        runner = FakeRunner(stale_plan=True)
        with self.assertRaisesRegex(RuntimeError, 'possible stale automatic start'):
            runner.run()
        self.assertEqual(runner.sent, [])
        self.assertEqual(runner.rows[0]['status'], 'NOT_SENT')

    def test_action_success_outside_tolerance_stops_sequence(self):
        runner = FakeRunner(arrival_error=.2)
        with self.assertRaisesRegex(RuntimeError, 'post-arrival TF exceeded'):
            runner.run()
        self.assertEqual(len(runner.sent), 1)
        self.assertEqual(runner.rows[0]['status'], 'SUCCEEDED')
        self.assertFalse(runner.rows[0]['passed'])

    def test_already_active_navigation_not_preempted(self):
        runner = FakeRunner()
        runner.statuses = [2]
        with self.assertRaisesRegex(RuntimeError, 'Another navigation goal'):
            runner.run()
        self.assertEqual(runner.sent, [])

    def test_cleanup_cancels_owned_goal_and_waits_for_terminal_result(self):
        runner = FakeRunner()
        runner.pending = None
        runner.action_label = 'navigation'
        runner.result_future = NS(done=lambda: False)
        cancel = Mock(return_value=object())
        runner.handle = NS(accepted=True, cancel_goal_async=cancel)
        runner.wait = Mock(side_effect=[NS(goals_canceling=[1]), NS(status=5)])
        runner.cancel_own_goal()
        cancel.assert_called_once()
        self.assertEqual(runner.wait.call_count, 2)
        self.assertTrue(all(c.kwargs['cleanup'] for c in runner.wait.call_args_list))
        self.assertEqual(runner.logged[-1][0], 'stop_confirmed')
        self.assertEqual(runner.logged[-1][1]['status'], 'CANCELED')
        self.assertIsNone(runner.handle)

    def test_unresolved_goal_acceptance_is_not_reported_as_stopped(self):
        runner = FakeRunner()
        runner.pending = object()
        runner.wait = Mock(side_effect=RuntimeError('acceptance unavailable'))
        runner.cancel_own_goal()
        self.assertEqual(runner.logged[-1][0], 'cancel_unconfirmed')


if __name__ == '__main__':
    unittest.main()
