#!/usr/bin/env python3
"""Run three B/A round trips in the existing room simulation; log every attempt.

Unlike check_return_plan.py, this script DOES move the simulated robot.
Use only after the previous navigation goal has finished, with no teleop running.
"""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import time
import xml.etree.ElementTree as ET


TARGETS = [('B', (1.5, 2.0, 0.0)), ('A', (1.0, -1.0, 0.0))]
STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def pose_errors(pose, target):
    dx, dy = pose[0] - target[0], pose[1] - target[1]
    angle = pose[2] - target[2]
    return math.hypot(dx, dy), abs(math.atan2(math.sin(angle), math.cos(angle)))


def plan_metrics(points, robot, target):
    if not points or any(not math.isfinite(v) for p in points for v in p):
        raise RuntimeError('Empty or non-finite global plan')
    first_gap = math.dist(points[0], robot[:2])
    end_gap = math.dist(points[-1], target[:2])
    metrics = dict(points=len(points), first_gap_m=first_gap, end_gap_m=end_gap,
                   length_m=sum(math.dist(a, b) for a, b in zip(points, points[1:])))
    if first_gap > 0.5:
        raise RuntimeError(f'Plan starts {first_gap:.3f} m from robot (limit 0.5 m): '
                           'possible stale automatic start')
    if end_gap > 0.2:
        raise RuntimeError(f'Plan ends {end_gap:.3f} m from target (limit 0.2 m)')
    return metrics


def observation_problem(snapshot):
    required = ('sim_time', 'tf_stamp', 'foot_stamp', 'foot_rx_age', 'pose_gap_m',
                'odom_tf_stamp', 'local_foot_stamp', 'local_foot_rx_age', 'local_pose_gap_m')
    if any(not math.isfinite(snapshot[k]) for k in required):
        return 'Non-finite pose observation'
    if snapshot['sim_time'] <= 0:
        return 'No simulation clock'
    if abs(snapshot['sim_time'] - snapshot['tf_stamp']) > 2.0:
        return 'Current map TF timestamp differs from simulation clock by over 2 s'
    if snapshot['foot_rx_age'] > 3.0:
        return 'No global footprint received for over 3 wall seconds'
    if abs(snapshot['sim_time'] - snapshot['foot_stamp']) > 2.0:
        return 'Global footprint timestamp differs from simulation clock by over 2 s'
    if snapshot['pose_gap_m'] > 0.5:
        return 'Global footprint center differs from current TF by over 0.5 m'
    if abs(snapshot['sim_time'] - snapshot['odom_tf_stamp']) > 2.0:
        return 'Current odom TF timestamp differs from simulation clock by over 2 s'
    if snapshot['local_foot_rx_age'] > 3.0:
        return 'No local footprint received for over 3 wall seconds'
    if abs(snapshot['sim_time'] - snapshot['local_foot_stamp']) > 2.0:
        return 'Local footprint timestamp differs from simulation clock by over 2 s'
    if snapshot['local_pose_gap_m'] > 0.5:
        return 'Local footprint center differs from current odom TF by over 0.5 m'
    return None


class Runner:
    def __init__(self, args, output):
        # Lazy imports allow offline tests of the calculations without ROS installed.
        import rclpy
        from action_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import PolygonStamped, Twist
        from nav_msgs.msg import Odometry
        from nav2_msgs.action import ComputePathToPose, NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data, qos_profile_action_status_default
        from rclpy.signals import SignalHandlerOptions
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener

        self.args, self.output = args, output
        self.ros, self.Time = rclpy, Time
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node('repeat_navigation_test', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.planner = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
        self.navigator = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.PlanGoal, self.NavGoal = ComputePathToPose.Goal, NavigateToPose.Goal
        self.footprint = None
        self.foot_received = 0.0
        self.local_footprint = None
        self.local_foot_received = 0.0
        self.motion = {}
        self.statuses = []
        self.node.create_subscription(PolygonStamped, '/global_costmap/published_footprint',
                                      self.on_footprint, qos_profile_sensor_data)
        self.node.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                      self.on_local_footprint, qos_profile_sensor_data)
        if args.finish_diagnostics:
            for topic, msg_type in (('/odom', Odometry), ('/cmd_vel', Twist), ('/cmd_vel_nav', Twist)):
                self.node.create_subscription(msg_type, topic,
                    lambda msg, topic=topic: self.on_motion(topic, msg), qos_profile_sensor_data)
        self.node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                      self.on_status, qos_profile_action_status_default)
        self.stop = False
        self.handle = self.pending = self.result_future = None
        self.action_label = ''
        self.feedback = None
        self.events = (output / 'events.jsonl').open('w', encoding='utf-8', buffering=1)
        self.rows = []
        self.started = time.monotonic()
        self.last_sample = self.last_progress = self.started
        self.bad_since = None

    def on_footprint(self, msg):
        self.footprint, self.foot_received = msg, time.monotonic()

    def on_local_footprint(self, msg):
        self.local_footprint, self.local_foot_received = msg, time.monotonic()

    def on_motion(self, topic, msg):
        twist = msg.twist.twist if topic == '/odom' else msg
        self.motion[topic] = dict(vx=twist.linear.x, vy=twist.linear.y, wz=twist.angular.z,
            stamp=(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) if topic == '/odom' else None,
            received=time.monotonic())

    @staticmethod
    def tf_pose(tf):
        p, q = tf.transform.translation, tf.transform.rotation
        return [p.x, p.y, math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))]

    def on_status(self, msg):
        self.statuses = [entry.status for entry in msg.status_list]

    def on_feedback(self, msg):
        self.feedback = msg.feedback

    def event(self, kind, **values):
        self.events.write(json.dumps(dict(event=kind, wall_elapsed_s=time.monotonic() - self.started,
                                         **values), ensure_ascii=False, allow_nan=False) + '\n')

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.stop:
                raise RuntimeError('Interrupted by user')
            self.ros.spin_once(self.node, timeout_sec=0.05)

    def snapshot(self):
        tf = self.buffer.lookup_transform('map', 'base_footprint', self.Time())
        p, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        foot = self.footprint
        if foot is None or not foot.polygon.points:
            raise RuntimeError('No global footprint yet')
        if foot.header.frame_id != 'map':
            raise RuntimeError('Expected global footprint in map, got ' + foot.header.frame_id)
        center = [sum(getattr(v, axis) for v in foot.polygon.points) / len(foot.polygon.points)
                  for axis in ('x', 'y')]
        stamp = lambda s: s.sec + s.nanosec * 1e-9
        odom_tf = self.buffer.lookup_transform('odom', 'base_footprint', self.Time())
        odom_pose = self.tf_pose(odom_tf)
        local = self.local_footprint
        if local is None or not local.polygon.points:
            raise RuntimeError('No local footprint yet')
        if local.header.frame_id != 'odom':
            raise RuntimeError('Expected local footprint in odom, got ' + local.header.frame_id)
        local_center = [sum(getattr(v, axis) for v in local.polygon.points) / len(local.polygon.points)
                        for axis in ('x', 'y')]
        snapshot = dict(pose=[p.x, p.y, yaw], sim_time=self.node.get_clock().now().nanoseconds * 1e-9,
                    tf_stamp=stamp(tf.header.stamp), foot_stamp=stamp(foot.header.stamp),
                    footprint_center=center, foot_rx_age=time.monotonic() - self.foot_received,
                    pose_gap_m=math.hypot(center[0] - p.x, center[1] - p.y),
                    odom_pose=odom_pose, odom_tf_stamp=stamp(odom_tf.header.stamp),
                    local_foot_stamp=stamp(local.header.stamp), local_footprint_center=local_center,
                    local_foot_rx_age=time.monotonic() - self.local_foot_received,
                    local_pose_gap_m=math.dist(local_center, odom_pose[:2]))
        if self.args.finish_diagnostics:
            correction = self.buffer.lookup_transform('map', 'odom', self.Time())
            snapshot['map_odom_pose'] = self.tf_pose(correction)
            snapshot['map_odom_stamp'] = stamp(correction.header.stamp)
            snapshot['motion'] = {topic: dict(vx=v['vx'], vy=v['vy'], wz=v['wz'], stamp=v['stamp'],
                rx_age=time.monotonic() - v['received']) for topic, v in self.motion.items()}
        return snapshot

    def healthy_snapshot(self):
        snapshot = self.snapshot()
        problem = observation_problem(snapshot)
        if problem:
            self.event('bad_observation', reason=problem, snapshot=snapshot)
            raise RuntimeError(problem)
        return snapshot

    def monitor(self):
        # Allow a short transient; persistent TF/footprint faults stop this goal.
        try:
            snapshot = self.snapshot()
            problem = observation_problem(snapshot)
        except Exception as error:
            snapshot, problem = None, str(error)
        now = time.monotonic()
        if problem:
            if self.bad_since is None:
                self.bad_since = now
                self.event('observation_warning', reason=problem, snapshot=snapshot)
            if now - self.bad_since >= 2.0:
                raise RuntimeError(problem)
        else:
            self.bad_since = None
        if now - self.last_sample >= (0.1 if self.args.finish_diagnostics else 1.0):
            self.event('pose_sample', snapshot=snapshot, problem=problem)
            self.last_sample = now
        if now - self.last_progress >= 5:
            extra = '' if self.feedback is None else (
                f' remaining={self.feedback.distance_remaining:.2f} m '
                f'recoveries={self.feedback.number_of_recoveries}')
            print('  Navigating...' + extra, flush=True)
            self.last_progress = now

    def finish_observation(self, trial, target):
        # Retain the original +0.5 s metric. Later samples are diagnostic only.
        if not getattr(self.args, 'finish_diagnostics', False):
            self.spin(0.5)
            return self.healthy_snapshot()
        previous = 0.0
        baseline = None
        for offset in (0.0, 0.5, 1.0, 2.0, 3.0):
            if offset > previous:
                self.spin(offset - previous)
            snapshot = self.healthy_snapshot()
            error, angle = pose_errors(snapshot['pose'], target)
            self.event('finish_sample', trial=trial, nominal_delay_s=offset, snapshot=snapshot,
                       position_error_m=error, yaw_error_deg=math.degrees(angle))
            odom = snapshot.get('motion', {}).get('/odom')
            speed = 'unavailable' if odom is None else f'{odom["wz"]:.4f} rad/s (age {odom["rx_age"]:.2f}s)'
            print(f'  FINISH +{offset:.1f}s: xy={error*100:.2f} cm, '
                  f'yaw={math.degrees(angle):.3f} deg, odom wz={speed}', flush=True)
            if 'map_odom_pose' in snapshot:
                cmd = snapshot.get('motion', {}).get('/cmd_vel')
                cmd_text = 'unavailable' if cmd is None else f'{cmd["wz"]:.4f} (age {cmd["rx_age"]:.2f}s)'
                print(f'    yaw deg: map/base={math.degrees(snapshot["pose"][2]):.3f}, '
                      f'odom/base={math.degrees(snapshot["odom_pose"][2]):.3f}, '
                      f'map/odom={math.degrees(snapshot["map_odom_pose"][2]):.3f}; '
                      f'cmd wz={cmd_text}', flush=True)
            if offset == 0.5:
                baseline = snapshot
            previous = offset
        return baseline

    def wait(self, future, timeout, monitor=False, cleanup=False):
        deadline = time.monotonic() + timeout
        while not future.done():
            if self.stop and not cleanup:
                raise RuntimeError('Interrupted by user')
            if time.monotonic() >= deadline:
                raise RuntimeError('Timeout during ' + self.action_label)
            if not self.ros.ok():
                raise RuntimeError('ROS context stopped')
            self.ros.spin_once(self.node, timeout_sec=0.1)
            if monitor and not future.done():
                self.monitor()
        return future.result()

    def request(self, client, goal, label, timeout, monitor=False):
        self.action_label = label
        self.pending = client.send_goal_async(goal, feedback_callback=self.on_feedback if monitor else None)
        self.handle = self.wait(self.pending, 10)
        self.pending = None
        if not self.handle.accepted:
            self.handle = None
            raise RuntimeError(label + ' rejected')
        self.event('goal_accepted', action=label, goal_id=bytes(self.handle.goal_id.uuid).hex())
        self.result_future = self.handle.get_result_async()
        result = self.wait(self.result_future, timeout, monitor=monitor)
        self.handle = self.result_future = None
        return result

    def cancel_own_goal(self):
        try:
            if self.pending is not None:
                self.handle = self.wait(self.pending, 5, cleanup=True)
                self.pending = None
            if self.handle is None or not self.handle.accepted:
                return
            if self.result_future is None:
                self.result_future = self.handle.get_result_async()
            if not self.result_future.done():
                print('Canceling this test goal; waiting for final status...', flush=True)
                self.wait(self.handle.cancel_goal_async(), 5, cleanup=True)
            result = self.wait(self.result_future, 10, cleanup=True)
            if result.status not in STATUS:
                raise RuntimeError('Action has not reached a terminal status')
            self.event('stop_confirmed', action=self.action_label, status=STATUS[result.status])
            print('Action terminal status: ' + STATUS[result.status], flush=True)
            self.handle = None
        except Exception as error:
            self.event('cancel_unconfirmed', reason=str(error))
            print('STOP NOT CONFIRMED. Press Ctrl+C in the navigation.launch.py terminal.', flush=True)

    @staticmethod
    def fill_pose(pose, target):
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y = target[:2]
        pose.pose.orientation.z = math.sin(target[2] / 2)
        pose.pose.orientation.w = math.cos(target[2] / 2)

    def save_rows(self):
        keys = ['trial', 'round', 'target', 'goal_x', 'goal_y', 'status', 'passed',
                'wall_s', 'sim_s', 'x', 'y', 'yaw_deg', 'position_error_m', 'yaw_error_deg', 'reason']
        with (self.output / 'results.csv').open('w', newline='', encoding='utf-8-sig') as stream:
            writer = csv.DictWriter(stream, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.rows)

    def run(self):
        self.event('test_start', rounds=self.args.rounds, targets=TARGETS,
                   xy_tolerance_m=0.15, yaw_tolerance_rad=0.15, goal_timeout_wall_s=self.args.timeout,
                   finish_diagnostics=getattr(self.args, 'finish_diagnostics', False),
                   behavior_tree=str(getattr(self.args, 'behavior_tree', None) or ''),
                   script_version=2)
        print('Checking live TF, global footprint and action servers...', flush=True)
        self.spin(3)
        if any(s in (1, 2, 3) for s in self.statuses):
            raise RuntimeError('Another navigation goal is active; finish/cancel it before this test')
        start = self.healthy_snapshot()
        if pose_errors(start['pose'], TARGETS[1][1])[0] > 0.3:
            raise RuntimeError('Start must be within 0.3 m of A=(1,-1); robot has not been moved')
        for client in (self.planner, self.navigator):
            if not client.wait_for_server(timeout_sec=8):
                raise RuntimeError('Planning/navigation action server unavailable')
        for round_number in range(1, self.args.rounds + 1):
            for name, target in TARGETS:
                row = dict(trial=len(self.rows) + 1, round=round_number, target=name,
                           goal_x=target[0], goal_y=target[1], status='NOT_SENT', passed=False, reason='')
                print(f'[{row["trial"]}/{self.args.rounds * 2}] round {round_number}: '
                      f'target {name} {target[:2]}', flush=True)
                wall_start = sim_start = None
                try:
                    self.spin(0.5)
                    self.healthy_snapshot()
                    planning_goal = self.PlanGoal()
                    self.fill_pose(planning_goal.goal, target)
                    planning_goal.planner_id, planning_goal.use_start = 'GridBased', False
                    planned = self.request(self.planner, planning_goal, 'planning', 20)
                    if planned.status != 4 or planned.result.path.header.frame_id != 'map':
                        raise RuntimeError('Automatic planning failed or returned a non-map frame')
                    path = planned.result.path
                    points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
                    current = self.healthy_snapshot()
                    self.event('raw_plan', trial=row['trial'], points=points, snapshot=current)
                    metrics = plan_metrics(points, current['pose'], target)
                    self.event('plan_check', trial=row['trial'], **metrics)
                    print(f'  Plan: {metrics["points"]} points, {metrics["length_m"]:.2f} m, '
                          f'start gap {metrics["first_gap_m"]:.3f} m', flush=True)
                    nav_goal = self.NavGoal()
                    self.fill_pose(nav_goal.pose, target)
                    if getattr(self.args, 'behavior_tree', None):
                        nav_goal.behavior_tree = str(self.args.behavior_tree)
                    self.feedback, self.bad_since = None, None
                    wall_start = time.monotonic()
                    sim_start = self.node.get_clock().now().nanoseconds * 1e-9
                    row['status'] = 'REQUESTED'
                    result = self.request(self.navigator, nav_goal, 'navigation', self.args.timeout, True)
                    row['status'] = STATUS.get(result.status, str(result.status))
                    self.event('navigation_result', trial=row['trial'], status=row['status'])
                    row['wall_s'] = time.monotonic() - wall_start
                    row['sim_s'] = self.node.get_clock().now().nanoseconds * 1e-9 - sim_start
                    after = self.finish_observation(row['trial'], target)
                    error, angle = pose_errors(after['pose'], target)
                    row.update(x=after['pose'][0], y=after['pose'][1], yaw_deg=math.degrees(after['pose'][2]),
                               position_error_m=error, yaw_error_deg=math.degrees(angle))
                    row['passed'] = result.status == 4 and error <= 0.15 and angle <= 0.15
                    self.event('arrival', trial=row['trial'], snapshot=after)
                    if not row['passed']:
                        if result.status != 4:
                            raise RuntimeError('Action failed: ' + row['status'])
                        raise RuntimeError(f'post-arrival TF exceeded tolerance: '
                            f'xy={error:.5f} m (limit 0.15), yaw={math.degrees(angle):.3f} deg '
                            '(limit 8.594); measured +0.5 s after action result')
                    print(f'  PASS: {row["status"]}, xy={error * 100:.2f} cm, '
                          f'yaw={math.degrees(angle):.3f} deg, wall={row["wall_s"]:.1f} s', flush=True)
                except Exception as error:
                    row['passed'] = False
                    row['reason'] = str(error)
                    raise
                finally:
                    if wall_start is not None and 'wall_s' not in row:
                        row['wall_s'] = time.monotonic() - wall_start
                        row['sim_s'] = self.node.get_clock().now().nanoseconds * 1e-9 - sim_start
                    self.rows.append(row)
                    self.event('trial_result', **row)
                    self.save_rows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rounds', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=120, help='Wall seconds per navigation goal')
    parser.add_argument('--finish-diagnostics', action='store_true',
                        help='Log TF and velocities at 10 Hz and observe 3 s after each result; retain +0.5 s pass criterion')
    parser.add_argument('--output', type=Path, default=Path.home() / 'ros2_ws/test_results')
    parser.add_argument('--behavior-tree', type=Path,
                        help='Optional BT XML for each navigation goal; omit to use Nav2 default')
    args = parser.parse_args()
    if args.behavior_tree is not None:
        args.behavior_tree = args.behavior_tree.expanduser().resolve()
        try:
            ET.parse(args.behavior_tree)
        except (OSError, ET.ParseError) as error:
            parser.error(f'Cannot read behavior-tree XML: {error}')
    if not 1 <= args.rounds <= 10 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('rounds must be 1..10 and timeout must be positive and finite')
    output = args.output.expanduser() / datetime.now().strftime('roundtrip_%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    print('Repeat navigation v2. Robot WILL MOVE: A -> B -> A, repeated. '
          'No concurrent teleop/navigation commands.', flush=True)
    print('Results: ' + str(output.resolve()), flush=True)
    if args.behavior_tree is not None:
        print('Per-goal behavior tree: ' + str(args.behavior_tree), flush=True)
    runner = Runner(args, output)
    old_signals = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        old_signals[signum] = signal.signal(signum, lambda *_: setattr(runner, 'stop', True))
    code = 0
    try:
        runner.run()
    except Exception as error:
        code = 1
        runner.event('test_stopped', reason=str(error))
        print('TEST STOPPED: ' + str(error), flush=True)
    finally:
        runner.cancel_own_goal()
        passed = sum(row['passed'] for row in runner.rows)
        succeeded = sum(row['status'] == 'SUCCEEDED' for row in runner.rows)
        summary = dict(planned=args.rounds * 2, attempted=len(runner.rows),
                       action_succeeded=succeeded, passed=passed,
                       all_passed=passed == args.rounds * 2, exit_code=code)
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        runner.events.close()
        runner.planner.destroy()
        runner.navigator.destroy()
        runner.node.destroy_node()
        runner.ros.try_shutdown()
        for signum, handler in old_signals.items():
            signal.signal(signum, handler)
        print(f'SUMMARY: planned={summary["planned"]}, attempted={summary["attempted"]}, '
              f'action_succeeded={succeeded}, passed={passed}', flush=True)
        print('Results: ' + str(output.resolve()), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
