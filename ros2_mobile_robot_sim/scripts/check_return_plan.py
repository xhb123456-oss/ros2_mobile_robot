#!/usr/bin/env python3
"""Inspect a stationary return plan; never send velocity or navigation goals.

Requests only ComputePathToPose, which may update the displayed global plan.
Run after the previous NavigateToPose action has finished.
"""
import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


def wait(node, future, seconds=10.0, label='ROS response'):
    rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
    if not future.done():
        raise RuntimeError('Timed out waiting for ' + label)
    return future.result()


def parameters(node, target, names):
    print('Reading parameters: ' + target, flush=True)
    client = node.create_client(GetParameters, target + '/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Parameter service unavailable: ' + target)
        reply = wait(node, client.call_async(GetParameters.Request(names=names)),
                     label=target + '/get_parameters')
        fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value'}
        result = {}
        for name, value in zip(names, reply.values):
            result[name] = getattr(value, fields[value.type]) if value.type in fields else None
            print(f'{target} {name} = {result[name]}')
        return result
    finally:
        node.destroy_client(client)


def transform(node, buffer, frame):
    print(f'Waiting for TF {frame} -> base_footprint', flush=True)
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            return buffer.lookup_transform(frame, 'base_footprint', Time())
        except TransformException:
            pass
    raise RuntimeError(f'No current {frame} -> base_footprint transform')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--explicit-start', action='store_true',
                        help='Supply the current map TF as the planning start for comparison')
    parser.add_argument('--skip-parameters', action='store_true',
                        help='Skip parameter services and the DWB crop estimate')
    args = parser.parse_args()
    rclpy.init()
    node = Node('check_return_plan', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    try:
        print('DIAGNOSTIC ONLY: planning to (1.0, -1.0); no motion command.')
        config, local = {}, {}
        if args.skip_parameters:
            print('Skipping parameter services; checking TF and planner only.', flush=True)
        else:
            config = parameters(node, '/controller_server', [
                'FollowPath.prune_plan', 'FollowPath.prune_distance',
                'FollowPath.forward_prune_distance', 'FollowPath.shorten_transformed_plan'])
            local = parameters(node, '/local_costmap/local_costmap', [
                'global_frame', 'robot_base_frame', 'width', 'height', 'resolution', 'rolling_window'])
            parameters(node, '/global_costmap/global_costmap', [
                'global_frame', 'robot_base_frame', 'use_sim_time'])
            parameters(node, '/planner_server', ['GridBased.plugin', 'GridBased.use_astar', 'use_sim_time'])
        for frame in ('map', 'odom'):
            tf = transform(node, buffer, frame)
            p = tf.transform.translation
            print(f'Current TF {frame}: x={p.x:.4f}, y={p.y:.4f}, '
                  f't={tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9:.3f}')
        print('Waiting for /compute_path_to_pose action server', flush=True)
        if not planner.wait_for_server(timeout_sec=8.0):
            raise RuntimeError('ComputePathToPose action unavailable')
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = 1.0
        goal.goal.pose.position.y = -1.0
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = 'GridBased'
        goal.use_start = args.explicit_start
        if args.explicit_start:
            start_tf = transform(node, buffer, 'map')
            goal.start.header.frame_id = 'map'
            goal.start.pose.position.x = start_tf.transform.translation.x
            goal.start.pose.position.y = start_tf.transform.translation.y
            goal.start.pose.position.z = start_tf.transform.translation.z
            goal.start.pose.orientation = start_tf.transform.rotation
            print(f'EXPLICIT start: x={goal.start.pose.position.x:.4f}, '
                  f'y={goal.start.pose.position.y:.4f}; use_start=True')
        else:
            print('AUTOMATIC start: use_start=False')
        print('Sending planning request (no motion)', flush=True)
        handle = wait(node, planner.send_goal_async(goal), label='planning request acceptance')
        if not handle.accepted:
            raise RuntimeError('Planning request rejected')
        try:
            print('Planning request accepted; waiting for result', flush=True)
            response = wait(node, handle.get_result_async(), 20.0, label='planning result')
        except RuntimeError:
            try:
                wait(node, handle.cancel_goal_async(), 5.0, label='planning cancellation')
            except Exception as cancel_error:
                print(f'Planning cancellation not confirmed: {cancel_error}', flush=True)
            raise
        print(f'Planning status: {response.status} (SUCCEEDED={GoalStatus.STATUS_SUCCEEDED})')
        path = response.result.path
        print(f'Path frame: {path.header.frame_id}; points: {len(path.poses)}')
        if response.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
            return
        points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        tf = transform(node, buffer, path.header.frame_id)
        robot = tf.transform.translation
        distances = [math.hypot(x - robot.x, y - robot.y) for x, y in points]
        length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        nearest = min(range(len(points)), key=distances.__getitem__)
        print(f'First point: {points[0]}; last point: {points[-1]}')
        print(f'Path length: {length:.4f} m; robot to first: {distances[0]:.4f} m')
        print(f'Nearest point: index={nearest}, distance={distances[nearest]:.4f} m')

        # Reproduce the 1.1.20 DWB index selection for this NEW path and latest TF.
        # This is not a replay of the earlier failure or a collision check.
        if any(value is None for value in config.values()) or any(
                local.get(name) is None for name in ('width', 'height', 'resolution')):
            print('Missing parameters: skipping DWB crop estimate.')
            return
        resolution = local['resolution']
        radius = max(int(local['width'] / resolution), int(local['height'] / resolution)) * resolution / 2
        prune = config['FollowPath.prune_distance']
        start_limit = min(radius, prune) if config['FollowPath.prune_plan'] else radius
        end_limit = (min(radius, config['FollowPath.forward_prune_distance'])
                     if config['FollowPath.shorten_transformed_plan'] else radius)
        prune_index, cumulative = len(points), 0.0
        for i in range(len(points) - 1):
            cumulative += math.dist(points[i], points[i + 1])
            if cumulative > prune:
                prune_index = i + 1
                break
        begin = next((i for i in range(prune_index) if distances[i] < start_limit), prune_index)
        end = next((i for i in range(begin, len(points)) if distances[i] > end_limit), len(points))
        print(f'DWB crop estimate: begin={begin}, end={end}, kept={end - begin} points')
        print('New stationary plan only; does not reconstruct the failed action.')
    except Exception as error:
        print(f'DIAGNOSTIC ERROR: {type(error).__name__}: {error}')
    finally:
        planner.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
