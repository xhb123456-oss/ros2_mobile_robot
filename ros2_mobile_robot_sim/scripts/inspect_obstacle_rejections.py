"""Compare rejected DWB trajectories with bracketing recorded local costmaps.

Read-only, single-file SQLite bag analysis; no ROS nodes or publishers.
Snapshots are not the controller's exact internal costmap at evaluation time.
"""
import argparse
import bisect
from collections import Counter
import math
from pathlib import Path
import sqlite3


def stamp(value):
    return value.sec + value.nanosec * 1e-9


def cell_cost(grid, pose):
    meta = grid.metadata
    x = math.floor((pose.x - meta.origin.position.x) / meta.resolution)
    y = math.floor((pose.y - meta.origin.position.y) / meta.resolution)
    if not (0 <= x < meta.size_x and 0 <= y < meta.size_y):
        return 'OUTSIDE', (x, y)
    return int(grid.data[y * meta.size_x + x]), (x, y)


def compare(evaluation, grid):
    meta = grid.metadata
    q = meta.origin.orientation
    if abs(q.x) + abs(q.y) + abs(q.z) > 1e-6:
        print('  SKIP: rotated grid origin is unsupported')
        return
    if meta.resolution <= 0 or len(grid.data) != meta.size_x * meta.size_y:
        print('  SKIP: invalid grid dimensions/data')
        return
    if evaluation.header.frame_id != grid.header.frame_id:
        print('  SKIP: frame mismatch; no asynchronous TF substitution')
        return
    reasons, indices, starts = Counter(), Counter(), Counter()
    examples = []
    for candidate in evaluation.twists:
        poses = candidate.traj.poses
        if not poses:
            reasons['EMPTY_TRAJECTORY'] += 1
            continue
        starts[str(cell_cost(grid, poses[0])[0])] += 1
        for i, pose in enumerate(poses):
            cost, cell = cell_cost(grid, pose)
            if cost == 'OUTSIDE' or cost in (253, 254, 255):
                reasons[str(cost)] += 1
                indices[i] += 1
                if len(examples) < 3:
                    velocity = candidate.traj.velocity
                    examples.append(dict(vx=round(velocity.x, 5),
                        wz=round(velocity.theta, 5), first_bad_index=i,
                        xy=[round(pose.x, 5), round(pose.y, 5)], cell=cell, cost=cost))
                break
        else:
            reasons['NO_REJECTION_IN_SNAPSHOT'] += 1
    print('  START_COST_COUNTS:', dict(starts))
    print('  FIRST_BAD_COST_COUNTS:', dict(reasons))
    print('  FIRST_BAD_POSE_INDEX_COUNTS:', dict(sorted(indices.items())))
    print('  EXAMPLES:', examples)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    args = parser.parse_args()
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    files = list(args.bag.glob('*.db3'))
    if len(files) != 1:
        parser.error('Expected exactly one db3 file.')
    conn = sqlite3.connect(files[0].resolve().as_uri() + '?mode=ro', uri=True)
    names = ('/evaluation', '/local_costmap/costmap_raw')
    types = {name: get_message(kind) for name, kind in conn.execute(
        'SELECT name,type FROM topics') if name in names}
    if set(types) != set(names):
        parser.error('Missing evaluation or local raw costmap topic.')
    grids, failures = [], []
    try:
        for receipt, name, data in conn.execute(
                'SELECT m.timestamp,t.name,m.data FROM messages m '
                'JOIN topics t ON t.id=m.topic_id WHERE t.name IN (?,?) '
                'ORDER BY m.timestamp,m.id', names):
            msg = deserialize_message(data, types[name])
            if name == names[1]:
                grids.append((receipt, msg))
            elif msg.twists and all(c.total < 0 for c in msg.twists):
                failures.append((receipt, msg))
    finally:
        conn.close()
    print('BAG:', args.bag)
    print('Local map snapshots:', len(grids), 'all-negative evaluations:', len(failures))
    print('253=inscribed inflated; 254=lethal; 255=unknown; OUTSIDE=off grid.')
    print('Brackets use bag receipt times, NOT exact controller map versions.')
    print('Index 0 means first stored trajectory pose, not an independently measured robot pose.')
    times = [row[0] for row in grids]
    for receipt, evaluation in failures:
        rejected = Counter(s.name for c in evaluation.twists for s in c.scores if s.raw_score < 0)
        print('\nINVALID_EVAL', round(stamp(evaluation.header.stamp), 6),
              'frame=', evaluation.header.frame_id, 'candidates=', len(evaluation.twists),
              'recorded_rejections=', dict(rejected))
        index = bisect.bisect_right(times, receipt)
        for label, i in (('BEFORE', index - 1), ('AFTER', index)):
            if not 0 <= i < len(grids):
                print(label, 'MISSING')
                continue
            grid_receipt, grid = grids[i]
            meta = grid.metadata
            print(label, 'receipt_delta_s=', round((grid_receipt - receipt) / 1e9, 6),
                  'header_stamp=', round(stamp(grid.header.stamp), 6),
                  'update_stamp=', round(stamp(meta.update_time), 6),
                  'frame=', grid.header.frame_id, 'resolution=', round(meta.resolution, 6),
                  'size=', (meta.size_x, meta.size_y),
                  'origin=', (round(meta.origin.position.x, 5), round(meta.origin.position.y, 5)))
            compare(evaluation, grid)


if __name__ == '__main__':
    main()
