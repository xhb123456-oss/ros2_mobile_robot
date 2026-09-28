"""Read a single SQLite rosbag without publishing messages or starting ROS nodes."""
import argparse
import bisect
import json
import math
import sqlite3
from pathlib import Path


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def yaw(q):
    return math.degrees(math.atan2(2 * (q.w*q.z + q.x*q.y),
                                   1 - 2 * (q.y*q.y + q.z*q.z)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--trials', type=int, nargs='+', default=[1, 2],
                        help='Trial numbers to inspect (default: 1 2).')
    parser.add_argument('--before', type=float, default=2.0,
                        help='Seconds before finish to show (default: 2).')
    args = parser.parse_args()
    if not math.isfinite(args.before) or args.before < 0:
        parser.error('--before must be finite and nonnegative.')
    dbs = list(args.bag.glob('*.db3'))
    if len(dbs) != 1:
        parser.error('Expected exactly one .db3 file; split bags are not supported.')
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    events = [json.loads(line) for line in
              (args.results / 'events.jsonl').read_text().splitlines()]
    finishes = [e for e in events if e['event'] == 'finish_sample']
    if not finishes:
        parser.error('No finish_sample events in results.')
    conn = sqlite3.connect(dbs[0].resolve().as_uri() + '?mode=ro', uri=True)
    types = {name: get_message(typ) for name, typ in conn.execute(
        'SELECT name, type FROM topics') if name in (
            '/clock', '/tf', '/amcl_pose', '/received_global_plan', '/transformed_global_plan')}
    print('BAG:', args.bag)
    print('Topic / count / first-last bag receipt seconds:')
    for row in conn.execute('SELECT t.name, count(*), min(m.timestamp), max(m.timestamp) '
                            'FROM messages m JOIN topics t ON m.topic_id=t.id GROUP BY t.name'):
        print(row[0], row[1], round(row[2]/1e9, 3), round(row[3]/1e9, 3))
    clocks, clock_receipts, corrections, amcl = [], [], [], []
    previous = None
    plan_ends = {'/received_global_plan': [], '/transformed_global_plan': []}
    for receipt, name, blob in conn.execute(
            "SELECT m.timestamp,t.name,m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
            "WHERE t.name IN ('/clock','/tf','/amcl_pose','/received_global_plan',"
            "'/transformed_global_plan') ORDER BY m.timestamp,m.id"):
        msg = deserialize_message(blob, types[name])
        if name == '/clock':
            clocks.append(stamp(msg.clock))
            clock_receipts.append(receipt)
        elif name == '/amcl_pose':
            p = msg.pose.pose
            amcl.append((stamp(msg.header.stamp), receipt, p.position.x, p.position.y,
                         yaw(p.orientation)))
        elif name in plan_ends:
            if msg.poses:
                end = msg.poses[-1]
                plan_ends[name].append((receipt, stamp(msg.header.stamp), msg.header.frame_id,
                    stamp(end.header.stamp), end.pose.position.x, end.pose.position.y,
                    yaw(end.pose.orientation)))
        else:
            for tf in msg.transforms:
                if (tf.header.frame_id.lstrip('/'), tf.child_frame_id.lstrip('/')) != ('map', 'odom'):
                    continue
                p, q = tf.transform.translation, tf.transform.rotation
                value = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
                if previous is None or any(abs(a-b) > 1e-9 for a, b in zip(value, previous)):
                    corrections.append((stamp(tf.header.stamp), receipt, p.x, p.y, yaw(q)))
                previous = value
    conn.close()
    if not clocks:
        raise RuntimeError('No clock messages recorded.')
    if any(b < a for a, b in zip(clocks, clocks[1:])):
        raise RuntimeError('Clock moved backwards; do not interpret windows across a reset.')
    sample_times = [e['snapshot']['tf_stamp'] for e in finishes]
    print('Clock range:', clocks[0], clocks[-1])
    print('Finish TF range:', min(sample_times), max(sample_times))
    print('Clock spans finish samples:', clocks[0] <= min(sample_times)
          and clocks[-1] >= max(sample_times))
    print('Coverage above is only a range check, not proof of zero dropped messages.')
    print('TF rows show correction VALUE changes only; unchanged rebroadcasts omitted.')
    print('recv_clock is the last recorded clock at receipt (approximate, not a message stamp).')
    for trial in args.trials:
        selected = [e for e in finishes if e['trial'] == trial]
        if not selected:
            continue
        start = selected[0]['snapshot']['tf_stamp']
        print('\nTRIAL', trial, 'finish t0=', start)
        for e in selected:
            s = e['snapshot']
            print('FINISH', e['nominal_delay_s'], 'stamp=', s['tf_stamp'],
                  'map_xy_yaw=', [round(v, 6) for v in s['pose'][:2]]
                  + [round(math.degrees(s['pose'][2]), 6)],
                  'odom_stamp=', s['odom_tf_stamp'],
                  'odom_xy_yaw=', [round(v, 6) for v in s['odom_pose'][:2]]
                  + [round(math.degrees(s['odom_pose'][2]), 6)])
        for label, records in (('TF_CHANGE', corrections), ('AMCL', amcl)):
            # Include preceding value as context, then all records in the stamp window.
            ordered = sorted(records)
            before = [r for r in ordered if r[0] < start - args.before]
            window = before[-1:] + [r for r in ordered if start - args.before <= r[0] <= start + 4]
            for t, receipt, x, y, angle in window:
                index = bisect.bisect_right(clock_receipts, receipt) - 1
                recv_clock = clocks[index] if index >= 0 else None
                print(label, 'stamp=', round(t, 6), 'recv_clock=', recv_clock,
                      'xy_yaw=', round(x, 6), round(y, 6), round(angle, 6))
        print('PLAN_END: published path/pose stamps may be rewritten by conversion;')
        print('transformed path can be cropped, so its endpoint is not necessarily the navigation goal.')
        for topic, records in plan_ends.items():
            last_time, last_key = None, None
            for receipt, path_stamp, frame, end_stamp, x, y, angle in records:
                index = bisect.bisect_right(clock_receipts, receipt) - 1
                t = clocks[index] if index >= 0 else None
                if t is None or not start - args.before <= t <= start + 0.5:
                    continue
                key = (path_stamp, frame)
                if last_time is not None and t - last_time < 0.49 and key == last_key:
                    continue
                print('PLAN_END', topic, 'recv_clock=', t, 'path_stamp=', round(path_stamp, 6),
                      'frame=', frame, 'end_stamp=', round(end_stamp, 6),
                      'xy_yaw=', round(x, 6), round(y, 6), round(angle, 6))
                last_time, last_key = t, key


if __name__ == '__main__':
    main()
