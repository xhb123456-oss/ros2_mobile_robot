"""Read saved test results, console feedback and rosbag logs; never publish ROS data."""
import argparse
import bisect
from collections import Counter
import csv
import json
import math
from pathlib import Path
import re
import sqlite3


def motion_window(events, start, end):
    samples = [e['snapshot'] for e in events if e['event'] == 'pose_sample'
               and e.get('snapshot') and start <= e['snapshot']['sim_time'] <= end]
    if not samples:
        raise ValueError('No pose samples in requested simulation-time window')
    print('MOTION_WINDOW', start, end, 'samples=', len(samples))
    print('One sample per second; xy in m, yaw in degrees, wz in rad/s.')
    print('Each velocity group: vx,wz,receive_age_s; asynchronous observations.')
    print('sim,map_x,map_y,map_yaw,odom_x,odom_y,odom_yaw,nav[vx,wz,age],cmd[vx,wz,age],odom[vx,wz,age]')
    printed = set()
    for s in samples:
        second = math.floor(s['sim_time'])
        if second in printed:
            continue
        printed.add(second)
        values = [s['sim_time'], *s['pose'][:2], math.degrees(s['pose'][2]),
                  *s['odom_pose'][:2], math.degrees(s['odom_pose'][2])]
        groups = []
        for topic in ('/cmd_vel_nav', '/cmd_vel', '/odom'):
            m = s.get('motion', {}).get(topic)
            groups.append('missing' if m is None else
                          '/'.join(f'{m[k]:.4f}' for k in ('vx', 'wz', 'rx_age')))
        print(','.join(f'{v:.4f}' for v in values) + ',' + ','.join(groups))
    for key in ('pose', 'odom_pose'):
        origin = samples[0][key]
        displacements = [math.hypot(s[key][0]-origin[0], s[key][1]-origin[1]) for s in samples]
        print(key, 'max_distance_from_first_m=', round(max(displacements), 6),
              'end_distance_from_first_m=', round(displacements[-1], 6))
    print('Window origin is not the progress checker internal baseline; no reset time inferred.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--bag', type=Path)
    parser.add_argument('--console', type=Path,
                        help='Explicit console log; must reference this results directory')
    parser.add_argument('--timeline', action='store_true',
                        help='Show selected rosout messages with approximate simulation time and nearby samples')
    parser.add_argument('--motion-window', type=float, nargs=2, metavar=('START', 'END'),
                        help='Only print existing pose/velocity samples in a simulation-time window')
    args = parser.parse_args()
    root = args.results.parent
    rows = list(csv.DictReader((args.results / 'results.csv').open(encoding='utf-8-sig')))
    events = [json.loads(s) for s in (args.results / 'events.jsonl').read_text().splitlines()]
    if args.motion_window:
        if not all(math.isfinite(t) for t in args.motion_window) or args.motion_window[0] >= args.motion_window[1]:
            parser.error('Motion window must have finite START < END')
        motion_window(events, *args.motion_window)
        return
    print('RESULTS:', args.results)
    print('SUMMARY:', (args.results / 'summary.json').read_text())
    if args.console:
        if args.results.name not in args.console.read_text():
            parser.error('--console does not reference the selected results directory')
        logs = [args.console]
    else:
        logs = [p for p in sorted(root.glob('arrival_console_endurance_*.log'))
                if args.results.name in p.read_text()]
    recoveries = {}
    if logs:
        print('CONSOLE:', logs[-1])
        trial = None
        for line in logs[-1].read_text().splitlines():
            header = re.match(r'\[(\d+)/\d+\]', line.strip())
            if header:
                trial = int(header[1])
            value = re.search(r'recoveries=(\d+)', line)
            if value and trial is not None:
                recoveries[trial] = max(recoveries.get(trial, 0), int(value[1]))
    else:
        print('WARNING: matching console not found; recovery counts unknown.')
    print('trial,target,wall_s,xy_cm,yaw_deg,max_printed_recoveries')
    for row in rows:
        print(row['trial'], row['target'], round(float(row['wall_s']), 2),
              round(float(row['position_error_m'])*100, 3),
              round(float(row['yaw_error_deg']), 3),
              recoveries.get(int(row['trial']), 'UNKNOWN'), sep=',')
    samples = [e for e in events if e['event'] == 'finish_sample']
    print('Post-finish checks use original 0.15 m / 0.15 rad limits, without rewriting PASS.')
    for delay in (0.5, 1.0, 2.0, 3.0):
        batch = [e for e in samples if e['nominal_delay_s'] == delay]
        counts = Counter(e['trial'] for e in batch)
        missing = [int(r['trial']) for r in rows if counts[int(r['trial'])] != 1]
        print('DELAY', delay, 'samples=', len(batch), 'missing_or_duplicate_trials=', missing)
        if batch:
            print('MAX xy_cm=', round(max(e['position_error_m'] for e in batch)*100, 4),
                  'yaw_deg=', round(max(abs(e['yaw_error_deg']) for e in batch), 4))
        for e in batch:
            if e['position_error_m'] > 0.15 or abs(e['yaw_error_deg']) > math.degrees(0.15):
                print('EXCEEDS:', e['trial'], delay, e['position_error_m'], e['yaw_error_deg'])
    warnings = [e for e in events if e['event'] in ('observation_warning', 'bad_observation')]
    print('Observation warnings:', len(warnings))
    for e in warnings[:20]:
        print(e['event'], e.get('reason'))
    bags = sorted(p for p in root.glob('arrival_tf_endurance_*') if p.is_dir())
    bag = args.bag or (bags[-1] if bags else None)
    if bag is None:
        print('No endurance bag found.')
        return
    print('BAG:', bag, '(latest matching name selected)' if args.bag is None else '')
    dbs = list(bag.glob('*.db3'))
    if len(dbs) != 1:
        print('Bag inspection requires exactly one db3; not inspected.')
        return
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    conn = sqlite3.connect(dbs[0].resolve().as_uri() + '?mode=ro', uri=True)
    print('Topic counts:', list(conn.execute(
        'SELECT t.name,count(*) FROM messages m JOIN topics t ON m.topic_id=t.id GROUP BY t.name')))
    clock_type = get_message('rosgraph_msgs/msg/Clock')
    clocks, clock_receipts = [], []
    for receipt, blob in conn.execute("SELECT m.timestamp,m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
                               "WHERE t.name='/clock' ORDER BY m.timestamp,m.id"):
        s = deserialize_message(blob, clock_type).clock
        clocks.append(s.sec + s.nanosec*1e-9)
        clock_receipts.append(receipt)
    if clocks and samples:
        times = [e['snapshot']['tf_stamp'] for e in samples]
        monotonic = all(b >= a for a, b in zip(clocks, clocks[1:]))
        print('CLOCK:', clocks[0], clocks[-1], 'monotonic=', monotonic,
              'spans_finish_samples=', monotonic and clocks[0] <= min(times) and clocks[-1] >= max(times))
    log_type = get_message('rcl_interfaces/msg/Log')
    counts, ranges = Counter(), {}
    details = []
    total = 0
    for receipt, blob in conn.execute("SELECT m.timestamp,m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
                               "WHERE t.name='/rosout' ORDER BY m.timestamp,m.id"):
        msg = deserialize_message(blob, log_type)
        total += 1
        if msg.level < 30 and not re.search(r'extrapolat|transform.*(fail|timed out)|Failed to make progress',
                                            msg.msg, re.I):
            continue
        key = (msg.level, msg.name, msg.msg)
        counts[key] += 1
        t = msg.stamp.sec + msg.stamp.nanosec*1e-9
        if key not in ranges:
            ranges[key] = [t, t]
        ranges[key][1] = t
        if msg.level >= 40 or 'No valid trajectories' in msg.msg:
            index = bisect.bisect_right(clock_receipts, receipt) - 1
            details.append((clocks[index] if index >= 0 else None, msg.name, msg.msg))
    conn.close()
    print('ROSOUT total=', total, 'selected=', sum(counts.values()), 'unique=', len(counts))
    for key, count in counts.most_common(40):
        print('LOG', count, 'first_last=', ranges[key], 'level_node_message=', key)
    if len(counts) > 40:
        print('Additional unique messages omitted:', len(counts)-40)
    if args.timeline:
        starts = sorted((e['snapshot']['sim_time'], e['trial']) for e in events
                        if e['event'] == 'raw_plan')
        observed = sorted((e['snapshot']['sim_time'], i, e['snapshot'])
                          for i, e in enumerate(events) if e.get('snapshot')
                          and 'sim_time' in e['snapshot'])
        print('TIMELINE: approximate receive-clock association, not synchronized node internals.')
        for t, node, message in details:
            if t is None:
                print('LOG without preceding clock:', node, message)
                continue
            candidates = [(s, trial) for s, trial in starts if s <= t]
            trial = candidates[-1][1] if candidates else 'UNKNOWN'
            print('AT sim~', t, 'trial~', trial, node, message)
            if observed:
                nearest = min(observed, key=lambda item: abs(item[0]-t))
                s = nearest[2]
                print('  sample_dt=', round(nearest[0]-t, 3),
                      'map_xy_yaw=', [round(v, 4) for v in s['pose']],
                      '(yaw radians)')
                if abs(nearest[0]-t) <= 0.3:
                    for topic in ('/cmd_vel_nav', '/cmd_vel', '/odom'):
                        m = s.get('motion', {}).get(topic)
                        if m:
                            print(' ', topic, {k: round(m[k], 5) for k in ('vx', 'vy', 'wz', 'rx_age')})
                else:
                    print('  No close sample; omit motion association.')
    print('Printed recoveries are sampled feedback; clock coverage is not a no-loss guarantee.')


if __name__ == '__main__':
    main()
