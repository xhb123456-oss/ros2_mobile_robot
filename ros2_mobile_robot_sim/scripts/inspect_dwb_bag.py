"""Offline DWB diagnostics. Opens one SQLite bag read-only, without ROS publishers."""
import argparse
import bisect
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import statistics
import struct
import sys
import time


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def path_key(msg):
    digest = hashlib.sha256(msg.header.frame_id.encode())
    for item in msg.poses:
        p, q = item.pose.position, item.pose.orientation
        digest.update(struct.pack('<7d', p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    return digest.digest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--results', type=Path, required=True)
    args = parser.parse_args()
    print('PROGRESS: opening read-only DWB analysis...', file=sys.stderr, flush=True)
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    dbs = list(args.bag.glob('*.db3'))
    if len(dbs) != 1:
        parser.error('Expected a single db3 file.')
    events = [json.loads(line) for line in (args.results/'events.jsonl').read_text().splitlines()]
    starts = {e['trial']: e['snapshot']['sim_time'] for e in events if e['event']=='raw_plan'}
    ends = {e['trial']: e['snapshot']['sim_time'] for e in events
            if e['event']=='finish_sample' and e['nominal_delay_s']==0.0}
    samples = sorted((e['snapshot']['sim_time'], i, e['snapshot']) for i, e in enumerate(events)
                     if e.get('snapshot') and 'sim_time' in e['snapshot'])
    sample_times = [v[0] for v in samples]

    def trial_at(t):
        return next((i for i in starts if starts[i] <= t <= ends.get(i, -1)), None)

    conn = sqlite3.connect(dbs[0].resolve().as_uri()+'?mode=ro', uri=True)
    wanted = ('/clock', '/received_global_plan', '/evaluation', '/rosout')
    types = {name: get_message(typ) for name, typ in conn.execute('SELECT name,type FROM topics')
             if name in wanted}
    topic_names = {topic_id: name for topic_id, name in conn.execute('SELECT id,name FROM topics')
                   if name in wanted}
    placeholders = ','.join('?' for _ in topic_names)
    total = conn.execute(
        f'SELECT count(*) FROM messages WHERE topic_id IN ({placeholders})',
        tuple(topic_names)).fetchone()[0]
    began = last_progress = time.monotonic()
    processed = 0
    print(f'PROGRESS: reading {total} selected messages; trajectory decoding may take minutes.',
          file=sys.stderr, flush=True)
    plans, evaluations, logs, clocks = [], [], [], []
    controller_path_events = []
    recv_clock = None
    for receipt, topic_id, data in conn.execute(
        f'SELECT timestamp,topic_id,data FROM messages WHERE topic_id IN ({placeholders}) '
        'ORDER BY timestamp,id', tuple(topic_names)):
        name = topic_names[topic_id]
        msg = deserialize_message(data, types[name])
        processed += 1
        now = time.monotonic()
        if now - last_progress >= 5:
            print(f'PROGRESS: {processed}/{total} messages; '
                  f'last clock={recv_clock}; elapsed={now-began:.0f}s',
                  file=sys.stderr, flush=True)
            last_progress = now
        if name == '/clock':
            recv_clock = stamp(msg.clock)
            clocks.append(recv_clock)
        elif name == '/received_global_plan':
            plans.append((receipt, recv_clock, stamp(msg.header.stamp), path_key(msg)))
        elif name == '/rosout':
            if msg.name == 'controller_server' and msg.msg in (
                    'Passing new path to controller.',
                    'Received a goal, begin computing control effort.'):
                controller_path_events.append((receipt, recv_clock, msg.msg))
            if msg.level >= 30:
                logs.append((msg.level, msg.name, msg.msg))
        else:
            t = stamp(msg.header.stamp)
            trial = trial_at(t)
            if trial is None:
                continue
            best = msg.twists[msg.best_index] if 0 <= msg.best_index < len(msg.twists) else None
            rejected, forward_rejected = Counter(), Counter()
            forward_nonnegative = 0
            forward_min = None
            for candidate in msg.twists:
                is_forward = candidate.traj.velocity.x >= 0.03
                if candidate.total < 0:
                    reasons = [s.name for s in candidate.scores if s.raw_score < 0] or ['unknown']
                    rejected.update(reasons)
                    if is_forward:
                        forward_rejected.update(reasons)
                elif is_forward:
                    forward_nonnegative += 1
                    if forward_min is None or candidate.total < forward_min.total:
                        forward_min = candidate
            near = bisect.bisect_left(sample_times, t)
            choices = samples[max(0, near-1):near+1]
            observation = min(choices, key=lambda v: abs(v[0]-t)) if choices else None
            distance = None
            if observation and abs(observation[0]-t) <= 0.3:
                x, y = observation[2]['pose'][:2]
                target = (1.5, 2.0) if trial % 2 else (1.0, -1.0)
                distance = math.hypot(x-target[0], y-target[1])
            if best is None or best.total < 0:
                evaluations.append(dict(t=t, receipt=receipt, trial=trial, valid=False,
                    rejected=rejected, forward_rejected=forward_rejected,
                    candidates=len(msg.twists), distance=distance,
                    pose=None if observation is None or abs(observation[0]-t)>0.3 else observation[2]['pose'],
                    motion=None if observation is None or abs(observation[0]-t)>0.3 else observation[2].get('motion')))
                continue
            evaluations.append(dict(t=t, receipt=receipt, trial=trial, valid=True,
                vx=best.traj.velocity.x, wz=best.traj.velocity.theta, distance=distance,
                rejected=rejected, forward_rejected=forward_rejected,
                forward_nonnegative=forward_nonnegative,
                best_total=best.total,
                forward_min=None if forward_min is None else dict(
                    recorded_total=round(forward_min.total, 4),
                    vx=round(forward_min.traj.velocity.x, 4),
                    wz=round(forward_min.traj.velocity.theta, 4),
                    scores={s.name: round(s.raw_score*s.scale, 4) for s in forward_min.scores}),
                scores={s.name: round(s.raw_score*s.scale, 4) for s in best.scores}))
    conn.close()
    print(f'PROGRESS: read complete ({processed} messages, {time.monotonic()-began:.1f}s); summarizing.',
          file=sys.stderr, flush=True)
    print('BAG:', args.bag)
    print('Clock range:', (clocks[0], clocks[-1]) if clocks else 'MISSING')
    if any(b < a for a,b in zip(clocks, clocks[1:])):
        raise RuntimeError('Clock reset detected; time association is ambiguous.')
    print('Trial association uses simulation stamps; plan receive-clock is approximate.')
    print('Forward means vx>=0.03. Nonnegative candidate scores may be short-circuited: not all are fully evaluated.')
    print('Repeated geometry excludes all header timestamps. Plan publication is not proof of changed geometry.')
    print('received_global_plan also publishes during pruning; its frequency is NOT the critic reset frequency.')
    print('Controller path logs mark the code path toward setPlan, not exact reset timestamps.')
    for trial in sorted(starts):
        ev = [e for e in evaluations if e['trial']==trial]
        pp = [p for p in plans if p[1] is not None and starts[trial] <= p[1] <= ends.get(trial, -1)]
        deltas = [(b[0]-a[0])/1e9 for a,b in zip(pp, pp[1:])]
        duplicates = sum(a[3]==b[3] for a,b in zip(pp, pp[1:]))
        print('\nTRIAL', trial, 'window=', starts[trial], ends.get(trial),
              'evaluations=', len(ev), 'plans=', len(pp),
              'identical_adjacent_geometry=', duplicates,
              'unique_plan_stamps=', len({p[2] for p in pp}),
              'median_plan_receipt_interval_s=', round(statistics.median(deltas), 4) if deltas else None)
        rejection_totals = Counter()
        previous_sign, flips, examples = 0, 0, []
        plan_receipts = [p[0] for p in pp]
        path_events = [p for p in controller_path_events if p[1] is not None
                       and starts[trial] <= p[1] <= ends.get(trial, -1)]
        print('CONTROLLER_PATH_EVENTS (approx recv clock):', [(p[1],p[2]) for p in path_events])
        path_event_receipts = [p[0] for p in path_events]
        for e in ev:
            rejection_totals.update(e['rejected'])
            if not e['valid']:
                print('INVALID_EVAL', round(e['t'],3), 'candidates=', e['candidates'],
                      'rejected=', dict(e['rejected']), 'forward_rejected=', dict(e['forward_rejected']),
                      'nearby_map_pose_xy_yaw_rad=', e['pose'])
                if e['motion']:
                    print('  INVALID_MOTION', {topic: {k: round(m[k],5) for k in ('vx','wz','rx_age')}
                          for topic,m in e['motion'].items()})
                continue
            if e['distance'] is None or e['distance'] <= 0.3 or abs(e['vx']) > 0.03:
                previous_sign = 0
                continue
            sign = 1 if e['wz'] > 1e-6 else -1 if e['wz'] < -1e-6 else 0
            flip = sign != 0 and previous_sign != 0 and sign != previous_sign
            flips += int(flip)
            if sign:
                previous_sign = sign
            if flip or not examples or e['t']-examples[-1]['t'] >= 1.0:
                examples.append(e)
        print('no_valid_best=', sum(not e['valid'] for e in ev),
              'rejected_candidates_by_first_negative_critic=', dict(rejection_totals))
        print('low_vx_turn_sign_flips_away_from_goal=', flips,
              'examples=', len(examples), '(show at most 30)')
        for e in examples[:30]:
            index = bisect.bisect_right(plan_receipts, e['receipt'])-1
            age = (e['receipt']-plan_receipts[index])/1e9 if index>=0 else None
            update_index = bisect.bisect_right(path_event_receipts, e['receipt'])-1
            update_age = ((e['receipt']-path_event_receipts[update_index])/1e9
                          if update_index >= 0 else None)
            print('EVAL', round(e['t'], 3), 'vx/wz=', round(e['vx'],4), round(e['wz'],4),
                  'goal_dist=', round(e['distance'],3),
                  'last_plan_receipt_age=', round(age,4) if age is not None else None,
                  'last_controller_path_event_age=', round(update_age,4) if update_age is not None else None,
                  'forward_nonnegative=', e['forward_nonnegative'],
                  'forward_rejected=', dict(e['forward_rejected']), 'best_scores=', e['scores'])
            print('  BEST_TOTAL=', round(e['best_total'],4),
                  'FORWARD_MIN_RECORDED (may be partial lower bound)=', e['forward_min'])
    print('\nROSOUT warnings/errors:', dict(Counter(logs)))
    print('This is offline evidence, not exact critic internal-state instrumentation or a no-message-loss check.')


if __name__ == '__main__':
    main()
