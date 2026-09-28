"""Read-only global grid connectivity around a recorded planner error.

Snapshot/nearest-pose diagnostics, not a replay of NavFn's internal potential.
"""
import argparse
import bisect
from collections import deque
import json
import math
from pathlib import Path
import sqlite3
import subprocess


def connected(data, width, height, start, goal):
    # Diagnostic four-neighbor search; clear robot start like the planner.
    seen = {start}
    queue = deque([start])
    while queue:
        x, y = queue.popleft()
        if (x, y) == goal:
            return True
        for a, b in ((x-1,y), (x+1,y), (x,y-1), (x,y+1)):
            if (0 < a < width-1 and 0 < b < height-1
                    and (a,b) not in seen and data[b*width+a] < 253):
                seen.add((a,b))
                queue.append((a,b))
    return False


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag', type=Path, required=True)
    p.add_argument('--results', type=Path, required=True)
    p.add_argument('--replay', type=Path, help='Optional compiled offline NavFn probe')
    args = p.parse_args()
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    events = [json.loads(s) for s in (args.results/'events.jsonl').read_text().splitlines()]
    observations = [e['snapshot'] for e in events if e.get('snapshot') and 'sim_time' in e['snapshot']]
    starts = sorted((e['snapshot']['sim_time'],e['trial']) for e in events if e['event']=='raw_plan')
    dbs = list(args.bag.glob('*.db3'))
    if len(dbs) != 1:
        p.error('Expected one db3 file')
    conn = sqlite3.connect(dbs[0].resolve().as_uri()+'?mode=ro',uri=True)
    names = ('/clock','/rosout','/global_costmap/costmap_raw')
    topics = {i:(n,get_message(t)) for i,n,t in conn.execute('SELECT id,name,type FROM topics') if n in names}
    maps, errors, clock = [], [], None
    print('Reading clock, rosout and global maps only...',flush=True)
    for receipt,topic,blob in conn.execute(
            'SELECT timestamp,topic_id,data FROM messages WHERE topic_id IN ('+
            ','.join('?' for _ in topics)+') ORDER BY timestamp,id',tuple(topics)):
        name,typ = topics[topic]
        msg = deserialize_message(blob,typ)
        if name == '/clock':
            new = msg.clock.sec + msg.clock.nanosec*1e-9
            if clock is not None and new < clock:
                raise RuntimeError('Clock reset; association ambiguous')
            clock = new
        elif name == '/global_costmap/costmap_raw':
            maps.append((receipt,clock,msg))
        elif 'Failed to create a plan from potential' in msg.msg:
            errors.append((receipt,clock))
    conn.close()
    print('Maps:',len(maps),'extraction errors:',len(errors))
    print('Receipt brackets are approximate; snapshots may differ from the planner map.')
    times = [x[0] for x in maps]
    for receipt,t in errors:
        if t is None or not observations:
            print('Missing clock/observations'); continue
        s = min(observations,key=lambda x:abs(x['sim_time']-t))
        trial = next((n for a,n in reversed(starts) if a <= t),None)
        print('\nERROR sim~',t,'trial~',trial,'sample_dt=',s['sim_time']-t,'pose=',s['pose'])
        if trial is None or abs(s['sim_time']-t) > .3:
            print('No sufficiently close trial pose'); continue
        target = (1.5,2.0) if trial%2 else (1.0,-1.0)
        k = bisect.bisect_right(times,receipt)
        for label,i in (('BEFORE',k-1),('AFTER',k)):
            if not 0 <= i < len(maps): continue
            rx,ct,g = maps[i]
            m = g.metadata
            print(label,'receipt_delta_s=',round((rx-receipt)/1e9,4),'recv_clock=',ct,
                  'stamp=',g.header.stamp.sec+g.header.stamp.nanosec*1e-9,
                  'frame=',g.header.frame_id,'resolution=',m.resolution,'size=',(m.size_x,m.size_y))
            q = m.origin.orientation
            if g.header.frame_id.lstrip('/') != 'map' or abs(q.x)+abs(q.y)+abs(q.z)>1e-6:
                print('Unsupported frame/orientation'); continue
            if m.resolution<=0 or len(g.data)!=m.size_x*m.size_y:
                print('Invalid grid'); continue
            def cell(x,y):
                return (math.floor((x-m.origin.position.x)/m.resolution),
                        math.floor((y-m.origin.position.y)/m.resolution))
            a,b = cell(*s['pose'][:2]),cell(*target)
            if not all(0<=x<m.size_x and 0<=y<m.size_y for x,y in (a,b)):
                print('Start/goal outside grid',a,b); continue
            for name,(x,y) in (('START',a),('GOAL',b)):
                print(name,'cell=',(x,y),'cost=',int(g.data[y*m.size_x+x]))
                print('5x5 costs, rows from high y to low y:')
                for v in range(y+2,y-3,-1):
                    print(' '.join(str(int(g.data[v*m.size_x+u])) if 0<=u<m.size_x and 0<=v<m.size_y
                                   else 'OUT' for u in range(x-2,x+3)))
            print('FOUR_NEIGHBOR_REACHABLE_EXACT_GOAL=',connected(g.data,m.size_x,m.size_y,a,b))
            # Humble NavfnPlanner uses round, unlike Costmap2D's floor mapping.
            def navfn_cell(x,y):
                return (math.floor((x-m.origin.position.x)/m.resolution+.5),
                        math.floor((y-m.origin.position.y)/m.resolution+.5))
            na,nb = navfn_cell(*s['pose'][:2]),navfn_cell(*target)
            print('NAVFN_ROUNDED_CELLS start=',na,'goal=',nb,'start_clear_floor_cell=',a)
            if args.replay:
                header = [m.size_x,m.size_y,*a,*na,*nb]
                payload = ' '.join(map(str,header))+'\n'+' '.join(map(str,g.data))+'\n'
                subprocess.run([str(args.replay.resolve())],input=payload,text=True,check=True)
    print('Connectivity check excludes costs >=253, clears start and blocks outer border.')
    if args.replay:
        print('Native NavFn probe uses snapshot maps and an approximate robot pose; no tolerance search or exact planner-state replay.')
    else:
        print('No native NavFn probe requested; connectivity alone does not test path extraction.')


if __name__ == '__main__':
    main()
