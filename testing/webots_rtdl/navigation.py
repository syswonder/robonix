# SPDX-License-Identifier: MulanPSL-2.0
"""Test real-model navigation, hard interruption, and natural-language cancellation."""
import asyncio
import json
import math
import os
from pathlib import Path
import shlex
import sys
import time
import uuid

OUT = Path(os.environ['RTDL_TEST_OUT']).resolve()
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(OUT / 'proto'))
import grpc
from google.protobuf.json_format import MessageToDict
import executor_pb2
import pilot_pb2
import robonix_contracts_pb2_grpc as rpc

NAV = 'robonix/service/navigation/navigate'
CAMERA = 'robonix/primitive/camera/snapshot'
OPTIONS = [('grpc.enable_http_proxy', 0), ('grpc.max_receive_message_length', 64*1024*1024)]


def save(name, value):
    (OUT / name).write_text(json.dumps(value, ensure_ascii=False, indent=2))


def distance(a, b):
    return math.hypot(a['x']-b['x'], a['y']-b['y'])


def goal_from_map(grid, pose, minimum):
    """Choose a clear bounded route, allowing the sensor blind area at the start."""
    if grid['frame'] != 'map' or abs(grid['origin']['yaw']) > 1e-5:
        raise RuntimeError('Goal selection requires an axis-aligned map grid')
    res, origin = grid['resolution'], grid['origin']
    radius = math.ceil(0.35/res)
    offsets = [(x, y) for x in range(-radius, radius+1)
               for y in range(-radius, radius+1) if math.hypot(x, y)*res <= 0.35]

    def free(x, y):
        """Reject obstacles; allow unknown only within 0.7 m of the initial robot pose."""
        cx, cy = math.floor((x-origin['x'])/res), math.floor((y-origin['y'])/res)
        for dx, dy in offsets:
            px, py = cx+dx, cy+dy
            if not (0 <= px < grid['width'] and 0 <= py < grid['height']):
                return False
            value = grid['data'][py*grid['width']+px]
            blind_area = math.hypot(origin['x']+(px+0.5)*res-pose['x'],
                                   origin['y']+(py+0.5)*res-pose['y']) <= 0.7
            if value > 0 or (value < 0 and not blind_area):
                return False
        return True

    candidates = []
    for i in range(32):
        heading = pose['yaw'] + i*math.tau/32
        length = 0.0
        for step in range(1, 51):
            d = step*0.05
            if not free(pose['x']+d*math.cos(heading), pose['y']+d*math.sin(heading)):
                break
            length = d
        if length >= minimum:
            # Prefer longer routes; break ties in favor of less initial rotation.
            turn = abs(math.atan2(math.sin(heading-pose['yaw']), math.cos(heading-pose['yaw'])))
            candidates.append((length-0.05*turn, length, heading))
    if not candidates:
        raise RuntimeError(f'No known free route >={minimum} m with 0.35 m clearance; inspect map')
    _, length, heading = max(candidates)
    return {'header': {'frame_id': 'map'}, 'pose': {
        'position': {'x': pose['x']+length*math.cos(heading),
                     'y': pose['y']+length*math.sin(heading), 'z': 0.0},
        'orientation': {'x': 0.0, 'y': 0.0,
                        'z': math.sin(heading/2), 'w': math.cos(heading/2)}}}


async def wait_until(predicate, timeout, description):
    """Wait for observed evidence rather than using a fixed delay to trigger control."""
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.1)
    raise TimeoutError(description)


async def active_plans():
    """Read the real Executor plan registry and fail on an unsuccessful query."""
    async with grpc.aio.insecure_channel('127.0.0.1:50061', options=OPTIONS) as ch:
        value = await rpc.RobonixSystemExecutorListActivePlansStub(ch).ListActivePlans(
            executor_pb2.ListActivePlans_Request(), timeout=5)
    if not value.success:
        raise RuntimeError(value.error)
    return json.loads(value.plans_json)


async def collect(stub, session, text, context, path, events, timeout=180):
    """Submit only task text/control context to Pilot and retain its timed events."""
    task = pilot_pb2.Task(task_id=str(uuid.uuid4()), session_id=session, text=text,
                         context_json=json.dumps(context), source=2, timestamp_ms=int(time.time()*1000))
    with (OUT / path).open('w') as log:
        async for event in stub.SubmitTask(task, timeout=timeout):
            item = MessageToDict(event, preserving_proto_field_name=True)
            events.append(item)
            log.write(json.dumps({'time': time.time(), 'event': item}, ensure_ascii=False)+'\n')
            log.flush()
            if item.get('final_text'):
                print(path, item['final_text'][:400], flush=True)


async def observe(process, state):
    """Store independent ROS evidence while exposing latest samples to the test."""
    with (OUT / 'navigation-ros.jsonl').open('w') as log:
        async for line in process.stdout:
            item = json.loads(line)
            log.write(line.decode())
            log.flush()
            if item['type'] == 'map':
                state['map'] = item
            else:
                state['latest'] = item
                state['samples'].append(item)


async def speed(stub, prefix, value=None):
    """Ask the real model to read/set navigation speed, validating its actual result."""
    name = 'get_speed_limit' if value is None else 'set_speed_limit'
    args = {} if value is None else {'percentage': value, 'persist': True}
    prompt = (f'只调用一次 nav2 的 robonix/service/navigation/{name}，参数为 '
              + json.dumps(args) + '。不要导航或执行其他动作，返回成功后回复并标记 done。')
    events = []
    save(prefix+'-input.json', {'prompt': prompt})
    await collect(stub, 'speed-'+str(uuid.uuid4()), prompt, {}, prefix+'.jsonl', events, timeout=60)
    leaves = [n for e in events if 'plan' in e for n in e['plan']['nodes'] if n.get('node_kind') == 2]
    terminals = [e['node_state'] for e in events if e.get('node_state', {}).get('node_kind') == 2
                 and e['node_state'].get('state') in (2, 3, 4, 5)]
    if not (len(leaves) == len(terminals) == 1 and terminals[0]['state'] == 2
            and leaves[0]['call']['contract_id'] == 'robonix/service/navigation/'+name):
        raise AssertionError('Speed preparation did not perform exactly the requested capability')
    output = json.loads(terminals[0]['leaf_result']['output'])
    if value is not None and (not output.get('accepted') or abs(output['effective_percentage']-value) > 0.01):
        raise AssertionError('Requested speed was not accepted')
    return output['effective_percentage']


def fresh(state):
    sample = state.get('latest', {})
    return time.time()-sample.get('time', 0) < 1 and time.time()-sample.get('odom', {}).get('received', 0) < 1


async def stopped(state, since):
    """Require three seconds of fresh low-velocity odometry and little displacement."""
    def stable():
        """Check a continuous observation window after the cancellation request."""
        recent = [s for s in state['samples'] if s['time'] >= max(since, time.time()-3.5)]
        return (fresh(state) and len(recent) >= 12 and recent[-1]['time']-recent[0]['time'] >= 3
                and all(s.get('odom', {}).get('v', 1) < 0.02
                        and abs(s.get('odom', {}).get('w', 1)) < 0.03 for s in recent)
                and max(distance(s['odom'], recent[0]['odom']) for s in recent) < 0.05)
    await wait_until(stable, 20, 'Robot did not remain stationary for three seconds')
    return time.time()


async def case(mode, state):
    """Exercise a real-model plan and check navigation or physical cancellation evidence."""
    prefix = 'navigation-'+mode
    if (OUT / (prefix+'-input.json')).exists():
        raise RuntimeError('Use a fresh output directory; existing evidence must not be overwritten')
    if (await active_plans())['count']:
        raise RuntimeError('This test requires an otherwise idle Executor')
    await wait_until(lambda: fresh(state) and 'map' in state, 30, 'No fresh ROS map and pose')
    before = state['latest']
    goal = goal_from_map(state['map'], before['pose'], 1.2 if mode == 'complete' else 0.8)
    session = prefix+'-'+str(uuid.uuid4())
    prompt = ('请在 Webots 中执行此导航任务：在同一个 sequence 中恰好两个 do，'
              '先调用 nav2 的 robonix/service/navigation/navigate 到达下面的 map 目标，'
              '再调用 tiago_camera 的 camera/snapshot 拍照。目标来自实时地图中的空闲区域。'
              '不要增加其他动作、读取文档或运行 shell；不允许重试或重新发起导航。'
              '失败或取消后停止剩余步骤，按实际结果回复；全部成功后标记 done。导航参数：'
              + json.dumps({'goal': goal}, ensure_ascii=False))
    save(prefix+'-input.json', {'mode': mode, 'session': session, 'before': before,
                              'goal': goal, 'prompt': prompt, 'map_time': state['map']['time']})
    save(prefix+'-map.json', state['map'])
    events, controls = [], []
    result = {'mode': mode, 'session': session, 'pass': False}
    executor_log = ROOT / 'examples/webots/rbnx-boot/logs/executor.log'
    log_offset = executor_log.stat().st_size
    async with grpc.aio.insecure_channel('127.0.0.1:50071', options=OPTIONS) as ch:
        stub = rpc.RobonixSystemPilotStub(ch)
        original_speed = None
        running = None
        try:
            if mode != 'complete':
                original_speed = await speed(stub, prefix+'-speed-before')
                await speed(stub, prefix+'-speed-test', 100.0)
            running = asyncio.create_task(collect(stub, session, prompt, {}, prefix+'.jsonl', events))
            await wait_until(lambda: any('plan' in e for e in events), 60, 'Pilot did not publish a plan')
            plan = next(e['plan'] for e in events if 'plan' in e)
            leaves = [n for n in plan['nodes'] if n.get('node_kind') == 2]
            if [n['call']['contract_id'] for n in leaves] != [NAV, CAMERA]:
                raise AssertionError('Model did not generate the requested navigation/camera plan')
            await wait_until(lambda: fresh(state) and distance(state['latest']['odom'], before['odom']) > 0.10
                             and state['latest']['odom']['v'] > 0.03
                             and any(g['status'] == 2 for g in state['latest'].get('goals', [])),
                             60, 'Navigation never produced measured translation with an executing ROS goal')
            moving = state['latest']
            result.update(plan_id=plan['plan_id'], before=before, moving=moving, goal=goal)
            nav_goals = [g['id'] for g in moving['goals'] if g['status'] == 2]
            if mode != 'complete':
                requested = time.time()
                text = ('取消当前导航任务和它后面的拍照步骤，立即停止机器人，不要重新导航或执行其他动作。'
                        '请通过 RTDL 的计划取消操作取消正在执行的计划，完成后简短确认。')
                context = {'abort_turn': True} if mode == 'abort' else {'steer': True}
                save(prefix+'-control-input.json', {'time': requested, 'session': session,
                                                    'text': '' if mode == 'abort' else text, 'context': context})
                await collect(stub, session, '' if mode == 'abort' else text, context,
                              prefix+'-control.jsonl', controls, timeout=90)
                result['control_requested_at'] = requested
                result['stationary_confirmed_at'] = await stopped(state, requested)
                await wait_until(lambda: all(any(g['id'] == gid and g['status'] == 5
                                                for g in state['latest'].get('goals', [])) for gid in nav_goals),
                                 10, 'Nav2 action did not report CANCELED for the interrupted goal')
            await asyncio.wait_for(asyncio.shield(running), timeout=180)
            await asyncio.sleep(1)
            result['active_after'] = await active_plans()
            result['after'] = state['latest']
            with executor_log.open('rb') as log:
                log.seek(log_offset)
                log_text = log.read().decode()
            (OUT / (prefix+'-executor.log')).write_text(log_text)
            dispatches = [line for line in log_text.splitlines() if 'dispatching call_id=' in line]
            counts = [sum('call_id='+n['call']['call_id']+' ' in line for line in dispatches) for n in leaves]
            result['dispatch_counts'] = counts
            result['navigation_dispatch_count'] = sum("contract='"+NAV+"'" in line for line in dispatches)
            result['camera_dispatch_count'] = sum("contract='"+CAMERA+"'" in line for line in dispatches)
            all_events = events + controls
            plans_seen = {e['plan']['plan_id'] for e in all_events if 'plan' in e}
            if plans_seen != {plan['plan_id']} or result['navigation_dispatch_count'] != 1:
                raise AssertionError('Model submitted an extra plan or navigation was dispatched again')
            # A steer transfers reply ownership to its subscriber; read both streams.
            terminal_events = [e['node_state'] for e in all_events if e.get('node_state', {}).get('node_kind') == 2
                               and e['node_state'].get('state') in (2, 3, 4, 5)]
            by_identity = {}
            for node in terminal_events:
                key = (node['plan_id'], node['op_id'])
                if key in by_identity and by_identity[key]['state'] != node['state']:
                    raise AssertionError('Conflicting terminal states for the same node')
                by_identity[key] = node
            terminals = list(by_identity.values())
            result['terminal_states'] = [{k: v for k, v in n.items() if k != 'leaf_result'} for n in terminals]
            if result['active_after']['count'] != 0:
                raise AssertionError('Active plans remained after the task ended')
            if mode == 'complete':
                goal_xy = goal['pose']['position']
                q = goal['pose']['orientation']
                target_yaw = 2*math.atan2(q['z'], q['w'])
                result['position_error_m'] = distance(state['latest']['pose'], goal_xy)
                result['yaw_error_rad'] = abs(math.atan2(math.sin(state['latest']['pose']['yaw']-target_yaw),
                                                         math.cos(state['latest']['pose']['yaw']-target_yaw)))
                result['actual_displacement_m'] = distance(state['latest']['pose'], before['pose'])
                expected_ids = {(plan['plan_id'], n['op_id']) for n in leaves}
                actual_ids = [(n['plan_id'], n['op_id']) for n in terminals]
                if not (counts == [1, 1] and len(actual_ids) == 2 and set(actual_ids) == expected_ids
                        and all(n['state'] == 2 for n in terminals)
                        and result['position_error_m'] < 0.5 and result['yaw_error_rad'] < 0.35
                        and result['actual_displacement_m'] > 0.7
                        and any(e.get('node_state', {}).get('state') == 7 for e in events)):
                    raise AssertionError('Navigation outcome, VERIFYING, or measured pose did not pass')
            else:
                if counts != [1, 0] or result['camera_dispatch_count'] != 0:
                    raise AssertionError('Navigation replayed or the post-navigation camera was dispatched')
                if mode == 'abort' and not any(e.get('status', {}).get('message') == 'interrupted' for e in events):
                    raise AssertionError('Pilot did not report the requested interruption')
                expected_cancel_ids = {(plan['plan_id'], n['op_id']) for n in leaves}
                if mode == 'cancel' and not (set(by_identity) == expected_cancel_ids
                                             and all(n['state'] == 4 for n in terminals)):
                    raise AssertionError('Natural-language cancellation did not cancel both RTDL leaves')
                recovery = []
                await collect(stub, session, '只读取一次 tiago_lidar 的 lidar/snapshot，不做其他动作，成功后回复并标记 done。',
                              {}, prefix+'-recovery.jsonl', recovery, timeout=60)
                recovery_leaves = [n for e in recovery if 'plan' in e for n in e['plan']['nodes'] if n.get('node_kind') == 2]
                recovery_terminals = [e['node_state'] for e in recovery if e.get('node_state', {}).get('node_kind') == 2
                                      and e['node_state'].get('state') in (2, 3, 4, 5)]
                result['recovery_success'] = (len(recovery_leaves) == len(recovery_terminals) == 1
                    and recovery_leaves[0]['call']['contract_id'] == 'robonix/primitive/lidar/snapshot'
                    and recovery_terminals[0]['state'] == 2
                    and recovery_terminals[0]['op_id'] == recovery_leaves[0]['op_id']
                    and any(e.get('task_state', {}).get('status') == 'done' for e in recovery)
                    and (await active_plans())['count'] == 0)
                if not result['recovery_success']:
                    raise AssertionError('Same-session follow-up did not succeed after cancellation')
            result['pass'] = True
        except Exception as error:
            result['error'] = repr(error)
            raise
        finally:
            # Abort outstanding work even if observation/assertions failed; detaching is not cancellation.
            if not result['pass']:
                try:
                    await collect(stub, session, '', {'abort_turn': True}, prefix+'-cleanup.jsonl', [], timeout=15)
                    await stopped(state, time.time())
                    result['active_after_cleanup'] = await active_plans()
                except Exception as cleanup_error:
                    result['cleanup_error'] = repr(cleanup_error)
            if running is not None and not running.done():
                running.cancel()
            if running is not None:
                await asyncio.gather(running, return_exceptions=True)
            if original_speed is not None:
                try:
                    result['restored_speed_percentage'] = await speed(stub, prefix+'-speed-restore', original_speed)
                except Exception as restore_error:
                    result['pass'] = False
                    result['speed_restore_error'] = repr(restore_error)
            save(prefix+'-summary.json', result)
            print(json.dumps(result, ensure_ascii=False), flush=True)
        if not result['pass']:
            raise AssertionError('Case failed; see summary')


async def main():
    """Run requested cases with a passive ROS observer and clean up that observer."""
    modes = sys.argv[1:] or ['complete', 'abort', 'cancel']
    if any(mode not in ('complete', 'abort', 'cancel') for mode in modes):
        raise SystemExit('Modes: complete abort cancel')
    if (OUT / 'navigation-ros.jsonl').exists():
        raise SystemExit('Use a fresh output directory; preserve the existing ROS observations')
    container = os.environ.get('ROBONIX_SIM_CONTAINER', 'robonix_tiago_sim')
    remote = '/tmp/rtdl-navigation-observer-'+str(uuid.uuid4())+'.py'
    copy = await asyncio.create_subprocess_exec('docker', 'cp', str(Path(__file__).with_name('ros_observer.py')), container+':'+remote)
    if await copy.wait():
        raise RuntimeError('Unable to copy passive observer to simulator')
    errors = (OUT / 'navigation-ros.stderr').open('w')
    process = await asyncio.create_subprocess_exec('docker', 'exec', container, 'bash', '-lc',
        'source /opt/ros/humble/setup.bash >/dev/null && exec python3 -u '+shlex.quote(remote),
        stdout=asyncio.subprocess.PIPE, stderr=errors, limit=16*1024*1024)
    state = {'samples': []}
    reader = asyncio.create_task(observe(process, state))
    try:
        for mode in modes:
            await case(mode, state)
    finally:
        killer = await asyncio.create_subprocess_exec('docker', 'exec', container, 'pkill', '-f', '^python3 -u '+remote)
        await killer.wait()
        await process.wait()
        await reader
        errors.close()


if __name__ == '__main__':
    asyncio.run(main())
