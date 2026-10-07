# SPDX-License-Identifier: MulanPSL-2.0
"""Real-model Webots cancellation of parallel branches and isolation of other work."""
import asyncio
import json
import math
import os
from pathlib import Path
import re
import shlex
import sys
import time
import uuid

import grpc
import navigation as n


async def check(mode, state):
    """Generate plans through Pilot, cancel moving work, and verify scope and ROS stop."""
    prefix = mode+'-'+uuid.uuid4().hex[:8]
    session = prefix+'-A'
    side_session = prefix+'-B' if mode == 'sessions' else session
    trace = n.OUT/(prefix+'.trace')
    jobs, streams = [], []
    result = {'case': mode, 'prefix': prefix, 'pass': False}
    original_speed = None
    await n.wait_until(lambda: n.fresh(state) and 'map' in state, 30, 'No fresh ROS observation')
    before = state['latest']
    goal = n.goal_from_map(state['map'], before['pose'], .8)
    command = 'echo start >> '+shlex.quote(str(trace))+'; sleep 45; echo end >> '+shlex.quote(str(trace))
    move_text = ('在一个 sequence 中恰好两个 do：nav2.navigation_navigate 导航到目标，'
                 '然后 tiago_camera.camera_snapshot 拍照。不要其他动作，不重试。导航参数：'
                 + json.dumps({'goal': goal}, ensure_ascii=False))
    side_text = ('这是独立任务，保留已运行的导航，不要取消或替换已有计划。立即新建另一个 sequence，'
                 '恰好两个 do：executor.builtin_run_command 运行下面命令，然后 tiago_lidar.lidar_snapshot。'
                 '不要等导航完成，不修改命令，不执行其他动作，命令参数：'+json.dumps({'command': command}))
    if mode.startswith('parallel'):
        prompt = ('生成恰好一个 parallel，包含两个 sequence 分支，每分支恰好两个 do，总共4个do和3个operator。'
                  '分支一：'+move_text+' 分支二：先 executor.builtin_run_command 执行命令 '
                  + json.dumps({'command': command})+'，再 tiago_lidar.lidar_snapshot。'
                  '两个分支并行。不要读文件、增加其他动作或重试。取消时停止整个计划。')
    else:
        prompt = move_text
    n.save(prefix+'-input.json', {'session': session, 'side_session': side_session, 'prompt': prompt,
                                 'side_prompt': side_text, 'goal': goal, 'before': before})
    log_path = n.ROOT/'examples/webots/rbnx-boot/logs/executor.log'
    offset = log_path.stat().st_size
    async with grpc.aio.insecure_channel('127.0.0.1:50071', options=n.OPTIONS) as channel:
        stub = n.rpc.RobonixSystemPilotStub(channel)

        def submit(sid, text, context, suffix):
            events = []
            streams.append(events)
            job = asyncio.create_task(n.collect(stub, sid, text, context, prefix+suffix+'.jsonl', events, timeout=180))
            jobs.append(job)
            return job

        def plans():
            return {e['plan']['plan_id']: e['plan'] for stream in streams for e in stream if 'plan' in e}

        async def wait(predicate, timeout, message):
            """Surface a failed Pilot RPC immediately instead of masking it as a timeout."""
            def observed():
                for job in jobs:
                    if job.done() and not job.cancelled() and job.exception():
                        raise job.exception()
                return predicate()
            await n.wait_until(observed, timeout, message)

        try:
            if (await n.active_plans())['count']:
                raise RuntimeError('Other work is active')
            original_speed = await n.speed(stub, prefix+'-speed-before')
            if os.environ.get('RTDL_CANCEL_ALIGN') == '1':
                alignment = json.loads(json.dumps(goal))
                alignment['pose']['position'].update(x=before['pose']['x'], y=before['pose']['y'])
                align_events = []
                align_prompt = ('只发起一次 nav2.navigation_navigate，在当前位置原地调整到以下朝向。'
                                '允许用 scene.get_robot_context 或 mapping 的 map/get_pose 查询位姿；禁止其他运动，不重试导航。'
                                '完成验证后回复。导航参数：'+json.dumps({'goal': alignment}))
                n.save(prefix+'-alignment-input.json', {'prompt': align_prompt, 'goal': alignment})
                await n.collect(stub, prefix+'-align', align_prompt, {}, prefix+'-alignment.jsonl', align_events)
                align_leaves = [x for e in align_events if 'plan' in e for x in e['plan']['nodes'] if x.get('node_kind') == 2]
                align_finals = [e['node_state'] for e in align_events if e.get('node_state', {}).get('node_kind') == 2
                                and e['node_state'].get('state') in (2, 3, 4, 5)]
                align_nav = [x for x in align_leaves if x['call']['contract_id'] == n.NAV]
                result['alignment_calls'] = [x['call']['contract_id'] for x in align_leaves]
                assert len(align_nav) == 1 and all(x['call']['contract_id'] in
                    (n.NAV, 'robonix/system/scene/get_robot_context', 'robonix/service/map/get_pose') for x in align_leaves), 'Unexpected preparation action'
                assert len(align_leaves) == len(align_finals) and all(x['state'] == 2 for x in align_finals), 'Alignment failed'
                before = state['latest']
                target_yaw = 2*math.atan2(goal['pose']['orientation']['z'], goal['pose']['orientation']['w'])
                assert abs(math.atan2(math.sin(before['pose']['yaw']-target_yaw),
                                      math.cos(before['pose']['yaw']-target_yaw))) < .35, 'Alignment pose mismatch'
                result['aligned_before'] = before
            await n.speed(stub, prefix+'-speed-test', 100)
            submit(session, prompt, {}, '-task')
            await wait(lambda: bool(plans()), 60, 'No model-generated plan')
            first_id, first_plan = next(iter(plans().items()))
            first_leaves = [x for x in first_plan['nodes'] if x.get('node_kind') == 2]
            result.update(first_plan_id=first_id, first_total_nodes=len(first_plan['nodes']), first_do_nodes=len(first_leaves))
            if mode.startswith('parallel'):
                assert len(first_leaves) == 4 and len(first_plan['nodes']) == 7, 'Wrong parallel plan shape'
                assert sorted(x.get('node_kind', 0) for x in first_plan['nodes'] if x.get('node_kind') != 2) == [0, 0, 1]
            else:
                assert len(first_leaves) == 2
                submit(side_session, side_text, {'steer': True} if mode == 'plans' else {}, '-side')
                await wait(lambda: len(plans()) == 2, 60, 'Independent plan was not generated')
                result['side_plan_id'] = next(pid for pid in plans() if pid != first_id)
                result['side_total_nodes'] = len(plans()[result['side_plan_id']]['nodes'])
            await wait(lambda: trace.exists() and n.fresh(state)
                       and n.distance(state['latest']['odom'], before['odom']) > .1
                       and state['latest']['odom']['v'] > .03
                       and any(g['status'] == 2 for g in state['latest'].get('goals', [])),
                       60, 'Both navigation motion and independent command start were not observed')
            assert 'end' not in trace.read_text(), 'Side work ended before cancellation; isolation not tested'
            result['active_before_control'] = await n.active_plans()
            assert result['active_before_control']['count'] == (1 if mode.startswith('parallel') else 2)
            moving = state['latest']
            ids = [g['id'] for g in moving['goals'] if g['status'] == 2]
            requested = time.time()
            hard = mode in ('parallel_abort', 'sessions')
            text = (f'只取消正在运行的计划 {first_id}，停止该计划的所有动作和后续拍照。'
                    '保留其他独立计划继续运行，不要 cancel_all，不要重试导航，取消完成后回复。')
            n.save(prefix+'-control-input.json', {'time': requested, 'text': '' if hard else text,
                                               'context': {'abort_turn': True} if hard else {'steer': True}})
            await submit(session, '' if hard else text, {'abort_turn': True} if hard else {'steer': True}, '-control')
            await n.stopped(state, requested)
            await wait(lambda: all(any(g['id'] == gid and g['status'] == 5 for g in state['latest'].get('goals', [])) for gid in ids),
                       10, 'ROS action was not canceled')
            await asyncio.wait_for(asyncio.gather(*jobs), 100)
            if mode == 'plans':
                # A cancellation reply closes its subscriber while unrelated plans keep running.
                # Observe their completion without submitting another user instruction.
                deadline = time.monotonic()+70
                while (await n.active_plans())['count'] and time.monotonic() < deadline:
                    await asyncio.sleep(.2)
            result['active_after'] = await n.active_plans()
            result['trace'] = trace.read_text().splitlines()
            result['after'] = state['latest']
            with log_path.open('rb') as log:
                log.seek(offset)
                log_text = log.read().decode()
            (n.OUT/(prefix+'-executor.log')).write_text(log_text)
            dispatch = [line for line in log_text.splitlines() if 'dispatching call_id=' in line]
            result['plans'] = list(plans().values())
            terms = {(s['plan_id'], s['op_id']): s for stream in streams for e in stream
                     if (s := e.get('node_state', {})).get('node_kind') == 2 and s.get('state') in (2, 3, 4, 5)}
            result['terminals'] = [{k: v for k, v in s.items() if k != 'leaf_result'} for s in terms.values()]
            calls = [x['call'] for p in plans().values() for x in p['nodes'] if x.get('node_kind') == 2]
            counts = {c['call_id']: sum('call_id='+c['call_id']+' ' in line for line in dispatch) for c in calls}
            result['dispatch_counts'] = counts
            assert result['active_after']['count'] == 0
            assert len(plans()) == (1 if mode.startswith('parallel') else 2), 'Extra plan generated'
            for call in calls:
                expected = 0 if call['contract_id'] == n.CAMERA else 1
                if mode.startswith('parallel') and call['contract_id'].endswith('/lidar/snapshot'):
                    expected = 0
                assert counts[call['call_id']] == expected, 'Unexpected or missing dispatch'
            if mode.startswith('parallel'):
                assert result['trace'] == ['start'], 'Canceled branch completed or replayed'
                if not hard:
                    assert len(terms) == 4 and all(s['state'] == 4 for s in terms.values())
            else:
                assert result['trace'] == ['start', 'end'], 'Unrelated command was interrupted'
                side_leaves = [x for x in plans()[result['side_plan_id']]['nodes'] if x.get('node_kind') == 2]
                assert len(side_leaves) == 2
                if mode == 'sessions':
                    assert all(terms[(result['side_plan_id'], x['op_id'])]['state'] == 2 for x in side_leaves)
                else:
                    pilot_log = (n.ROOT/'examples/webots/rbnx-boot/logs/pilot.log').read_text()
                    result['side_success_log'] = [line for line in pilot_log.splitlines()
                        if re.search(r'plan_id='+re.escape(result['side_plan_id'])+r' node=\d+ op_id=\S+ state=Succeeded', line)]
                    assert all(any('op_id='+x['op_id']+' state=Succeeded' in line
                                   for line in result['side_success_log']) for x in side_leaves)
            result['pass'] = True
        except Exception as error:
            result['error'] = repr(error)
        finally:
            if not result['pass']:
                for sid in set([session, side_session]):
                    try:
                        await n.collect(stub, sid, '', {'abort_turn': True}, prefix+'-cleanup-'+sid[-1]+'.jsonl', [], timeout=15)
                    except Exception as error:
                        result.setdefault('cleanup_errors', []).append(repr(error))
                await n.stopped(state, time.time())
            for job in jobs:
                if not job.done():
                    job.cancel()
            await asyncio.gather(*jobs, return_exceptions=True)
            if original_speed is not None:
                try:
                    result['restored_speed'] = await n.speed(stub, prefix+'-speed-restore', original_speed)
                except Exception as error:
                    result['pass'] = False
                    result['restore_error'] = repr(error)
            n.save(prefix+'-summary.json', result)
            print(json.dumps({k: v for k, v in result.items() if k not in ('plans', 'after', 'terminals')}, ensure_ascii=False), flush=True)
    return result


async def main():
    """Run selected real-model scenarios while an independent ROS observer records motion."""
    modes = sys.argv[1:] or ['parallel_abort', 'parallel_cancel', 'sessions', 'plans']
    assert all(m in ('parallel_abort', 'parallel_cancel', 'sessions', 'plans') for m in modes)
    if (n.OUT/'navigation-ros.jsonl').exists():
        raise SystemExit('Use a fresh output directory')
    container = os.environ.get('ROBONIX_SIM_CONTAINER', 'robonix_tiago_sim')
    remote = '/tmp/rtdl-navigation-observer-'+uuid.uuid4().hex+'.py'
    copy = await asyncio.create_subprocess_exec('docker', 'cp', str(Path(__file__).with_name('ros_observer.py')), container+':'+remote)
    assert await copy.wait() == 0
    errors = (n.OUT/'observer.stderr').open('w')
    process = await asyncio.create_subprocess_exec('docker', 'exec', container, 'bash', '-lc',
        'source /opt/ros/humble/setup.bash >/dev/null && exec python3 -u '+shlex.quote(remote),
        stdout=asyncio.subprocess.PIPE, stderr=errors, limit=16*1024*1024)
    state = {'samples': []}
    reader = asyncio.create_task(n.observe(process, state))
    results = []
    try:
        for mode in modes:
            result = await check(mode, state)
            results.append(result)
            if (await n.active_plans())['count']:
                break
    finally:
        killer = await asyncio.create_subprocess_exec('docker', 'exec', container, 'pkill', '-f', '^python3 -u '+remote)
        await killer.wait()
        await process.wait()
        await reader
        errors.close()
        n.save('summary.json', {'cases': len(results), 'passed': sum(r['pass'] for r in results),
                               'results': [{k: v for k, v in r.items() if k not in ('plans', 'after', 'terminals')} for r in results]})
    return len(results) == len(modes) and all(result['pass'] for result in results)


if __name__ == '__main__':
    raise SystemExit(0 if asyncio.run(main()) else 1)
