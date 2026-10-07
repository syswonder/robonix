# SPDX-License-Identifier: MulanPSL-2.0
"""Interrupt the current real model after dispatch starts but before its plan is published."""
import asyncio
import json
import shlex
import time
import uuid

import grpc
import navigation as n


async def case(index):
    """Require real command execution during generation and prove its cancellation."""
    prefix = 'generation-live-'+str(index)
    trace = n.OUT/(prefix+'.trace')
    session = prefix+'-'+uuid.uuid4().hex
    command = 'echo start >> '+shlex.quote(str(trace))+'; sleep 90; echo end >> '+shlex.quote(str(trace))
    prompt = ('在同一个 sequence 中恰好生成10个do：第1个用executor.builtin_run_command原样执行 '
              + json.dumps({'command': command})+'；第2至9个各调用一次tiago_lidar.lidar_snapshot；'
              '第10个调用tiago_camera.camera_snapshot。不要合并、省略、重试或执行其他动作。'
              '每个节点用一句中文描述意图。全部成功后回复完成；取消后不要继续。')
    events = []
    result = {'case': prefix, 'pass': False}
    n.save(prefix+'-input.json', {'session': session, 'prompt': prompt})
    path = n.ROOT/'examples/webots/rbnx-boot/logs/executor.log'
    offset = path.stat().st_size
    async with grpc.aio.insecure_channel('127.0.0.1:50071', options=n.OPTIONS) as channel:
        stub = n.rpc.RobonixSystemPilotStub(channel)
        job = asyncio.create_task(n.collect(stub, session, prompt, {}, prefix+'.jsonl', events, timeout=90))
        try:
            def started():
                if job.done() and job.exception():
                    raise job.exception()
                return trace.exists()
            await n.wait_until(started, 60, 'First generated command did not start')
            result['plan_published_before_abort'] = any('plan' in e for e in events)
            result['abort_at'] = time.time()
            await n.collect(stub, session, '', {'abort_turn': True}, prefix+'-abort.jsonl', [], timeout=15)
            await asyncio.wait_for(asyncio.shield(job), 15)
            await asyncio.sleep(.5)
            plans = [e['plan'] for e in events if 'plan' in e]
            leaves = [x for p in plans for x in p['nodes'] if x.get('node_kind') == 2]
            terminals = [e['node_state'] for e in events if e.get('node_state', {}).get('node_kind') == 2
                         and e['node_state'].get('state') in (2, 3, 4, 5)]
            result.update(plans=plans, trace=trace.read_text().splitlines(), active=await n.active_plans(),
                          submitted_do=len(leaves), terminal_states=[t['state'] for t in terminals])
            with path.open('rb') as log:
                log.seek(offset)
                text = log.read().decode()
            (n.OUT/(prefix+'-executor.log')).write_text(text)
            dispatch = [line for line in text.splitlines() if 'dispatching call_id=' in line]
            expected = {(p['plan_id'], x['op_id']) for p in plans for x in p['nodes'] if x.get('node_kind') == 2}
            actual = [(t['plan_id'], t['op_id']) for t in terminals]
            result['pass'] = (not result['plan_published_before_abort'] and len(plans) == 1
                and result['trace'] == ['start'] and result['active']['count'] == 0
                and bool(leaves) and set(actual) == expected and len(actual) == len(set(actual))
                and all(t['state'] == 4 for t in terminals)
                and sum('call_id='+leaves[0]['call']['call_id']+' ' in line for line in dispatch) == 1
                and not any("provider='tiago_lidar'" in line or "provider='tiago_camera'" in line for line in dispatch))
        except Exception as error:
            result['error'] = repr(error)
        finally:
            if not result['pass']:
                try:
                    await n.collect(stub, session, '', {'abort_turn': True}, prefix+'-cleanup.jsonl', [], timeout=15)
                except Exception as error:
                    result['cleanup_error'] = repr(error)
            if not job.done():
                job.cancel()
            await asyncio.gather(job, return_exceptions=True)
            n.save(prefix+'-summary.json', result)
            print(json.dumps({k: v for k, v in result.items() if k != 'plans'}, ensure_ascii=False), flush=True)
    return result


async def main():
    """Repeat with fresh sessions and retain failures instead of retrying them away."""
    if (n.OUT/'generation-live-0-input.json').exists():
        raise SystemExit('Use a fresh output directory')
    results = [await case(i) for i in range(3)]
    n.save('summary.json', {'cases': len(results), 'passed': sum(r['pass'] for r in results), 'results': results})
    return all(r['pass'] for r in results)


if __name__ == '__main__':
    raise SystemExit(0 if asyncio.run(main()) else 1)
