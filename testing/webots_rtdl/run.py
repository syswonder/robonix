# SPDX-License-Identifier: MulanPSL-2.0
"""Exercise live Pilot with Webots capabilities and retain full timed events."""
import asyncio
import json
import os
import pathlib
import sys
import time
import uuid
sys.path.insert(0, str(pathlib.Path(os.environ['RTDL_TEST_OUT']) / 'proto'))
import grpc
from google.protobuf.json_format import MessageToDict
import pilot_pb2
import robonix_contracts_pb2_grpc as rpc
OUT = pathlib.Path(os.environ['RTDL_TEST_OUT']).resolve()


def prompt(n):
    """Specify observable robot work with exactly n desired capability calls."""
    if n == 0:
        return '这是一项零动作测试：不要调用任何 capability，不要读取文档或记忆，不需要观察环境。只回答“零动作测试完成”，并将当前任务标记为 done。'
    steps = ['读取一次当前激光快照（tiago_lidar 的 lidar/snapshot，参数 {}）'] * n
    if n >= 2:
        steps[-1] = '读取一次当前相机快照（tiago_camera 的 camera/snapshot，参数 {}）'
    return (f'在 Webots 中执行以下严格计数的 RTDL 测试：在同一个 sequence 中恰好执行 {n} 个 do，按顺序逐项执行，不合并、不省略重复采样。'
            '这些是独立的实时采样，不能复用上次结果。只使用已提供目录中的对应 capability，不读文档、不执行 shell、不使用记忆工具，也不要额外采样。'
            '失败时停止并报告；全部返回成功后根据结果简短汇报并标记 done，不再执行额外动作。任务步骤：\n'
            + '\n'.join(f'{i+1}. {s}' for i,s in enumerate(steps)))


async def run(n):
    """Capture a fresh conversation and check node counts and terminal identities."""
    tag=f'count-{n:02d}'
    sid=f'webots-stream-{tag}-{uuid.uuid4()}'
    task=pilot_pb2.Task(task_id=str(uuid.uuid4()), session_id=sid, text=prompt(n), source=2,
                       timestamp_ms=int(time.time()*1000))
    (OUT/f'{tag}.prompt.txt').write_text(task.text)
    events=[]
    start=time.time()
    error=None
    async with grpc.aio.insecure_channel('127.0.0.1:50071',options=[('grpc.max_receive_message_length',64*1024*1024),('grpc.enable_http_proxy',0)]) as ch:
        stub=rpc.RobonixSystemPilotStub(ch)
        try:
            with (OUT/f'{tag}.jsonl').open('w') as f:
                async for e in stub.SubmitTask(task,timeout=240):
                    d=MessageToDict(e,preserving_proto_field_name=True)
                    events.append(d)
                    f.write(json.dumps({'time':time.time(),'elapsed':time.time()-start,'event':d},ensure_ascii=False)+'\n'); f.flush()
                    if e.HasField('plan'):
                        print(tag,'PLAN',e.plan.plan_id,'nodes',len(e.plan.nodes),'do',sum(x.node_kind==2 for x in e.plan.nodes),flush=True)
                    if e.HasField('node_state') and e.node_state.node_kind==2:
                        print(tag,'NODE',e.node_state.node_index,e.node_state.state,flush=True)
                    if e.final_text: print(tag,'FINAL',e.final_text[:300],flush=True)
        except Exception as exc:
            error=str(exc)
    plans=[e['plan'] for e in events if 'plan' in e]
    leaves=[x for p in plans for x in p.get('nodes',[]) if x.get('node_kind')==2]
    terminals=[e['node_state'] for e in events if 'node_state' in e and e['node_state'].get('node_kind')==2 and e['node_state'].get('state') in [2,3,4,5]]
    done=any(e.get('task_state',{}).get('status')=='done' for e in events)
    result={'n':n,'session_id':sid,'start':start,'end':time.time(),'error':error,'plans':len(plans),'do_count':len(leaves),'terminal_count':len(terminals),'states':[x.get('state') for x in terminals],'done':done,'final':next((e['final_text'] for e in reversed(events) if e.get('final_text')),'')}
    expected={(p['plan_id'],x['op_id']) for p in plans for x in p.get('nodes',[]) if x.get('node_kind')==2}
    actual=[(x['plan_id'],x['op_id']) for x in terminals]
    result['terminal_ids_match']=set(actual)==expected and len(actual)==len(set(actual))
    result['pass']=result['terminal_ids_match'] and len(plans)==int(n>0) and error is None and len(leaves)==n and len(terminals)==n and all(x.get('state')==2 for x in terminals) and done
    (OUT/f'{tag}.summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print('SUMMARY',json.dumps(result,ensure_ascii=False),flush=True)
    return result


async def main():
    """Run requested counts sequentially and halt on runtime failure."""
    counts=list(map(int,sys.argv[1:] or range(11)))
    if any(n<0 or n>10 for n in counts):
        raise SystemExit('Counts must be within 0..10')
    for n in counts:
        if (OUT/f'count-{n:02d}.jsonl').exists():
            raise SystemExit('Use a fresh RTDL_TEST_OUT; existing evidence must not be overwritten')
        r=await run(n)
        if not r['pass']:
            raise SystemExit(1)

asyncio.run(main())
