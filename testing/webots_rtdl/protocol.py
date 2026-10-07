# SPDX-License-Identifier: MulanPSL-2.0
"""Validate ExecuteStream boundaries against real Webots sensor capabilities."""
import asyncio
import json
import os
import pathlib
import sys
import time
import uuid
sys.path.insert(0,str(pathlib.Path(os.environ['RTDL_TEST_OUT']) / 'proto'))
import grpc
import executor_pb2 as ex
import pilot_pb2 as pi
import robonix_contracts_pb2_grpc as rpc
from google.protobuf.json_format import MessageToDict
OUT = pathlib.Path(os.environ['RTDL_TEST_OUT']).resolve()


def leaf(index,parent,provider='tiago_lidar',contract='robonix/primitive/lidar/snapshot'):
    """Construct one complete sensor call as a streamed child."""
    return ex.ExecuteStreamInput(message_kind=1,node_index=index,has_parent=True,parent_index=parent,
        node=pi.RtdlNode(node_kind=2,op_id=f'op-{index}',description='实时传感器采样',call=pi.CapabilityCall(call_id=f'call-{index}',provider_id=provider,contract_id=contract,args_json='{}')))


async def case(ch,mode):
    """Withhold END, check early leaf completion, then seal or truncate input."""
    pid='protocol-'+mode+'-'+str(uuid.uuid4())
    stream=rpc.RobonixSystemExecutorExecuteStreamStub(ch).ExecuteStream(timeout=30)
    events=[]; reached=asyncio.Event(); errors=[]
    async def collect():
        """Record events and identify real successful work before END."""
        try:
            async for e in stream:
                events.append({'t':time.time(),'event':MessageToDict(e,preserving_proto_field_name=True)})
                if e.HasField('node_state') and e.node_state.node_kind==2 and e.node_state.state==2: reached.set()
        except grpc.aio.AioRpcError as e: errors.append(e.code().name)
    reader=asyncio.create_task(collect())
    await stream.write(ex.ExecuteStreamInput(message_kind=0,plan_id=pid,session_id='protocol'))
    await stream.write(ex.ExecuteStreamInput(message_kind=1,node_index=0,node=pi.RtdlNode(node_kind=0,op_id='root',description='测试顺序')))
    await stream.write(leaf(1,0))
    await asyncio.wait_for(reached.wait(),10)
    early=not any('plan_complete' in x['event'] for x in events)
    if mode=='nested':
        await stream.write(ex.ExecuteStreamInput(message_kind=1,node_index=2,has_parent=True,parent_index=0,node=pi.RtdlNode(node_kind=1,op_id='parallel',description='并行采样')))
        await stream.write(leaf(3,2))
        await stream.write(leaf(4,2,'tiago_camera','robonix/primitive/camera/snapshot'))
        await stream.write(ex.ExecuteStreamInput(message_kind=2,node_index=2))
        await stream.write(ex.ExecuteStreamInput(message_kind=2,node_index=0))
        await stream.write(ex.ExecuteStreamInput(message_kind=3))
    elif mode=='invalid_parent':
        await stream.write(leaf(2,99))
    await stream.done_writing()
    await reader
    await asyncio.sleep(.2)
    active=await rpc.RobonixSystemExecutorListActivePlansStub(ch).ListActivePlans(ex.ListActivePlans_Request(),timeout=5)
    cleaned=active.success and pid not in active.plans_json
    finals=[e['event']['node_state'] for e in events if 'node_state' in e['event'] and e['event']['node_state'].get('node_kind')==2 and e['event']['node_state'].get('state') in (2,3,4,5)]
    result={'mode':mode,'early_success_before_END':early,'cleaned':cleaned,'errors':errors,'terminals':len(finals),'events':events}
    if mode=='nested': result['pass']=early and cleaned and not errors and len(finals)==3 and all(e['state']==2 for e in finals) and any('plan_complete' in e['event'] and not e['event']['plan_complete'].get('any_failed',False) for e in events)
    else: result['pass']=early and cleaned and errors==['INVALID_ARGUMENT'] and len(finals)==1 and finals[0]['state']==2
    (OUT/f'protocol-{mode}.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='events'}),flush=True)
    if not result['pass']:
        raise SystemExit(1)


async def main():
    """Run protocol checks sequentially without model-generated plans."""
    async with grpc.aio.insecure_channel('127.0.0.1:50061',options=[('grpc.enable_http_proxy',0),('grpc.max_receive_message_length',64*1024*1024)]) as ch:
        for mode in ['nested','eof','invalid_parent']: await case(ch,mode)
        active=await rpc.RobonixSystemExecutorListActivePlansStub(ch).ListActivePlans(ex.ListActivePlans_Request(),timeout=5)
        (OUT/'active-plans-final.json').write_text(active.plans_json)
        if not active.success or json.loads(active.plans_json)['count']!=0:
            raise SystemExit('Active plans remain; inspect and clean up before retrying')

asyncio.run(main())
