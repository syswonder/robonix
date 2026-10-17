# SPDX-License-Identifier: MulanPSL-2.0
"""Isolated Executor cancellation checks with handwritten inputs and a fault provider.

Uses existing navigation, camera and verifier contracts; does not move Webots.
Run with RTDL_TEST_OUT containing freshly generated protobuf clients, including atlas.
"""
import asyncio
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import uuid

OUT = Path(os.environ['RTDL_TEST_OUT']).resolve()
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(OUT/'proto'))
NAV = 'robonix/service/navigation/navigate'
CAM = 'robonix/primitive/camera/snapshot'
VERIFY = 'robonix/service/verifier/verify'


def write(name, value):
    """Replace a complete JSON observation atomically for cross-process readers."""
    temporary = OUT/(name+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2))
    temporary.replace(OUT/name)


def provider_main():
    """Serve existing contracts with controllable cancellation and verification outcomes."""
    from mcp.server.fastmcp import FastMCP
    from concurrent.futures import ThreadPoolExecutor
    import grpc
    import lifecycle_pb2
    import robonix_contracts_pb2_grpc as rpc

    class Driver(rpc.RobonixLifecycleDriverServicer):
        """Expose the existing lifecycle contract for the isolated test service."""
        def Driver(self, request, context):
            return lifecycle_pb2.Driver_Response(ok=True, state='ACTIVE')

    driver_server = grpc.server(ThreadPoolExecutor(max_workers=2))
    rpc.add_RobonixLifecycleDriverServicer_to_server(Driver(), driver_server)
    driver_server.add_insecure_port('127.0.0.1:53182')
    driver_server.start()
    mcp = FastMCP('cancel-fixture', host='127.0.0.1', port=53181, stateless_http=True, json_response=True)
    runs = {}

    def record(kind, **fields):
        with (OUT/'provider-events.jsonl').open('a') as log:
            log.write(json.dumps({'time': time.time(), 'kind': kind, **fields})+'\n')

    @mcp.tool()
    async def navigate(goal: dict) -> dict:
        """Start a simulated asynchronous run, retaining independent backend state."""
        config = json.loads((OUT/'provider-config.json').read_text())
        run_id = str(uuid.uuid4())
        runs[run_id] = dict(config, start=time.time(), state='RUNNING')
        write('provider-runs.json', runs)
        record('start', run_id=run_id, config=config)
        return {'accepted': True, 'run_id': run_id, 'detail': 'test fixture only'}

    @mcp.tool()
    async def status(run_id: str) -> dict:
        """Report backend state independently of Executor's cancellation flag."""
        run = runs[run_id]
        if run['state'] == 'RUNNING' and time.time()-run['start'] >= run.get('duration', 90):
            run['state'] = 'SUCCEEDED'
            write('provider-runs.json', runs)
        return {'known': True, 'state': run['state'], 'detail': 'fixture status'}

    @mcp.tool()
    async def cancel(run_id: str) -> dict:
        """Accept, reject, error, or delay cancellation while recording actual state."""
        run = runs[run_id]
        record('cancel', run_id=run_id, behavior=run.get('behavior', 'accept'))
        if run.get('behavior') == 'reject':
            return {'accepted': False, 'detail': 'injected cancellation rejection'}
        if run.get('behavior') == 'error':
            raise RuntimeError('injected cancellation transport/tool error')
        if run.get('behavior') == 'delay':
            await asyncio.sleep(10)
            return {'accepted': False, 'detail': 'injected slow cancellation rejection'}
        run['state'] = 'CANCELED'
        write('provider-runs.json', runs)
        return {'accepted': True, 'detail': 'fixture stopped'}

    @mcp.tool()
    async def snapshot() -> dict:
        return {'fixture': True}

    @mcp.tool()
    async def driver(command: int = 0) -> dict:
        return {'success': True}

    @mcp.tool()
    async def verify(call_id: str, args_json: str) -> dict:
        """Hold verification briefly so cancellation can be triggered inside VERIFYING."""
        record('verify', call_id=call_id)
        await asyncio.sleep(2)
        return {'passed': True, 'detail': 'fixture verification completed'}

    mcp.run(transport='streamable-http')


async def register_provider(atlas, contracts):
    import atlas_pb2 as ap
    await atlas.RegisterService(ap.RegisterRequest(id='fixture', namespace='robonix/service/navigation'))
    await atlas.DeclareCapability(ap.DeclareCapabilityRequest(provider_id='fixture',
        contract_id='robonix/lifecycle/driver', transport=1, endpoint='127.0.0.1:53182',
        params=ap.TransportParams(grpc=ap.GrpcParams(proto_file='robonix_contracts.proto',
            service_name='robonix.contracts.RobonixLifecycleDriver', method='/robonix.contracts.RobonixLifecycleDriver/Driver'))))
    for contract in contracts:
        await atlas.DeclareCapability(ap.DeclareCapabilityRequest(provider_id='fixture', contract_id=contract,
            transport=3, endpoint='http://127.0.0.1:53181/mcp',
            params=ap.TransportParams(mcp=ap.McpParams(input_schema_json='{"type":"object"}'))))
    await atlas.SetLifecycleState(ap.SetLifecycleStateRequest(id='fixture', state=2))
    await atlas.SetLifecycleState(ap.SetLifecycleStateRequest(id='fixture', state=3))


async def suite():
    """Run a boundary matrix against isolated real Atlas/Executor processes."""
    import grpc
    import atlas_pb2 as ap
    import atlas_pb2_grpc as ar
    import executor_pb2 as ex
    import pilot_pb2 as pi
    import robonix_contracts_pb2_grpc as rpc
    from google.protobuf.json_format import MessageToDict as dictionary
    options = [('grpc.enable_http_proxy', 0)]
    atlas_channel = grpc.aio.insecure_channel('127.0.0.1:53151', options=options)
    atlas = ar.AtlasStub(atlas_channel)
    await register_provider(atlas, [NAV, NAV+'/status', NAV+'/cancel', CAM, VERIFY])
    channel = grpc.aio.insecure_channel('127.0.0.1:53161', options=options)
    control_stub = rpc.RobonixSystemExecutorControlPlanStub(channel)
    active_stub = rpc.RobonixSystemExecutorListActivePlansStub(channel)
    stream_stub = rpc.RobonixSystemExecutorExecuteStreamStub(channel)
    results = []

    async def active():
        response = await active_stub.ListActivePlans(ex.ListActivePlans_Request(), timeout=5)
        assert response.success
        return json.loads(response.plans_json)

    async def control(pid, action='cancel', wait_ms=2000, timeout=4):
        """Record the public cancellation response, including client-visible timeout."""
        begin = time.time()
        try:
            response = await control_stub.ControlPlan(ex.ControlPlan_Request(action=action, plan_id=pid, wait_ms=wait_ms), timeout=timeout)
            return dict(dictionary(response, preserving_proto_field_name=True), elapsed=time.time()-begin)
        except grpc.aio.AioRpcError as error:
            return {'rpc_error': error.code().name, 'detail': error.details(), 'elapsed': time.time()-begin}

    async def launch(label, kind=0, contracts=(NAV, CAM), seal=True):
        """Send a handwritten root and leaves, retaining every Executor event."""
        pid = label+'-'+str(uuid.uuid4())
        events = []
        stream = stream_stub.ExecuteStream(timeout=25)

        async def read():
            async for event in stream:
                events.append(dictionary(event, preserving_proto_field_name=True))

        reader = asyncio.create_task(read())
        await stream.write(ex.ExecuteStreamInput(message_kind=0, plan_id=pid, session_id=label))
        await stream.write(ex.ExecuteStreamInput(message_kind=1, node_index=0,
            node=pi.RtdlNode(node_kind=kind, op_id='root', description=label)))
        for index, contract in enumerate(contracts, 1):
            args = {'goal': {}} if contract == NAV else {}
            await stream.write(ex.ExecuteStreamInput(message_kind=1, node_index=index, has_parent=True, parent_index=0,
                node=pi.RtdlNode(node_kind=2, op_id=str(index), description=contract,
                    call=pi.CapabilityCall(call_id=pid+':'+str(index), provider_id='fixture', contract_id=contract, args_json=json.dumps(args)))))
        if seal:
            await stream.write(ex.ExecuteStreamInput(message_kind=2, node_index=0))
            await stream.write(ex.ExecuteStreamInput(message_kind=3))
            await stream.done_writing()
        return pid, events, stream, reader

    async def until(predicate, timeout=8):
        """Require an observed condition before selecting a cancellation timing point."""
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            await asyncio.sleep(.05)
        raise TimeoutError('fixture condition not reached')

    def runs_for(name):
        if not (OUT/'provider-runs.json').exists():
            return {}
        return {k: v for k, v in json.loads((OUT/'provider-runs.json').read_text()).items() if v.get('case') == name}

    async def test(name, behavior='accept', kind=0, contracts=(NAV, CAM), phase='running', repetitions=1):
        """Compare cancellation replies, leaf terminals and independent backend state."""
        await atlas.Heartbeat(ap.HeartbeatRequest(id='fixture'))
        write('provider-config.json', {'case': name, 'behavior': behavior, 'duration': 0 if phase in ('finished', 'race') else 90})
        record = {'case': name, 'pass': False}
        pid, events, stream, reader = await launch(name, kind, () if phase == 'before' else contracts, seal=phase != 'before')
        try:
            if phase == 'before':
                await until(lambda: bool(events))
            elif phase == 'verifying':
                await until(lambda: any(e.get('node_state', {}).get('state') == 7 for e in events))
            else:
                await until(lambda: len(runs_for(name)) == contracts.count(NAV))
                if phase == 'finished':
                    await asyncio.wait_for(asyncio.shield(reader), 10)
                elif phase == 'race':
                    await asyncio.sleep([1.8, 1.95, 2.0, 2.05, 2.2][int(name.rsplit('_', 1)[1])])
            record['responses'] = await asyncio.gather(*(control(pid, wait_ms=100 if phase == 'verifying' else 2000)
                                                        for _ in range(repetitions)))
            if phase == 'before':
                await stream.done_writing()
            await asyncio.wait_for(asyncio.shield(reader), 12)
            record['events'] = events
            record['active'] = await active()
            record['backend_runs'] = runs_for(name)
            terminals = [e['node_state'] for e in events if e.get('node_state', {}).get('node_kind') == 2
                         and e['node_state'].get('state') in (2, 3, 4, 5)]
            ids = [(n['plan_id'], n['op_id']) for n in terminals]
            record['terminal_states'] = [n['state'] for n in terminals]
            record['backend_stopped'] = all(v['state'] in ('SUCCEEDED', 'CANCELED') for v in record['backend_runs'].values())
            healthy_rpc = all(r.get('success') and 'rpc_error' not in r for r in record['responses'])
            record['pass'] = (healthy_rpc and record['active']['count'] == 0 and record['backend_stopped']
                and len(terminals) == (0 if phase == 'before' else len(contracts)) and len(ids) == len(set(ids)))
            if phase == 'verifying':
                record['pass'] &= any(n['node_index'] == 1 and n['state'] == 2 for n in terminals)
            elif phase not in ('finished', 'race', 'before'):
                record['pass'] &= all(n['state'] == 4 for n in terminals)
        except Exception as error:
            record['error'] = repr(error)
            record['events'] = events
            record['backend_runs'] = runs_for(name)
        finally:
            if not reader.done():
                reader.cancel()
            await asyncio.gather(reader, return_exceptions=True)
            if behavior == 'delay':
                # Let the deliberately delayed server response finish before the next case.
                await asyncio.sleep(10)
            record['active_final'] = await active()
            write(name+'.json', record)
            results.append(record)
            print(json.dumps({k: v for k, v in record.items() if k not in ('events', 'backend_runs')}, ensure_ascii=False), flush=True)

    async def multiple(all_plans):
        """Check targeted cancellation preserves another plan and explicit cancel_all stops both."""
        name = 'cancel_all' if all_plans else 'targeted_isolation'
        write('provider-config.json', {'case': name, 'behavior': 'accept', 'duration': 90})
        first = await launch(name+'-A', contracts=(NAV,))
        second = await launch(name+'-B', contracts=(NAV,))
        record = {'case': name, 'pass': False}
        try:
            await until(lambda: len(runs_for(name)) == 2)
            response = await control(first[0], action='cancel_all' if all_plans else 'cancel')
            await asyncio.wait_for(asyncio.shield(first[3]), 10)
            if not all_plans:
                remaining = await active()
                record['other_preserved'] = (remaining['count'] == 1 and remaining['plans'][0]['plan_id'] == second[0]
                    and sum(r['state'] == 'RUNNING' for r in runs_for(name).values()) == 1)
                await control(second[0])
            await asyncio.wait_for(asyncio.shield(second[3]), 10)
            record.update(response=response, events=first[1]+second[1], active=await active(), backend_runs=runs_for(name))
            record['pass'] = (response.get('success', False) and record['active']['count'] == 0
                and all(r['state'] == 'CANCELED' for r in record['backend_runs'].values())
                and (all_plans or record['other_preserved']))
        except Exception as error:
            record['error'] = repr(error)
        finally:
            for plan in (first, second):
                if not plan[3].done():
                    await control(plan[0])
                    plan[3].cancel()
            await asyncio.gather(first[3], second[3], return_exceptions=True)
            results.append(record)
            write(name+'.json', record)
            print(json.dumps({k: v for k, v in record.items() if k not in ('events', 'backend_runs')}, ensure_ascii=False), flush=True)

    try:
        response = await control('does-not-exist')
        results.append({'case': 'unknown_plan', 'pass': response.get('success') and response.get('completed'), 'response': response})
        await test('before_first_node', phase='before')
        await test('parallel_four_leaves', kind=1, contracts=(NAV, NAV, NAV, NAV))
        await test('duplicate_cancel', repetitions=5)
        await test('already_finished', phase='finished', contracts=(NAV,))
        await test('during_verification', phase='verifying', contracts=(CAM, NAV))
        await multiple(False)
        await multiple(True)
        for index in range(5):
            await test('completion_race_'+str(index), phase='race', contracts=(NAV,))
        for index in range(5):
            await test('repeat_'+str(index))
        for behavior in ('reject', 'error', 'delay'):
            await test('provider_'+behavior, behavior=behavior)
        write('summary.json', {'cases': len(results), 'passed': sum(bool(r['pass']) for r in results),
                              'results': [{k: v for k, v in r.items() if k not in ('events', 'backend_runs')} for r in results]})
    finally:
        await channel.close()
        await atlas_channel.close()
    return all(record['pass'] for record in results)


def main():
    """Start isolated ports, run the checks and always terminate test processes."""
    if (OUT/'summary.json').exists():
        raise SystemExit('Use a fresh output directory')
    processes = []
    environment = os.environ.copy()
    for key in list(environment):
        if key.lower().endswith('_proxy'):
            environment.pop(key)
    environment.update(ROBONIX_SOURCE_PATH=str(ROOT), SCRIBE_LOG_DIR=str(OUT/'scribe'), SCRIBE_FILE_LEVEL='debug')

    def start(name, args, port):
        """Start a subprocess and require its listening socket before proceeding."""
        log = (OUT/(name+'.log')).open('w')
        process = subprocess.Popen(args, env=environment, stdout=log, stderr=subprocess.STDOUT)
        processes.append(process)
        for _ in range(100):
            if process.poll() is not None:
                raise RuntimeError(name+' exited during startup')
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=.1):
                    return
            except OSError:
                time.sleep(.1)
        raise TimeoutError(name+' startup')

    try:
        start('atlas', [str(ROOT/'target/debug/robonix-atlas'), '--listen', '127.0.0.1:53151', '--capabilities', str(ROOT/'capabilities')], 53151)
        start('provider', [sys.executable, __file__, 'provider'], 53181)
        config = {'verification': {'overlap': True, 'rules': [{'target_contract_id': CAM,
                  'target_provider_id': 'fixture', 'verifier_provider_id': 'fixture'}]}}
        start('executor', [str(ROOT/'target/debug/robonix-executor'), '--atlas', '127.0.0.1:53151',
                          '--listen', '127.0.0.1:53161', '--config-json', json.dumps(config)], 53161)
        passed = asyncio.run(suite())
    finally:
        for process in reversed(processes):
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    return passed


if __name__ == '__main__':
    if sys.argv[1:] == ['provider']:
        provider_main()
    else:
        raise SystemExit(0 if main() else 1)
