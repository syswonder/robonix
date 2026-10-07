# SPDX-License-Identifier: MulanPSL-2.0
"""Cross-process Pilot/Executor cancellation tests with a controlled local SSE source."""
import asyncio,json,os,pathlib,subprocess,sys,threading,time,uuid,socket
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
OUT=pathlib.Path(os.environ['RTDL_TEST_OUT']).resolve()
sys.path.insert(0,str(OUT/'proto'))
import grpc,pilot_pb2,executor_pb2
import robonix_contracts_pb2_grpc as rpc
from google.protobuf.json_format import MessageToDict
ROOT=pathlib.Path(__file__).resolve().parents[2]
requests={}; release=threading.Event(); processes=[]


class Handler(BaseHTTPRequestHandler):
    """Emit complete calls followed by deliberately incomplete or invalid RTDL."""
    def log_message(self,*args): pass
    def do_POST(self):
        """Keep a two-call sequence open until its first command really starts."""
        body=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        mode=next(m['content'].split('fault:')[-1].strip() for m in reversed(body['messages']) if isinstance(m.get('content'),str) and 'fault:' in m['content'])
        # The most recent user message is the test instruction.
        mode=mode.split()[0]
        requests[mode]=requests.get(mode,0)+1
        trace=OUT/f'fault-{mode}.trace'
        self.send_response(200);self.send_header('Content-Type','text/event-stream');self.end_headers()
        def send(text):
            """Write one model content delta without buffering the rest of the response."""
            self.wfile.write(('data: '+json.dumps({'choices':[{'delta':{'content':text}}]})+'\n\n').encode());self.wfile.flush()
        calls=[{'op':'do','op_id':0,'description':'long first step','cap':'executor.builtin_run_command','args':{'command':f'echo start >> {trace}; sleep 60; echo end >> {trace}'}},
               {'op':'do','op_id':0,'description':'pending second step','cap':'executor.builtin_run_command','args':{'command':f'echo second >> {trace}'}}]
        prefix='{"content":"","rtdl_description":"fault test","rtdl":{"op":"sequence","op_id":0,"description":"two steps","children":['+','.join(json.dumps(c) for c in calls)
        try:
            if mode == 'before':
                send('{\"content\":\"')
                (OUT/'before-started').write_text('started')
                release.wait(12)
                return
            send(prefix)
            for _ in range(150):
                if trace.exists():break
                if release.wait(.1):return
            if mode=='malformed':send(',!invalid')
            elif mode=='eof':return
            else:release.wait(12)
        except (BrokenPipeError,ConnectionResetError):pass
        self.close_connection=True


def start(name,args,env):
    """Launch one current Rust executable and retain its own log."""
    log=(OUT/f'fault-{name}.log').open('w')
    p=subprocess.Popen([str(ROOT/'target/debug'/f'robonix-{name}'),*args],env=env,stdout=log,stderr=subprocess.STDOUT)
    processes.append(p)


def wait_port(port):
    """Wait for local startup without consuming the gRPC service."""
    for _ in range(100):
        try:
            with socket.create_connection(('127.0.0.1',port),timeout=.1):return
        except OSError:time.sleep(.1)
    raise RuntimeError(f'port {port} not ready')


async def case(mode):
    """Check no retry/replay, canceled leaves, and removal from active plans."""
    release.clear();sid='fault-'+mode+'-'+str(uuid.uuid4());events=[];errors=[]
    async with grpc.aio.insecure_channel('127.0.0.1:52171',options=[('grpc.enable_http_proxy',0)]) as ch:
        stub=rpc.RobonixSystemPilotStub(ch)
        async def collect():
            """Retain prefix events even when SubmitTask terminates with an error."""
            try:
                task=pilot_pb2.Task(task_id=str(uuid.uuid4()),session_id=sid,text='fault:'+mode)
                async for e in stub.SubmitTask(task,timeout=30):events.append(MessageToDict(e,preserving_proto_field_name=True))
            except grpc.aio.AioRpcError as e:errors.append({'code':e.code().name,'detail':e.details()})
        running=asyncio.create_task(collect())
        if mode in ('abort', 'before'):
            for _ in range(150):
                if (OUT/('before-started' if mode == 'before' else f'fault-{mode}.trace')).exists():break
                if running.done():break
                await asyncio.sleep(.1)
            abort=pilot_pb2.Task(task_id=str(uuid.uuid4()),session_id=sid,context_json='{"abort_turn":true}')
            async for _ in stub.SubmitTask(abort,timeout=10):pass
        await running
    release.set()
    async with grpc.aio.insecure_channel('127.0.0.1:52161',options=[('grpc.enable_http_proxy',0)]) as ch:
        active=await rpc.RobonixSystemExecutorListActivePlansStub(ch).ListActivePlans(executor_pb2.ListActivePlans_Request(),timeout=5)
    trace=OUT/f'fault-{mode}.trace';lines=trace.read_text().splitlines() if trace.exists() else []
    plans=[e['plan'] for e in events if 'plan'in e]
    terminals=[e['node_state'] for e in events if e.get('node_state',{}).get('node_kind')==2 and e['node_state'].get('state') in (2,3,4,5)]
    result={'mode':mode,'requests':requests.get(mode,0),'trace':lines,'plans':len(plans),'terminal_states':[e['state'] for e in terminals],'errors':errors,'active':json.loads(active.plans_json),'events':events}
    result['pass']=(lines==['start'] and requests.get(mode)==1 and len(plans)==1 and len(terminals)==2 and all(e['state']==4 for e in terminals) and active.success and result['active']['count']==0 and bool(errors)==(mode not in ('abort', 'before')))
    if mode == 'before':
        result['pass'] = lines == [] and requests.get(mode) == 1 and len(plans) == len(terminals) == 0 and not errors and active.success and result['active']['count'] == 0
    (OUT/f'fault-{mode}.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='events'},ensure_ascii=False),flush=True)
    if not result['pass']:raise RuntimeError('fault check failed')
    await asyncio.sleep(.3)


async def main():
    """Exercise three generation failures and abort before/after the first node."""
    for mode in ['before','malformed','eof','idle','abort']:await case(mode)

if (OUT/'fault-malformed.json').exists():
    raise SystemExit('Use a fresh output directory')

server=ThreadingHTTPServer(('127.0.0.1',52181),Handler)
threading.Thread(target=server.serve_forever,daemon=True).start()
env=os.environ.copy()
for key in list(env):
    if key.lower().endswith('_proxy'):env.pop(key)
env.update({'ROBONIX_SOURCE_PATH':str(ROOT),'ROBONIX_WORKSPACE':str(OUT),'ROBONIX_PILOT_VLM_IDLE_TIMEOUT_SECS':'5','ROBONIX_VLM_UPSTREAM':'http://127.0.0.1:52181/v1','ROBONIX_VLM_API_KEY':'local-fixture','ROBONIX_VLM_MODEL':'local-fixture','SCRIBE_FILE_LEVEL':'debug','SCRIBE_LOG_DIR':str(OUT/'fault-scribe')})
try:
    start('atlas',['--listen','127.0.0.1:52151','--capabilities',str(ROOT/'capabilities')],env);wait_port(52151)
    start('executor',['--atlas','127.0.0.1:52151','--listen','127.0.0.1:52161'],env);wait_port(52161)
    start('pilot',['--atlas','127.0.0.1:52151','--listen','127.0.0.1:52171'],env);wait_port(52171)
    asyncio.run(main())
finally:
    release.set()
    for p in reversed(processes):
        p.terminate()
        try:p.wait(timeout=8)
        except subprocess.TimeoutExpired:p.kill();p.wait()
    server.shutdown()
