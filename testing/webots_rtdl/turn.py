# SPDX-License-Identifier: MulanPSL-2.0
"""Test model-planned Webots rotation via navigation, with independent TF evidence."""
import asyncio
import json
import math
import os
import pathlib
import shlex
import subprocess
import sys
import time
import uuid

OUT = pathlib.Path(os.environ['RTDL_TEST_OUT']).resolve()
sys.path.insert(0, str(OUT / 'proto'))
import grpc
from google.protobuf.json_format import MessageToDict
import pilot_pb2
import robonix_contracts_pb2_grpc as rpc


def pose():
    """Read a fresh map-to-base transform from the simulator's ROS graph."""
    code = '''
import json, time
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
rclpy.init()
node = rclpy.create_node('rtdl_turn_observer')
buffer = Buffer()
listener = TransformListener(buffer, node)
deadline = time.monotonic() + 20
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        t = buffer.lookup_transform('map', 'base_link', Time())
    except TransformException:
        continue
    p, q = t.transform.translation, t.transform.rotation
    print(json.dumps({'x': p.x, 'y': p.y, 'z': p.z,
                      'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}))
    break
else:
    raise SystemExit('No fresh map-to-base transform')
node.destroy_node()
rclpy.shutdown()
'''
    result = subprocess.run([
        'docker', 'exec', os.environ.get('ROBONIX_SIM_CONTAINER', 'robonix_tiago_sim'),
        'bash', '-lc', 'source /opt/ros/humble/setup.bash >/dev/null && python3 -c ' + shlex.quote(code),
    ], check=True, capture_output=True, text=True, timeout=30)
    return json.loads(result.stdout.splitlines()[-1])


def yaw(p):
    return math.atan2(2*(p['qw']*p['qz']+p['qx']*p['qy']), 1-2*(p['qy']**2+p['qz']**2))


async def main():
    """Submit camera/navigation/camera work and verify execution plus measured pose."""
    if (OUT / 'turn.jsonl').exists():
        raise SystemExit('Use a fresh RTDL_TEST_OUT; preserve previous evidence')
    before = pose()
    target = yaw(before) + math.pi/4
    goal = {'header': {'frame_id': 'map'}, 'pose': {
        'position': {'x': before['x'], 'y': before['y'], 'z': 0},
        'orientation': {'x': 0, 'y': 0, 'z': math.sin(target/2), 'w': math.cos(target/2)},
    }}
    prompt = ('在 Webots 执行一次转动测试。下面的 map 坐标目标来自刚读取的实时 TF，'
              '目标位置不变，目标朝向为当前朝向逆时针增加45度。请在同一个 sequence 中执行恰好三个 do：'
              '1. tiago_camera 相机快照；2. nav2 的 robonix/service/navigation/navigate；'
              '3. tiago_camera 相机快照。不要调用底盘 move/cmd，不要读文件或运行 shell，'
              '不要另外发起运动。导航失败则停止并报告；等待执行与验证结果再汇报。导航参数：'
              + json.dumps({'goal': goal}, ensure_ascii=False))
    (OUT / 'turn-input.json').write_text(json.dumps({'before': before, 'goal': goal, 'prompt': prompt}, ensure_ascii=False, indent=2))
    events = []
    async with grpc.aio.insecure_channel('127.0.0.1:50071', options=[('grpc.enable_http_proxy', 0), ('grpc.max_receive_message_length', 64*1024*1024)]) as channel:
        task = pilot_pb2.Task(task_id=str(uuid.uuid4()), session_id='turn-'+str(uuid.uuid4()), text=prompt, source=2)
        with (OUT / 'turn.jsonl').open('w') as log:
            async for event in rpc.RobonixSystemPilotStub(channel).SubmitTask(task, timeout=180):
                data = MessageToDict(event, preserving_proto_field_name=True)
                events.append(data)
                log.write(json.dumps({'time': time.time(), 'event': data}, ensure_ascii=False)+'\n')
                log.flush()
                if event.HasField('node_state'):
                    print('NODE', event.node_state.node_index, event.node_state.state, flush=True)
                if event.final_text:
                    print(event.final_text, flush=True)
    after = pose()
    plans = [e['plan'] for e in events if 'plan' in e]
    leaves = [(p['plan_id'], n) for p in plans for n in p.get('nodes', []) if n.get('node_kind') == 2]
    states = [e['node_state'] for e in events if e.get('node_state', {}).get('node_kind') == 2 and e['node_state'].get('state') in (2, 3, 4, 5)]
    contracts = [n['call']['contract_id'] for _, n in leaves]
    expected = ['robonix/primitive/camera/snapshot', 'robonix/service/navigation/navigate', 'robonix/primitive/camera/snapshot']
    angle_error = abs(math.atan2(math.sin(yaw(after)-target), math.cos(yaw(after)-target)))
    displacement = math.hypot(after['x']-before['x'], after['y']-before['y'])
    identities = {(p, n['op_id']) for p, n in leaves}
    terminal_ids = [(n['plan_id'], n['op_id']) for n in states]
    passed = (len(plans) == 1 and contracts == expected and len(states) == 3
              and set(terminal_ids) == identities and all(n['state'] == 2 for n in states)
              and angle_error < 0.35 and displacement < 0.25)
    result = {'pass': passed, 'before': before, 'after': after, 'yaw_change_deg': math.degrees(math.atan2(math.sin(yaw(after)-yaw(before)), math.cos(yaw(after)-yaw(before)))),
              'yaw_error_deg': math.degrees(angle_error), 'displacement_m': displacement,
              'contracts': contracts, 'terminal_states': [n['state'] for n in states]}
    (OUT / 'turn-summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return passed


if __name__ == '__main__':
    raise SystemExit(0 if asyncio.run(main()) else 1)
