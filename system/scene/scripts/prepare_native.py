#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Reuse or prepare a native Scene environment, verified before publication."""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import subprocess
import sys
import urllib.request
import uuid

PKG = Path(__file__).resolve().parents[1]
BUILD = PKG / 'rbnx-build'
sys.path.insert(0, str(PKG))
from scene_service.inference_config import inference_backend, validate_tensorrt_config  # noqa: E402

TORCH_CHECK = '''
import json, os, torch, torchvision
cpu = os.environ.get('SCENE_CG_FORCE_CPU', '').strip().lower() in ('1', 'true', 'yes')
device = 'cpu' if cpu else 'cuda'
assert tuple(map(int, torch.__version__.split('+')[0].split('.')[:2])) >= (2, 5)
assert cpu or torch.cuda.is_available(), 'Torch cannot use CUDA'
x = torch.ones((1, 3, 16, 16), device=device)
y = torch.nn.functional.conv2d(x, torch.ones((4, 3, 3, 3), device=device))
assert y.isfinite().all().item()
boxes = torch.tensor([[0.,0.,8.,8.],[1.,1.,9.,9.]], device=device)
assert torchvision.ops.nms(boxes, torch.tensor([.9,.8], device=device), .5).tolist() == [0]
if not cpu: torch.cuda.synchronize()
print(json.dumps({'torch': torch.__version__, 'torchvision': torchvision.__version__,
                  'cuda': torch.version.cuda, 'gpu': None if cpu else torch.cuda.get_device_name()}))
'''
GPU_CHECK = """
import ctypes as c, json
lib = c.CDLL('libcuda.so.1')
assert lib.cuInit(0) == 0, 'CUDA driver cannot initialize'
version, device, major, minor = (c.c_int() for _ in range(4))
assert lib.cuDriverGetVersion(c.byref(version)) == 0
assert lib.cuDeviceGet(c.byref(device), 0) == 0, 'No CUDA device'
assert lib.cuDeviceComputeCapability(c.byref(major), c.byref(minor), device) == 0
print(json.dumps({'driver_cuda': version.value, 'sm': f'{major.value}{minor.value}'}))
"""
CUDART_CHECK = """
import ctypes as c, json
for major in dict.fromkeys((PREFERRED_MAJOR, 13, 12)):
    if major < 12:
        continue
    try:
        lib = c.CDLL(f'libcudart.so.{major}')
        version, count = c.c_int(), c.c_int()
        if lib.cudaRuntimeGetVersion(c.byref(version)) == 0 and lib.cudaGetDeviceCount(c.byref(count)) == 0 and count.value:
            print(json.dumps({'cuda_runtime': version.value}))
            break
    except OSError:
        pass
else:
    raise RuntimeError('No working CUDA runtime for TensorRT Python bindings')
"""
CUDA_BINDINGS_CHECK = """
import json
from cuda.bindings import runtime as rt
error, count = rt.cudaGetDeviceCount()
assert int(error) == 0 and count, f'CUDA bindings cannot use the driver: {error}'
error, pointer = rt.cudaMalloc(16)
assert int(error) == 0, f'CUDA bindings allocation failed: {error}'
error, = rt.cudaFree(pointer)
assert int(error) == 0, f'CUDA bindings release failed: {error}'
print(json.dumps({'cuda_bindings': 'ok'}))
"""
TRT_CHECK = '''
import json, tensorrt as trt
assert trt.__version__.split('.')[0] == '10', 'Scene requires TensorRT 10'
assert trt.Builder(trt.Logger(trt.Logger.ERROR)), 'TensorRT cannot use CUDA'
print(json.dumps({'tensorrt': trt.__version__}))
'''
STORAGE_CHECK = f'''
import json, sys
sys.path.insert(0, {str(PKG)!r})
from scene_service.persistence import verify_local_store
verify_local_store()
print(json.dumps({{"storage": "ok"}}))
'''
ROS_CHECK = ('import rclpy, tf2_ros; from sensor_msgs.msg import Image; '
             'from nav_msgs.msg import OccupancyGrid; print("ROS Python imports OK")')
ROS_BUILD_CHECK = '''
import colcon_core, colcon_cmake, colcon_ros, colcon_parallel_executor
import rosidl_adapter, json
from ament_index_python.packages import get_package_share_directory
get_package_share_directory('rosidl_default_generators')
print(json.dumps({'ros_build_tools': 'ok'}))
'''


def detect() -> dict:
    release = Path('/etc/nv_tegra_release').read_text()
    match = re.search(r'# R(\d+).*?REVISION:\s*([\d.]+)', release)
    if match is None:
        raise RuntimeError('Cannot read Jetson L4T version from /etc/nv_tegra_release')
    system = platform.freedesktop_os_release()
    model = Path('/proc/device-tree/model').read_text().rstrip('\0\n')
    return dict(l4t=f'{match[1]}.{match[2]}', distro=system['ID'], ubuntu=system['VERSION_ID'],
                arch=platform.machine(), python=f'{sys.version_info.major}.{sys.version_info.minor}',
                model=model)


def select_profile(host: dict, *, check_python: bool = True) -> dict:
    """Deprecated: fixed environment selection belongs to Docker preparation."""
    from scripts.prepare_docker import select_profile as docker_profile
    return docker_profile(host, check_python=check_python)


def run(command: list[str], **kwargs) -> subprocess.CompletedProcess:
    if not kwargs.get('capture_output'):
        print('[scene/native] ' + shlex.join(map(str, command)), flush=True)
    return subprocess.run(command, check=True, **kwargs)


def probe(python: str, code: str, ros: str = '') -> subprocess.CompletedProcess:
    command = [python, '-c', code]
    if ros:
        command = ['bash', '-c', 'set -e; source "$1"; exec "$2" -c "$3"',
                   'scene-native-probe', f'/opt/ros/{ros}/setup.bash', python, code]
    return subprocess.run(command, capture_output=True, text=True, timeout=120)


def require_probe(python: str, code: str, ros: str = '') -> dict:
    result = probe(python, code, ros)
    if result.returncode:
        raise RuntimeError(f'Native environment self-test failed:\n{result.stdout}\n{result.stderr}')
    if code == ROS_CHECK:
        return {}
    return json.loads(result.stdout.strip().splitlines()[-1])


def apt_install(packages: list[str]) -> None:
    privilege = [] if os.geteuid() == 0 else ['sudo', '-n']
    run(privilege + ['apt-get', 'update'])
    command = ['apt-get', 'install', '-y', '--no-install-recommends', *packages]
    plan = run(privilege + command + ['--simulate'], capture_output=True, text=True).stdout
    if any(line.startswith('Remv ') or
           (line.startswith('Inst ') and line.split()[1].startswith(('nvidia-l4t-', 'nvidia-jetpack', 'nvidia-driver-',
               'cuda-', 'libcud', 'libcublas', 'libcufft', 'libcurand', 'libcusolver', 'libcusparse', 'libnvjitlink')))
           for line in plan.splitlines()):
        raise RuntimeError('Dependency installation would change CUDA/driver/BSP packages or remove packages; '
                           'prepare the matching JetPack runtime before retrying')
    run(privilege + command)


def download_verified(source: dict, destination: Path) -> None:
    with urllib.request.urlopen(source['url'], timeout=30) as response, destination.open('wb') as output:
        shutil.copyfileobj(response, output)
    if hashlib.sha256(destination.read_bytes()).hexdigest() != source['sha256']:
        raise RuntimeError(f"Checksum mismatch for {source['url']}")


def install_ros() -> str:
    requested = os.environ.get('ROS_DISTRO', '')
    if requested and not re.fullmatch('[a-z][a-z0-9]*', requested):
        raise RuntimeError('Invalid ROS_DISTRO')
    installed = [p.parent.name for p in Path('/opt/ros').glob('*/setup.bash')]
    choices = [requested] if requested else installed
    for ros in choices:
        if probe(sys.executable, ROS_CHECK, ros).returncode == 0:
            return ros
    ros = requested or {'22.04': 'humble', '24.04': 'jazzy'}.get(
        platform.freedesktop_os_release().get('VERSION_ID'))
    if not ros:
        raise RuntimeError('No working ROS installation; install ROS for this OS and set ROS_DISTRO')
    if not re.fullmatch('[a-z][a-z0-9]*', ros):
        raise RuntimeError('Invalid ROS_DISTRO')
    # Use configured OS repositories; native preparation does not add SDK sources.
    apt_install([f'ros-{ros}-{name}' for name in ('ros-base', 'tf2-ros', 'rmw-zenoh-cpp')])
    require_probe(sys.executable, ROS_CHECK, ros)
    return ros


def prepare_ros_overlay(ros: str) -> None:
    workspace = BUILD / 'codegen/ros2_idl'
    if not (workspace / 'src/map').is_dir():
        print('[scene/native] WARNING: map interface sources missing; skipping ROS overlay build', flush=True)
        return
    if probe(sys.executable, ROS_BUILD_CHECK, ros).returncode:
        apt_install(['python3-colcon-common-extensions', f'ros-{ros}-rosidl-default-generators'])
        require_probe(sys.executable, ROS_BUILD_CHECK, ros)
    command = [sys.executable, '-c', 'from colcon_core.command import main; raise SystemExit(main())',
               'build', '--packages-up-to', 'map', '--parallel-workers', '2', '--cmake-args',
               f'-DPython3_EXECUTABLE={sys.executable}', f'-DPYTHON_EXECUTABLE={sys.executable}']
    run(['bash', '-c', 'set -e; source "$1"; shift; exec "$@"',
         'scene-native-overlay', f'/opt/ros/{ros}/setup.bash', *command], cwd=workspace)
    check = ('from map.msg import MapLifecycle; '
             'MapLifecycle.__class__.__import_type_support__(); '
             'assert MapLifecycle.__class__._TYPE_SUPPORT is not None')
    run(['bash', '-c', 'set -e; source "$1"; source "$2"; exec "$3" -c "$4"',
         'scene-native-overlay-check', f'/opt/ros/{ros}/setup.bash',
         str(workspace / 'install/setup.bash'), sys.executable, check])


def environment_key(host: dict, base_python: str = None) -> str:
    digest = hashlib.sha256(json.dumps([host, inference_backend()], sort_keys=True).encode())
    for path in [Path(__file__), *sorted((PKG / 'docker/requirements').glob('scene-*.txt'))]:
        digest.update(path.read_bytes())
    digest.update(str(Path(base_python or sys.executable).absolute()).encode())
    return digest.hexdigest()


def ensure_torch(python: str) -> dict:
    existing = probe(python, TORCH_CHECK)
    if existing.returncode == 0:
        print('[scene/native] reusing validated Torch/torchvision', flush=True)
        return json.loads(existing.stdout.strip().splitlines()[-1])
    cpu = os.environ.get('SCENE_CG_FORCE_CPU', '').strip().lower() in ('1', 'true', 'yes')
    index = 'https://download.pytorch.org/whl/'
    if cpu:
        indexes = [index + 'cpu']
    else:
        gpu = require_probe(python, GPU_CHECK)
        version = gpu['driver_cuda']
        with urllib.request.urlopen(index, timeout=30) as response:
            published = set(re.findall(r'href=["\'](?:https://download.pytorch.org/whl/)?(cu[0-9]+)/?["\']',
                                       response.read().decode()))
        compatible = sorted((int(tag[2:]), tag) for tag in published
                            if int(tag[2:]) // 10 * 1000 + int(tag[2:]) % 10 * 10 <= version)
        indexes = [index + tag for _, tag in compatible[-2:][::-1]]
        if not indexes:
            raise RuntimeError(f'No official Torch CUDA wheel index for driver {version}, SM{gpu["sm"]}')
    failures = []
    directory = Path(python).parents[1]
    for source in indexes:
        # Each attempt starts clean; incompatible wheels must not enter a retry.
        shutil.rmtree(directory)
        create_environment(directory)
        try:
            run([python, '-m', 'pip', 'install', '--ignore-installed', '--only-binary=:all:',
                 '--index-url', source, 'torch>=2.5', 'torchvision', 'numpy<2'],
                env=install_environment())
            result = require_probe(python, TORCH_CHECK)
            result['source'] = source
            return result
        except (subprocess.SubprocessError, RuntimeError) as exc:
            failures.append(f'{source}: {exc}')
    raise RuntimeError('Torch installation/self-test failed; prepare a compatible GPU environment or use Docker.\n' +
                       '\n'.join(failures))


def install_environment() -> dict:
    env = os.environ.copy()
    # A previously sourced Scene environment may pin a different GPU stack.
    for key in ('PIP_CONSTRAINT', 'PIP_BUILD_CONSTRAINT', 'PIP_TARGET', 'PIP_PREFIX', 'PIP_USER', 'PYTHONPATH',
                'PIP_EXTRA_INDEX_URL', 'PIP_FIND_LINKS', 'PIP_NO_INDEX'):
        env.pop(key, None)
    env['PIP_CONFIG_FILE'] = os.devnull
    return env


def create_environment(directory: Path) -> None:
    try:
        run([sys._base_executable, '-m', 'venv', '--system-site-packages', str(directory)])
    except subprocess.CalledProcessError:
        apt_install([f'python{sys.version_info.major}.{sys.version_info.minor}-venv'])
        run([sys._base_executable, '-m', 'venv', '--system-site-packages', str(directory)])
    prefix = Path(sys.prefix)
    if sys.prefix != sys.base_prefix and not prefix.resolve().is_relative_to((BUILD / 'native').resolve()):
        version = f'{sys.version_info.major}.{sys.version_info.minor}'
        site = directory / 'lib' / f'python{version}' / 'site-packages'
        source = prefix / 'lib' / f'python{version}' / 'site-packages'
        (site / 'scene-host.pth').write_text(f'import site; site.addsitedir({str(source)!r})\n')


def cuda_library_paths(version: str) -> list[str]:
    root = Path('/usr/local') / f'cuda-{version}'
    return [str(p) for p in (*root.glob('targets/*/lib'), root / 'lib64') if p.is_dir()]


def package_versions(python: str) -> dict:
    code = '''
import importlib.metadata as m, json
result = {}
for name in ('torch', 'torchvision', 'tensorrt', 'tensorrt-cu12', 'tensorrt-cu13', 'numpy', 'cuda-bindings', 'cuda-pathfinder'):
    try: result[name] = m.version(name)
    except m.PackageNotFoundError: pass
print(json.dumps(result))
'''
    return json.loads(run([python, '-c', code], capture_output=True, text=True).stdout)


def prune_environments(keep) -> None:
    root = BUILD / 'native'
    for directory in root.iterdir():
        if (directory.is_dir() and re.fullmatch(r'[0-9a-f]{64}(?:-[0-9a-f]{8})?', directory.name)
                and directory.name not in keep):
            shutil.rmtree(directory)


def setup(host: dict) -> None:
    ros = install_ros()
    key = environment_key(host)
    root = BUILD / 'native'
    root.mkdir(exist_ok=True)
    marker = BUILD / 'native-env.json'
    published = json.loads(marker.read_text()) if marker.exists() else {}
    prune_environments((published.get('generation'), published.get('previous')))
    directory = root / f'{key}-{uuid.uuid4().hex[:8]}'
    python = str(directory / 'bin/python')
    create_environment(directory)
    torch = ensure_torch(python)
    if inference_backend() == 'tensorrt' and probe(python, TRT_CHECK).returncode:
        apt_install(['python3-libnvinfer'])
        require_probe(python, TRT_CHECK)
    constraints = directory / 'gpu-constraints.txt'
    versions = package_versions(python)
    binding_constraint = ''
    if inference_backend() == 'tensorrt':
        preferred = int((torch['cuda'] or '0').split('.')[0])
        runtime = require_probe(python, CUDART_CHECK.replace('PREFERRED_MAJOR', str(preferred)))
        major = runtime['cuda_runtime'] // 1000
        minimum = '12.8' if major == 12 else str(major)
        binding = versions.get('cuda-bindings')
        if binding:
            installed = tuple(map(int, binding.split('.')[:2]))
            if installed[0] != major or installed < (major, 8 if major == 12 else 0):
                versions.pop('cuda-bindings')
        binding_constraint = f'cuda-bindings>={minimum},<{major + 1}\n'
    constraints.write_text('numpy<2\n' + binding_constraint + ''.join(
        f'{name}=={version}\n' for name, version in versions.items() if name != 'numpy'))
    env = dict(host=host, python=python, ros=ros,
               inference=inference_backend(), key=key, generation=directory.name,
               base_python=sys.executable, torch_source=torch.pop('source', 'existing'),
               torch=torch, cuda_paths=cuda_library_paths(torch['cuda']) if torch['cuda'] else [])
    (BUILD / 'native-candidate.json').write_text(json.dumps(env, indent=2) + '\n')
    write_shell(BUILD / 'native-candidate.sh', env)


def write_shell(path: Path, env: dict) -> None:
    path.write_text('export SCENE_NATIVE_PYTHON=' + shlex.quote(env['python']) + '\n'
                    'export ROS_DISTRO=' + shlex.quote(env['ros']) + '\n'
                    'export PIP_CONSTRAINT=' + shlex.quote(str(Path(env['python']).parents[1] / 'gpu-constraints.txt')) + '\n')
    if env['cuda_paths']:
        with path.open('a') as output:
            output.write('export LD_LIBRARY_PATH=' + shlex.quote(os.pathsep.join(env['cuda_paths'])) +
                         '${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}\n')


def verify(host: dict) -> None:
    candidate = json.loads((BUILD / 'native-candidate.json').read_text())
    if candidate['host'] != host or candidate['key'] != environment_key(host, candidate['base_python']):
        raise RuntimeError('Native build environment changed; rerun the native build')
    python, ros = candidate['python'], candidate['ros']
    if Path(python).absolute() != Path(sys.executable).absolute():
        raise RuntimeError('Verify must use the candidate interpreter selected by native-candidate.sh')
    candidate['torch'] = require_probe(python, TORCH_CHECK)
    require_probe(python, ROS_CHECK, ros)
    if inference_backend() == 'tensorrt':
        candidate.update(require_probe(python, TRT_CHECK))
        candidate.update(require_probe(python, CUDA_BINDINGS_CHECK))
    require_probe(python, STORAGE_CHECK)
    candidate['versions'] = package_versions(python)
    packages = run([python, '-m', 'pip', 'list', '--format=freeze'], capture_output=True, text=True).stdout
    marker = BUILD / 'native-env.json'
    previous = json.loads(marker.read_text()) if marker.exists() else {}
    candidate['previous'] = (previous.get('generation') if previous.get('generation') != candidate['generation']
                             else previous.get('previous'))
    directory = Path(python).parents[1]
    (directory / 'requirements.lock').write_text(packages)
    (directory / 'env.json').write_text(json.dumps(candidate, indent=2) + '\n')
    write_shell(directory / 'env.sh', candidate)
    # Preserve the old selection while migrating file-based installations.
    legacy = BUILD / '.native-legacy'
    if not (BUILD / 'native-current').exists() and marker.is_file():
        legacy.mkdir(exist_ok=True)
        shutil.copyfile(marker, legacy / 'env.json')
        shutil.copyfile(BUILD / 'native-env.sh', legacy / 'env.sh')
        pending = BUILD / '.native-current.tmp'
        pending.unlink(missing_ok=True)
        pending.symlink_to(legacy.relative_to(BUILD), target_is_directory=True)
        os.replace(pending, BUILD / 'native-current')
    # These stable aliases both resolve through one atomically replaced pointer.
    for name, target in (('native-env.json', 'native-current/env.json'),
                         ('native-env.sh', 'native-current/env.sh')):
        alias = BUILD / ('.' + name + '.tmp')
        alias.unlink(missing_ok=True)
        alias.symlink_to(target)
        os.replace(alias, BUILD / name)
    pending = BUILD / '.native-current.tmp'
    pending.unlink(missing_ok=True)
    pending.symlink_to(directory.relative_to(BUILD), target_is_directory=True)
    os.replace(pending, BUILD / 'native-current')
    if legacy.is_dir():
        shutil.rmtree(legacy)
    prune_environments((candidate['generation'], candidate['previous']))
    print('[scene/native] verified environment published', flush=True)


def check(host: dict) -> None:
    saved = json.loads((BUILD / 'native-env.json').read_text())
    if (saved['host'] != host or Path(saved['python']).absolute() != Path(sys.executable).absolute()
            or saved['inference'] != inference_backend()
            or saved['versions'] != package_versions(sys.executable)):
        raise RuntimeError('Native environment changed; rerun the native build')
    if not Path(f"/opt/ros/{saved['ros']}/setup.bash").is_file():
        raise RuntimeError('Prepared ROS environment is missing; rerun the native build')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--plan', action='store_true', help='print the detected environment without installing anything')
    mode.add_argument('--setup', action='store_true', help='prepare a candidate environment')
    mode.add_argument('--verify', action='store_true', help='self-test and publish a completed native build')
    mode.add_argument('--check', action='store_true', help='check recorded versions without importing Torch')
    args = parser.parse_args()
    if inference_backend() == "tensorrt":
        validate_tensorrt_config()
    host = detect()
    if args.plan:
        print(json.dumps(dict(host=host), indent=2))
        return
    if args.check:
        check(host)
        return
    BUILD.mkdir(parents=True, exist_ok=True)
    with (BUILD / '.native-prepare.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if args.setup:
            setup(host)
        else:
            verify(host)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        raise SystemExit(f'[scene/native] {exc}') from exc
