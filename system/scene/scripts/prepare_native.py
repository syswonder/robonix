#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Select and prepare a Scene native environment for a supported Jetson."""
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
import tempfile
import urllib.request
import uuid

PKG = Path(__file__).resolve().parents[1]
BUILD = PKG / 'rbnx-build'
PROFILES = Path(__file__).with_name('native_profiles.json')
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
CUDA_CHECK = """
import ctypes
for name in ('libcudart.so.12', 'libcublas.so.12', 'libcusparse.so.12', 'libcudnn.so.9'):
    ctypes.CDLL(name)
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
    profiles = json.loads(PROFILES.read_text())['profiles']
    for profile in profiles:
        if (host['distro'] == 'ubuntu' and host['arch'] == 'aarch64' and 'orin' in host['model'].lower()
                and host['l4t'].startswith(profile['l4t_prefix'])
                and host['ubuntu'] == profile['ubuntu']
                and (not check_python or host['python'] == profile['python'])):
            return profile
    raise RuntimeError(f'Unsupported Scene native environment: {host}; '
                       'see scripts/native_profiles.json for supported combinations')


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
           (line.startswith('Inst ') and line.split()[1].startswith(('nvidia-l4t-', 'nvidia-jetpack')))
           for line in plan.splitlines()):
        raise RuntimeError('Dependency installation would replace/remove JetPack BSP packages; '
                           'prepare the matching JetPack runtime before retrying')
    run(privilege + command)


def download_verified(source: dict, destination: Path) -> None:
    with urllib.request.urlopen(source['url'], timeout=30) as response, destination.open('wb') as output:
        shutil.copyfileobj(response, output)
    if hashlib.sha256(destination.read_bytes()).hexdigest() != source['sha256']:
        raise RuntimeError(f"Checksum mismatch for {source['url']}")


def ros_apt_configured() -> bool:
    root = Path('/etc/apt')
    sources = [root / 'sources.list', *(root / 'sources.list.d').glob('*.list'),
               *(root / 'sources.list.d').glob('*.sources')]
    for path in sources:
        if not path.is_file():
            continue
        for block in path.read_text().split('\n\n'):
            if re.search(r'(?im)^Enabled:\s*no\s*$', block):
                continue
            if any('/ros2/ubuntu' in line or '/ros2-testing/ubuntu' in line
                   for line in block.splitlines() if not line.lstrip().startswith('#')):
                return True
    return False


def install_ros(profile: dict) -> str:
    ros = os.environ.get('ROS_DISTRO') or profile['ros']
    if ros != profile['ros']:
        raise RuntimeError(f"This native profile requires ROS_DISTRO={profile['ros']}, got {ros}")
    if Path(f'/opt/ros/{ros}/setup.bash').is_file() and probe(sys.executable, ROS_CHECK, ros).returncode == 0:
        return ros
    # Use the ROS-maintained apt-source package to install its signing key.
    available = subprocess.run(['apt-cache', 'show', f'ros-{ros}-ros-base'],
                               capture_output=True, text=True)
    if available.returncode and not ros_apt_configured():
        with tempfile.TemporaryDirectory() as temporary:
            package = Path(temporary) / 'ros2-apt-source.deb'
            download_verified(profile['ros_source'], package)
            apt_install([str(package)])
    apt_install([f'ros-{ros}-{name}' for name in (
        'ros-base', 'tf2-ros', 'rmw-zenoh-cpp',
    )])
    require_probe(sys.executable, ROS_CHECK, ros)
    return ros


def environment_key(host: dict, profile: dict, base_python: str = None) -> str:
    digest = hashlib.sha256(json.dumps([host, profile, inference_backend()], sort_keys=True).encode())
    for path in [Path(__file__), *sorted((PKG / 'docker/requirements').glob('scene-*.txt'))]:
        digest.update(path.read_bytes())
    digest.update(str(Path(base_python or sys.executable).absolute()).encode())
    return digest.hexdigest()


def ensure_torch(python: str, profile: dict) -> dict:
    existing = probe(python, TORCH_CHECK)
    if existing.returncode == 0:
        print('[scene/native] reusing validated Torch/torchvision', flush=True)
        return json.loads(existing.stdout.strip().splitlines()[-1])
    paths = cuda_library_paths(profile['cuda'])
    if paths:
        os.environ['LD_LIBRARY_PATH'] = os.pathsep.join(paths + [os.environ.get('LD_LIBRARY_PATH', '')])
    if probe(python, CUDA_CHECK).returncode:
        cuda = profile['cuda'].replace('.', '-')
        available = subprocess.run(['apt-cache', 'show', f'cuda-libraries-{cuda}'],
                                   capture_output=True, text=True)
        if available.returncode and profile['ubuntu'] == '24.04':
            with tempfile.TemporaryDirectory() as temporary:
                package = Path(temporary) / 'cuda-keyring.deb'
                download_verified(json.loads(PROFILES.read_text())['cuda_keyring'], package)
                apt_install([str(package)])
        apt_install([f'cuda-libraries-{cuda}', f'cuda-cupti-{cuda}', 'libcudnn9-cuda-12', 'libopenblas0'])
    # Install into Scene's venv; existing host wheels must remain untouched.
    paths = cuda_library_paths(profile['cuda'])
    if paths:
        os.environ['LD_LIBRARY_PATH'] = os.pathsep.join(paths + [os.environ.get('LD_LIBRARY_PATH', '')])
    run([python, '-m', 'pip', 'install', '--ignore-installed', '--no-deps', *profile['wheels']])
    run([python, '-m', 'pip', 'install', f"torch=={profile['torch']}",
         f"torchvision=={profile['torchvision']}", 'numpy<2'])
    result = require_probe(python, TORCH_CHECK)
    for name in ('torch', 'torchvision'):
        if result[name].split('+')[0] != profile[name]:
            raise RuntimeError(f'Installed {name} does not match the selected native profile')
    return result


def cuda_library_paths(version: str) -> list[str]:
    root = Path('/usr/local') / f'cuda-{version}'
    return [str(p) for p in (*root.glob('targets/*/lib'), root / 'lib64') if p.is_dir()]


def package_versions(python: str) -> dict:
    code = '''
import importlib.metadata as m, json
result = {}
for name in ('torch', 'torchvision', 'tensorrt', 'tensorrt-cu12', 'tensorrt-cu13', 'numpy'):
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


def setup(host: dict, profile: dict) -> None:
    ros = install_ros(profile)
    key = environment_key(host, profile)
    root = BUILD / 'native'
    root.mkdir(exist_ok=True)
    marker = BUILD / 'native-env.json'
    published = json.loads(marker.read_text()) if marker.exists() else {}
    prune_environments((published.get('generation'), published.get('previous')))
    directory = root / f'{key}-{uuid.uuid4().hex[:8]}'
    python = str(directory / 'bin/python')
    try:
        run([sys._base_executable, '-m', 'venv', '--system-site-packages', str(directory)])
    except subprocess.CalledProcessError:
        apt_install([f"python{profile['python']}-venv"])
        run([sys._base_executable, '-m', 'venv', '--system-site-packages', str(directory)])
    # An explicitly selected external venv can carry Jetson-specific wheels.
    # Do not inherit Scene generations: they are pruned independently.
    prefix = Path(sys.prefix)
    if sys.prefix != sys.base_prefix and not prefix.resolve().is_relative_to(root.resolve()):
        site = directory / 'lib' / f"python{profile['python']}" / 'site-packages'
        source = prefix / 'lib' / f"python{profile['python']}" / 'site-packages'
        (site / 'scene-host.pth').write_text(f'import site; site.addsitedir({str(source)!r})\n')
    torch = ensure_torch(python, profile)
    if inference_backend() == 'tensorrt' and probe(python, TRT_CHECK).returncode:
        apt_install(['python3-libnvinfer'])
        require_probe(python, TRT_CHECK)
    constraints = directory / 'gpu-constraints.txt'
    versions = package_versions(python)
    constraints.write_text('numpy<2\n' + ''.join(
        f'{name}=={version}\n' for name, version in versions.items() if name != 'numpy'))
    env = dict(host=host, profile=profile['name'], python=python, ros=ros,
               inference=inference_backend(), key=key, generation=directory.name,
               base_python=sys.executable, torch=torch, cuda_paths=cuda_library_paths(torch['cuda']))
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


def verify(host: dict, profile: dict) -> None:
    candidate = json.loads((BUILD / 'native-candidate.json').read_text())
    if candidate['host'] != host or candidate['key'] != environment_key(host, profile, candidate['base_python']):
        raise RuntimeError('Native build environment changed; rerun the native build')
    python, ros = candidate['python'], candidate['ros']
    if Path(python).absolute() != Path(sys.executable).absolute():
        raise RuntimeError('Verify must use the candidate interpreter selected by native-candidate.sh')
    candidate['torch'] = require_probe(python, TORCH_CHECK)
    require_probe(python, ROS_CHECK, ros)
    if inference_backend() == 'tensorrt':
        candidate.update(require_probe(python, TRT_CHECK))
    require_probe(python, STORAGE_CHECK)
    candidate['versions'] = package_versions(python)
    marker = BUILD / 'native-env.json'
    previous = json.loads(marker.read_text()) if marker.exists() else {}
    candidate['previous'] = (previous.get('generation') if previous.get('generation') != candidate['generation']
                             else previous.get('previous'))
    directory = Path(python).parents[1]
    (directory / 'env.json').write_text(json.dumps(candidate, indent=2) + '\n')
    write_shell(directory / 'env.sh', candidate)
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
    mode.add_argument('--plan', action='store_true', help='print detected profile without installing anything')
    mode.add_argument('--setup', action='store_true', help='prepare a candidate environment')
    mode.add_argument('--verify', action='store_true', help='self-test and publish a completed native build')
    mode.add_argument('--check', action='store_true', help='check recorded versions without importing Torch')
    args = parser.parse_args()
    if inference_backend() == "tensorrt":
        validate_tensorrt_config()
    host = detect()
    profile = select_profile(host)
    if args.plan:
        print(json.dumps(dict(host=host, profile=profile), indent=2))
        return
    if args.check:
        check(host)
        return
    BUILD.mkdir(parents=True, exist_ok=True)
    with (BUILD / '.native-prepare.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if args.setup:
            setup(host, profile)
        else:
            verify(host, profile)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        raise SystemExit(f'[scene/native] {exc}') from exc
