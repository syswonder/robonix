# SPDX-License-Identifier: MulanPSL-2.0
"""Native preparation reuses host capabilities and preserves working environments."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from unittest.mock import Mock
from types import SimpleNamespace

import pytest


def module():
    path = Path(__file__).resolve().parents[1] / 'scripts/prepare_native.py'
    spec = importlib.util.spec_from_file_location('scene_prepare_native', path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def host(l4t='39.2.1', ubuntu='24.04', python='3.12'):
    return dict(l4t=l4t, distro='ubuntu', ubuntu=ubuntu, python=python,
                arch='aarch64', model='NVIDIA Jetson Orin Nano')


def test_setup_reuses_working_torch_without_installing_packages(tmp_path, monkeypatch):
    prepare = module()
    monkeypatch.setattr(prepare, 'BUILD', tmp_path)
    monkeypatch.setattr(prepare, 'environment_key', lambda *_: 'a' * 64)
    monkeypatch.setattr(prepare, 'install_ros', lambda: 'jazzy')
    monkeypatch.setattr(prepare.uuid, 'uuid4', lambda: SimpleNamespace(hex='c' * 32))
    python = tmp_path / 'native' / ('a' * 64 + '-cccccccc') / 'bin/python'
    gpu = dict(torch='2.8.0', torchvision='0.23.0', cuda='12.9', gpu='Orin')
    monkeypatch.setattr(prepare, 'probe', lambda *_: subprocess.CompletedProcess([], 0, json.dumps(gpu), ''))
    monkeypatch.setattr(prepare, 'package_versions', lambda _: {'torch': '2.8.0', 'torchvision': '0.23.0'})
    install = Mock(side_effect=AssertionError('working host must not be changed'))
    monkeypatch.setattr(prepare, 'apt_install', install)
    commands = []

    def create_venv(command, **kwargs):
        commands.append(command)
        assert command[1:4] == ['-m', 'venv', '--system-site-packages']
        python.parent.mkdir(parents=True)
        python.touch()
        (python.parents[1] / f'lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages').mkdir(parents=True)

    monkeypatch.setattr(prepare, 'run', create_venv)
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'torch')
    prepare.setup(host())
    install.assert_not_called()
    candidate = json.loads((tmp_path / 'native-candidate.json').read_text())
    assert candidate['python'] == str(python) and candidate['torch'] == gpu
    assert (python.parents[1] / 'gpu-constraints.txt').read_text() == (
        'numpy<2\ntorch==2.8.0\ntorchvision==0.23.0\n')
    assert not (tmp_path / 'native-env.sh').exists()


@pytest.mark.parametrize('transaction', [
    'Inst nvidia-l4t-core [39.2.1] (39.3.0 NVIDIA)\n',
    'Remv python3-rclpy [1.0]\n',
    'Inst cuda-cudart-13-2 (13.2 NVIDIA)\n',
    'Inst libcudnn9-cuda-12 (9.0 NVIDIA)\n',
])
def test_apt_refuses_bsp_replacement_or_package_removal(monkeypatch, transaction):
    prepare = module()
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return subprocess.CompletedProcess(command, 0, transaction, '')

    monkeypatch.setattr(prepare, 'run', run)
    with pytest.raises(RuntimeError, match='BSP'):
        prepare.apt_install(['python3-libnvinfer'])
    assert len(calls) == 2
    assert '--simulate' in calls[-1]


@pytest.mark.parametrize('failed_check', ['TORCH_CHECK', 'STORAGE_CHECK'])
def test_failed_self_test_keeps_published_environment(tmp_path, monkeypatch, failed_check):
    prepare = module()
    monkeypatch.setattr(prepare, 'BUILD', tmp_path)
    monkeypatch.setattr(prepare, 'environment_key', lambda *_: 'b' * 64)
    candidate = dict(host=host(), key='b' * 64, python=sys.executable, ros='jazzy', base_python=sys.executable)
    (tmp_path / 'native-candidate.json').write_text(json.dumps(candidate))
    (tmp_path / 'native-env.json').write_text('{"key":"old"}')
    (tmp_path / 'native-env.sh').write_text('export SCENE_NATIVE_PYTHON=/scene/old/bin/python\n')
    def probe(python, code, *args):
        if code == getattr(prepare, failed_check):
            raise RuntimeError('self-test failed')
        return {}

    monkeypatch.setattr(prepare, 'require_probe', probe)
    with pytest.raises(RuntimeError, match='self-test'):
        prepare.verify(host())
    assert (tmp_path / 'native-env.json').read_text() == '{"key":"old"}'
    assert '/scene/old/' in (tmp_path / 'native-env.sh').read_text()


def test_startup_check_uses_metadata_and_detects_changed_versions(tmp_path, monkeypatch):
    prepare = module()
    monkeypatch.setattr(prepare, 'BUILD', tmp_path)
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'torch')
    saved = dict(host=host(), python=sys.executable, ros='jazzy', inference='torch',
                 versions={'torch': '2.8.0'})
    (tmp_path / 'native-env.json').write_text(json.dumps(saved))
    original = Path.is_file
    monkeypatch.setattr(Path, 'is_file', lambda p: True if str(p).startswith('/opt/ros/') else original(p))
    monkeypatch.setattr(prepare, 'package_versions', lambda _: {'torch': '2.8.0'})
    monkeypatch.setattr(prepare, 'probe', Mock(side_effect=AssertionError('startup must not import Torch')))
    prepare.check(host())
    monkeypatch.setattr(prepare, 'package_versions', lambda _: {'torch': '2.9.0'})
    with pytest.raises(RuntimeError, match='rerun'):
        prepare.check(host())


def test_missing_ros_module_reuses_existing_apt_source(monkeypatch):
    prepare = module()
    monkeypatch.delenv('ROS_DISTRO', raising=False)
    monkeypatch.setattr(prepare, 'probe', lambda *a: subprocess.CompletedProcess([], 1, '', 'missing ROS module'))
    monkeypatch.setattr(prepare.subprocess, 'run', lambda *a, **k: subprocess.CompletedProcess([], 0, 'ROS package available', ''))
    monkeypatch.setattr(prepare, 'require_probe', lambda *a: {})
    install = Mock()
    monkeypatch.setattr(prepare, 'apt_install', install)
    monkeypatch.setenv('ROS_DISTRO', 'jazzy')
    assert prepare.install_ros() == 'jazzy'
    install.assert_called_once_with(['ros-jazzy-ros-base', 'ros-jazzy-tf2-ros', 'ros-jazzy-rmw-zenoh-cpp'])


def test_rebuild_stages_dependencies_without_changing_published_venv(tmp_path, monkeypatch):
    prepare = module()
    monkeypatch.setattr(prepare, 'BUILD', tmp_path)
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'torch')
    monkeypatch.setattr(prepare, 'environment_key', lambda *_: 'a' * 64)
    monkeypatch.setattr(prepare, 'install_ros', lambda: 'jazzy')
    gpu = dict(torch='2.8.0', torchvision='0.23.0', cuda='12.9')
    monkeypatch.setattr(prepare, 'ensure_torch', lambda *_: gpu)
    monkeypatch.setattr(prepare, 'package_versions', lambda _: {'torch': '2.8.0'})
    identifiers = iter(['b' * 32, 'c' * 32])
    monkeypatch.setattr(prepare.uuid, 'uuid4', lambda: SimpleNamespace(hex=next(identifiers)))

    def create(command, **kwargs):
        directory = Path(command[-1])
        (directory / 'bin').mkdir(parents=True)
        (directory / 'bin/python').touch()
        (directory / f'lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages').mkdir(parents=True)

    monkeypatch.setattr(prepare, 'run', create)
    prepare.setup(host())
    first = json.loads((tmp_path / 'native-candidate.json').read_text())
    marker = tmp_path / 'native-env.json'
    marker.write_text(json.dumps(first))
    owned = Path(first['python']).parents[1] / 'dependency.txt'
    owned.write_text('published dependency')
    original = marker.read_bytes()
    prepare.setup(host())
    second = json.loads((tmp_path / 'native-candidate.json').read_text())
    assert first['key'] == second['key'] and first['python'] != second['python']
    (Path(second['python']).parents[1] / 'dependency.txt').write_text('failed replacement')
    monkeypatch.setattr(prepare, 'sys', SimpleNamespace(executable=second['python']))
    monkeypatch.setattr(prepare, 'require_probe', Mock(side_effect=RuntimeError('CUDA test failed')))
    with pytest.raises(RuntimeError, match='CUDA'):
        prepare.verify(host())
    assert marker.read_bytes() == original
    assert owned.read_text() == 'published dependency'


@pytest.mark.parametrize('legacy_files', [False, True])
def test_verify_publishes_one_pointer_and_retains_previous_on_retry(tmp_path, monkeypatch, legacy_files):
    prepare = module()
    monkeypatch.setattr(prepare, 'BUILD', tmp_path)
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'torch')
    monkeypatch.setattr(prepare, 'environment_key', lambda *_: 'c' * 64)
    root = tmp_path / 'native'
    for key in ('a', 'b', 'c'):
        (root / (key * 64) / 'bin').mkdir(parents=True)
    first = root / ('a' * 64)
    (first / 'env.json').write_text(json.dumps(dict(generation='a' * 64, previous='b' * 64)))
    (first / 'env.sh').write_text('old environment')
    if legacy_files:
        (tmp_path / 'native-env.json').write_bytes((first / 'env.json').read_bytes())
        (tmp_path / 'native-env.sh').write_bytes((first / 'env.sh').read_bytes())
    else:
        (tmp_path / 'native-current').symlink_to(first)
        (tmp_path / 'native-env.json').symlink_to('native-current/env.json')
        (tmp_path / 'native-env.sh').symlink_to('native-current/env.sh')
    candidate = dict(host=host(), key='c' * 64, generation='c' * 64, ros='jazzy',
                     python=str(root / ('c' * 64) / 'bin/python'), base_python='/usr/bin/python3',
                     inference='torch', cuda_paths=[])
    (tmp_path / 'native-candidate.json').write_text(json.dumps(candidate))
    monkeypatch.setattr(prepare, 'sys', SimpleNamespace(executable=candidate['python']))
    monkeypatch.setattr(prepare, 'require_probe', lambda *_: {})
    monkeypatch.setattr(prepare, 'package_versions', lambda _: {'torch': '2.8.0'})
    monkeypatch.setattr(prepare, 'run', lambda *a, **k: subprocess.CompletedProcess([], 0, 'torch==2.8.0\n', ''))
    if legacy_files:
        replace = prepare.os.replace
        def interrupt(source, target):
            if Path(source).name == '.native-env.sh.tmp':
                raise OSError('simulated interruption during migration')
            return replace(source, target)
        monkeypatch.setattr(prepare.os, 'replace', interrupt)
        with pytest.raises(OSError, match='interruption'):
            prepare.verify(host())
        assert json.loads((tmp_path / 'native-env.json').read_text())['generation'] == 'a' * 64
        assert (tmp_path / 'native-env.sh').read_text() == 'old environment'
        monkeypatch.setattr(prepare.os, 'replace', replace)
    prepare.verify(host())
    prepare.verify(host())
    assert (tmp_path / 'native-current').resolve() == root / ('c' * 64)
    saved = json.loads((tmp_path / 'native-env.json').read_text())
    assert saved['previous'] == 'a' * 64
    assert candidate['python'] in (tmp_path / 'native-env.sh').read_text()
    assert first.is_dir() and not (root / ('b' * 64)).exists()


def test_bootstrap_download_rejects_checksum_mismatch(tmp_path, monkeypatch):
    prepare = module()
    from io import BytesIO
    monkeypatch.setattr(prepare.urllib.request, 'urlopen', lambda url, timeout: BytesIO(b'wrong package'))
    with pytest.raises(RuntimeError, match='Checksum'):
        prepare.download_verified({'url': 'https://example.invalid/package.deb', 'sha256': '0' * 64},
                                  tmp_path / 'package.deb')




@pytest.mark.parametrize('model,l4t', [('Jetson Thor', '38.2.1'), ('Unknown Jetson', '40.0.1')])
def test_unknown_board_with_working_dependencies_is_not_rejected(tmp_path, monkeypatch, model, l4t):
    prepare = module()
    monkeypatch.setattr(prepare, 'BUILD', tmp_path)
    monkeypatch.setattr(prepare, 'install_ros', lambda: 'jazzy')
    monkeypatch.setattr(prepare, 'create_environment', lambda p: (p / 'bin').mkdir(parents=True))
    gpu = dict(torch='2.10.0', torchvision='0.25.0', cuda='13.0', gpu=model)
    monkeypatch.setattr(prepare, 'probe', lambda *a: subprocess.CompletedProcess([], 0, json.dumps(gpu), ''))
    monkeypatch.setattr(prepare, 'package_versions', lambda _: {'torch': '2.10.0'})
    monkeypatch.setattr(prepare, 'run', Mock(side_effect=AssertionError('working GPU stack must be reused')))
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'torch')
    environment = dict(host(), model=model, l4t=l4t)
    prepare.setup(environment)
    saved = json.loads((tmp_path / 'native-candidate.json').read_text())
    assert saved['torch'] == gpu and saved['torch_source'] == 'existing'


@pytest.mark.parametrize('first_works,indexes,expected', [
    (True, ['cu130', 'cu132'], 'cu132'),
    (False, ['cu130', 'cu132'], 'cu130'),
    (False, ['cu129', 'cu130'], 'cu129'),
])
def test_missing_torch_tries_official_cuda_candidates_and_validates_gpu(tmp_path, monkeypatch, first_works, indexes, expected):
    from io import BytesIO
    prepare = module()
    directory = tmp_path / 'candidate'
    directory.mkdir()
    python = str(directory / 'bin/python')
    html = ''.join(f'<a href="{index}/">CUDA</a>' for index in indexes + ['cu134'])
    monkeypatch.setattr(prepare.urllib.request, 'urlopen', lambda *a, **k: BytesIO(html.encode()))
    calls = []
    def probe(python, code, ros=''):
        if code == prepare.GPU_CHECK:
            return subprocess.CompletedProcess([], 0, '{"driver_cuda":13020,"sm":"87"}', '')
        works = bool(calls) and (first_works or len(calls) == 2)
        return subprocess.CompletedProcess([], 0 if works else 1,
            '{"torch":"2.10.0","torchvision":"0.25.0","cuda":"13.0"}', 'unsupported GPU')
    monkeypatch.setattr(prepare, 'probe', probe)
    def install(command, **kwargs):
        assert not (directory / 'failed-wheel').exists()
        (directory / 'failed-wheel').touch()
        calls.append(command)
        assert 'PIP_CONSTRAINT' not in kwargs['env']
    monkeypatch.setattr(prepare, 'run', install)
    monkeypatch.setattr(prepare, 'create_environment', lambda p: p.mkdir())
    monkeypatch.delenv('SCENE_CG_FORCE_CPU', raising=False)
    monkeypatch.setenv('PIP_CONSTRAINT', '/old/environment/constraints.txt')
    result = prepare.ensure_torch(python)
    assert len(calls) == (1 if first_works else 2)
    assert calls[0][calls[0].index('--index-url') + 1].endswith('/' + indexes[-1])
    assert result['source'].endswith('/' + expected)
    assert all('--only-binary=:all:' in cmd for cmd in calls)


def test_all_torch_candidates_fail_without_touching_published_environment(tmp_path, monkeypatch):
    from io import BytesIO
    prepare = module()
    directory = tmp_path / 'candidate'
    directory.mkdir()
    marker = tmp_path / 'native-env.json'
    marker.write_text('{"generation":"old"}')
    monkeypatch.setattr(prepare.urllib.request, 'urlopen', lambda *a, **k: BytesIO(b'<a href="cu130/">13</a>'))
    monkeypatch.setattr(prepare, 'probe', lambda p, code, *a: subprocess.CompletedProcess([], 0, '{"driver_cuda":13020,"sm":"87"}', '')
                        if code == prepare.GPU_CHECK else subprocess.CompletedProcess([], 1, '', 'unsupported GPU'))
    monkeypatch.setattr(prepare, 'run', lambda *a, **k: None)
    monkeypatch.setattr(prepare, 'create_environment', lambda p: p.mkdir())
    monkeypatch.delenv('SCENE_CG_FORCE_CPU', raising=False)
    with pytest.raises(RuntimeError, match='unsupported GPU'):
        prepare.ensure_torch(str(directory / 'bin/python'))
    assert marker.read_text() == '{"generation":"old"}'


def test_ros_name_is_validated_before_sourcing_any_path(monkeypatch):
    prepare = module()
    monkeypatch.setenv('ROS_DISTRO', '../../tmp/custom')
    monkeypatch.setattr(prepare, 'probe', Mock(side_effect=AssertionError('must reject before shell')))
    with pytest.raises(RuntimeError, match='Invalid ROS_DISTRO'):
        prepare.install_ros()


def test_legacy_profile_entry_points_still_resolve_docker_profiles():
    prepare = module()
    assert prepare.select_profile(host())['name'] == 'jetpack-7-orin'
    alias = Path(prepare.__file__).with_name('native_profiles.json')
    assert json.loads(alias.read_text()) == json.loads(alias.with_name('docker_profiles.json').read_text())


@pytest.mark.parametrize('runtime,binding,keep_pin', [
    (12060, '13.4.4', False), (13020, '13.4.4', True),
    (12060, '12.7.1', False), (12060, '12.9.10', True),
])
def test_bindings_constraint_follows_working_cuda_runtime(tmp_path, monkeypatch, runtime, binding, keep_pin):
    prepare = module()
    monkeypatch.setattr(prepare, 'BUILD', tmp_path)
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'tensorrt')
    monkeypatch.setattr(prepare, 'install_ros', lambda: 'humble')
    monkeypatch.setattr(prepare, 'environment_key', lambda *_: 'a' * 64)
    monkeypatch.setattr(prepare, 'ensure_torch', lambda _: dict(cuda='12.6', torch='2.10.0'))
    monkeypatch.setattr(prepare, 'probe', lambda *a: subprocess.CompletedProcess([], 0, '', ''))
    monkeypatch.setattr(prepare, 'require_probe', lambda *a: {'cuda_runtime': runtime})
    monkeypatch.setattr(prepare, 'package_versions', lambda _: {'torch': '2.10.0', 'cuda-bindings': binding})
    def create(directory):
        (directory / 'bin').mkdir(parents=True)
        (directory / 'bin/python').touch()
    monkeypatch.setattr(prepare, 'create_environment', create)
    prepare.setup(host())
    candidate = json.loads((tmp_path / 'native-candidate.json').read_text())
    constraints = (Path(candidate['python']).parents[1] / 'gpu-constraints.txt').read_text()
    expected = 'cuda-bindings>=12.8,<13' if runtime // 1000 == 12 else 'cuda-bindings>=13,<14'
    assert expected in constraints
    assert (f'cuda-bindings=={binding}' in constraints) == keep_pin


def test_failed_cuda_bindings_probe_preserves_published_selection(tmp_path, monkeypatch):
    prepare = module()
    monkeypatch.setattr(prepare, 'BUILD', tmp_path)
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'tensorrt')
    monkeypatch.setattr(prepare, 'environment_key', lambda *_: 'b' * 64)
    (tmp_path / 'native-candidate.json').write_text(json.dumps(dict(
        host=host(), key='b' * 64, python=sys.executable, ros='jazzy', base_python=sys.executable)))
    (tmp_path / 'native-env.json').write_text('{"key":"old"}')
    def check(python, code, *args):
        if code == prepare.CUDA_BINDINGS_CHECK:
            raise RuntimeError('CUDA bindings cannot use the driver')
        return {}
    monkeypatch.setattr(prepare, 'require_probe', check)
    with pytest.raises(RuntimeError, match='bindings'):
        prepare.verify(host())
    assert json.loads((tmp_path / 'native-env.json').read_text()) == {'key': 'old'}


@pytest.mark.parametrize('tools_ready', [True, False])
def test_ros_overlay_builds_and_checks_type_support(tmp_path, monkeypatch, tools_ready):
    prepare = module()
    monkeypatch.setattr(prepare, 'BUILD', tmp_path)
    workspace = tmp_path / 'codegen/ros2_idl'
    (workspace / 'src/map').mkdir(parents=True)
    monkeypatch.setattr(prepare, 'probe', lambda *args: subprocess.CompletedProcess(
        [], 0 if tools_ready else 1, '', ''))
    monkeypatch.setattr(prepare, 'require_probe', Mock(return_value={}))
    install = Mock()
    execute = Mock()
    monkeypatch.setattr(prepare, 'apt_install', install)
    monkeypatch.setattr(prepare, 'run', execute)
    prepare.prepare_ros_overlay('humble')
    if tools_ready:
        install.assert_not_called()
    else:
        install.assert_called_once_with([
            'python3-colcon-common-extensions', 'ros-humble-rosidl-default-generators'])
    build, check = execute.call_args_list
    assert build.kwargs['cwd'] == workspace
    assert build.args[0][-2:] == [f'-DPython3_EXECUTABLE={sys.executable}',
                                 f'-DPYTHON_EXECUTABLE={sys.executable}']
    assert str(workspace / 'install/setup.bash') in check.args[0]
    assert check.args[0][-2] == sys.executable


def test_failed_ros_overlay_build_stops_before_type_check(tmp_path, monkeypatch):
    prepare = module()
    monkeypatch.setattr(prepare, 'BUILD', tmp_path)
    (tmp_path / 'codegen/ros2_idl/src/map').mkdir(parents=True)
    monkeypatch.setattr(prepare, 'probe', lambda *args: subprocess.CompletedProcess([], 0, '', ''))
    execute = Mock(side_effect=subprocess.CalledProcessError(1, ['colcon']))
    monkeypatch.setattr(prepare, 'run', execute)
    with pytest.raises(subprocess.CalledProcessError):
        prepare.prepare_ros_overlay('humble')
    assert execute.call_count == 1
