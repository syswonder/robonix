# SPDX-License-Identifier: MulanPSL-2.0
"""Native adaptation selects pinned profiles and preserves working environments."""
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


@pytest.mark.parametrize('environment,ros,cuda', [
    (host(), 'jazzy', '12.9'),
    (host('36.4.4', '22.04', '3.10'), 'humble', '12.6'),
])
def test_profiles_pin_matching_python_wheels(environment, ros, cuda):
    prepare = module()
    profile = prepare.select_profile(environment)
    assert profile['ros'] == ros and profile['cuda'] == cuda
    assert profile['torch'] == '2.8.0' and profile['torchvision'] == '0.23.0'
    tag = 'cp' + environment['python'].replace('.', '')
    assert all(f'{tag}-{tag}-linux_aarch64.whl#sha256=' in url for url in profile['wheels'])


@pytest.mark.parametrize('changes', [
    {'l4t': '38.2.1'}, {'model': 'NVIDIA Jetson Thor'},
    {'python': '3.13'}, {'arch': 'x86_64'}, {'ubuntu': '22.04'}, {'distro': 'debian'},
])
def test_unknown_combinations_stop_before_installation(changes):
    prepare = module()
    environment = host()
    environment.update(changes)
    with pytest.raises(RuntimeError, match='Unsupported'):
        prepare.select_profile(environment)


def test_setup_reuses_working_torch_without_installing_packages(tmp_path, monkeypatch):
    prepare = module()
    monkeypatch.setattr(prepare, 'BUILD', tmp_path)
    monkeypatch.setattr(prepare, 'environment_key', lambda *_: 'a' * 64)
    monkeypatch.setattr(prepare, 'install_ros', lambda _: 'jazzy')
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
        (python.parents[1] / 'lib/python3.12/site-packages').mkdir(parents=True)

    monkeypatch.setattr(prepare, 'run', create_venv)
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'torch')
    prepare.setup(host(), prepare.select_profile(host()))
    install.assert_not_called()
    candidate = json.loads((tmp_path / 'native-candidate.json').read_text())
    assert candidate['python'] == str(python) and candidate['torch'] == gpu
    assert (python.parents[1] / 'gpu-constraints.txt').read_text() == (
        'numpy<2\ntorch==2.8.0\ntorchvision==0.23.0\n')
    assert not (tmp_path / 'native-env.sh').exists()


@pytest.mark.parametrize('missing_runtime', [False, True])
def test_missing_torch_installs_pinned_wheels_into_scene_environment(monkeypatch, missing_runtime):
    prepare = module()
    profile = prepare.select_profile(host())
    responses = iter([subprocess.CompletedProcess([], 1, '', 'Torch missing'),
                      subprocess.CompletedProcess([], int(missing_runtime), '', ''),
                      subprocess.CompletedProcess([], 0, '{"torch":"2.8.0","torchvision":"0.23.0"}', '')])
    monkeypatch.setattr(prepare, 'probe', lambda *_: next(responses))
    monkeypatch.setattr(prepare.subprocess, 'run', lambda *a, **k: subprocess.CompletedProcess([], 0, 'available', ''))
    install = Mock()
    commands = Mock()
    monkeypatch.setattr(prepare, 'apt_install', install)
    monkeypatch.setattr(prepare, 'run', commands)
    assert prepare.ensure_torch('/scene/env/bin/python', profile) == {'torch': '2.8.0', 'torchvision': '0.23.0'}
    if missing_runtime:
        install.assert_called_once()
        assert 'cuda-libraries-12-9' in install.call_args.args[0]
    else:
        install.assert_not_called()
    wheel_command = commands.call_args_list[0].args[0]
    assert wheel_command[:4] == ['/scene/env/bin/python', '-m', 'pip', 'install']
    assert wheel_command[-2:] == profile['wheels']
    assert '--ignore-installed' in wheel_command and '--no-deps' in wheel_command


@pytest.mark.parametrize('transaction', [
    'Inst nvidia-l4t-core [39.2.1] (39.3.0 NVIDIA)\n',
    'Remv python3-rclpy [1.0]\n',
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
        prepare.verify(host(), prepare.select_profile(host()))
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
    monkeypatch.setattr(prepare.urllib.request, 'urlopen', Mock(side_effect=AssertionError('must reuse existing source')))
    monkeypatch.setattr(prepare, 'require_probe', lambda *a: {})
    install = Mock()
    monkeypatch.setattr(prepare, 'apt_install', install)
    assert prepare.install_ros(prepare.select_profile(host())) == 'jazzy'
    install.assert_called_once_with(['ros-jazzy-ros-base', 'ros-jazzy-tf2-ros', 'ros-jazzy-rmw-zenoh-cpp'])


def test_rebuild_stages_dependencies_without_changing_published_venv(tmp_path, monkeypatch):
    prepare = module()
    monkeypatch.setattr(prepare, 'BUILD', tmp_path)
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'torch')
    monkeypatch.setattr(prepare, 'environment_key', lambda *_: 'a' * 64)
    monkeypatch.setattr(prepare, 'install_ros', lambda _: 'jazzy')
    gpu = dict(torch='2.8.0', torchvision='0.23.0', cuda='12.9')
    monkeypatch.setattr(prepare, 'ensure_torch', lambda *_: gpu)
    monkeypatch.setattr(prepare, 'package_versions', lambda _: {'torch': '2.8.0'})
    identifiers = iter(['b' * 32, 'c' * 32])
    monkeypatch.setattr(prepare.uuid, 'uuid4', lambda: SimpleNamespace(hex=next(identifiers)))

    def create(command, **kwargs):
        directory = Path(command[-1])
        (directory / 'bin').mkdir(parents=True)
        (directory / 'bin/python').touch()
        (directory / 'lib/python3.12/site-packages').mkdir(parents=True)

    monkeypatch.setattr(prepare, 'run', create)
    profile = prepare.select_profile(host())
    prepare.setup(host(), profile)
    first = json.loads((tmp_path / 'native-candidate.json').read_text())
    marker = tmp_path / 'native-env.json'
    marker.write_text(json.dumps(first))
    owned = Path(first['python']).parents[1] / 'dependency.txt'
    owned.write_text('published dependency')
    original = marker.read_bytes()
    prepare.setup(host(), profile)
    second = json.loads((tmp_path / 'native-candidate.json').read_text())
    assert first['key'] == second['key'] and first['python'] != second['python']
    (Path(second['python']).parents[1] / 'dependency.txt').write_text('failed replacement')
    monkeypatch.setattr(prepare, 'sys', SimpleNamespace(executable=second['python']))
    monkeypatch.setattr(prepare, 'require_probe', Mock(side_effect=RuntimeError('CUDA test failed')))
    with pytest.raises(RuntimeError, match='CUDA'):
        prepare.verify(host(), profile)
    assert marker.read_bytes() == original
    assert owned.read_text() == 'published dependency'


def test_verify_publishes_one_pointer_and_retains_previous_on_retry(tmp_path, monkeypatch):
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
    profile = prepare.select_profile(host())
    prepare.verify(host(), profile)
    prepare.verify(host(), profile)
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


def test_ros_source_detection_handles_empty_apt_cache_and_disabled_entries(tmp_path, monkeypatch):
    prepare = module()
    monkeypatch.setattr(prepare, 'Path', lambda value: tmp_path if value == '/etc/apt' else Path(value))
    sources = tmp_path / 'sources.list.d'
    sources.mkdir()
    source = sources / 'ros2.sources'
    source.write_text('Types: deb\nURIs: http://packages.ros.org/ros2/ubuntu\nEnabled: no\n')
    assert not prepare.ros_apt_configured()
    source.write_text('Types: deb\nURIs: http://packages.ros.org/ros2/ubuntu\nSuites: noble\n')
    assert prepare.ros_apt_configured()
    source.unlink()
    source = sources / 'ros2.list'
    source.write_text('# deb http://packages.ros.org/ros2/ubuntu noble main\n')
    assert not prepare.ros_apt_configured()
    source.write_text('deb [signed-by=/key.gpg] http://packages.ros.org/ros2/ubuntu noble main\n')
    assert prepare.ros_apt_configured()
