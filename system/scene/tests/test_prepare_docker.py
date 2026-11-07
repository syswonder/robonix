# SPDX-License-Identifier: MulanPSL-2.0
"""Container preparation validates candidates before selecting an image."""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[1]
HARDWARE = dict(name='Orin', sm='87', multiprocessors=8, memory=8 << 30)
PROFILE = dict(name='jetpack-7-orin', l4t_prefix='39.2.', ros='jazzy')


@pytest.fixture
def docker(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / 'scripts'))
    path = ROOT / 'scripts/prepare_docker.py'
    spec = importlib.util.spec_from_file_location('scene_prepare_docker', path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    for key in ('ROBONIX_FORCE_CPU', 'SCENE_CG_FORCE_CPU', 'SCENE_PROFILE',
                'SCENE_CLIP_MODEL', 'SCENE_CLIP_PRETRAINED', 'SCENE_PERCEPTION_BACKEND',
                'ROBONIX_SCENE_IMAGE', 'SCENE_MODELS_DIR'):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'torch')
    return result


def test_docker_profile_does_not_require_host_python_to_match(docker):
    host = dict(distro='ubuntu', arch='aarch64', model='Jetson Orin Nano',
                l4t='39.2.1', ubuntu='24.04', python='3.13')
    assert docker.select_profile(host, check_python=False)['ros'] == 'jazzy'
    with pytest.raises(RuntimeError, match='Unsupported'):
        docker.select_profile(host, check_python=True)


@pytest.mark.parametrize('key,value', [
    ('SCENE_PROFILE', 'unsupported'), ('SCENE_CG_FORCE_CPU', '1'),
    ('ROBONIX_FORCE_CPU', '1'),
    ('SCENE_PERCEPTION_BACKEND', 'unknown'),
])
def test_invalid_trt_configuration_stops_before_touching_images(docker, monkeypatch, key, value):
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'tensorrt')
    monkeypatch.setenv(key, value)
    image = Mock(side_effect=AssertionError('must not touch images'))
    monkeypatch.setattr(docker, 'inspect', image)
    with pytest.raises((ValueError, RuntimeError)):
        docker.check({'l4t': '39.2.1'})
    image.assert_not_called()


def test_candidate_checks_receive_the_actual_runtime_configuration(docker, monkeypatch, tmp_path):
    monkeypatch.setenv('SCENE_PROFILE', 'full')
    monkeypatch.setenv('SCENE_CLIP_MODEL', 'ViT-H-14')
    monkeypatch.setenv('SCENE_CLIP_PRETRAINED', '/opt/models/full/clip.bin')
    monkeypatch.setenv('SCENE_OPEN_VOCAB_CLASSES', 'chair,table')
    monkeypatch.setenv('SCENE_MODELS_DIR', str(tmp_path))
    monkeypatch.setattr(docker, 'gpu_args', lambda: ['--runtime', 'nvidia'])
    execute = Mock(return_value=subprocess.CompletedProcess([], 0, '', ''))
    monkeypatch.setattr(docker, 'run', execute)
    monkeypatch.setattr(docker.subprocess, 'run', lambda *a, **k: None)
    docker.image_run('image', ['python3', 'check.py'], inference='torch')
    args = execute.call_args.args[0]
    assert f'{docker.os.getuid()}:{docker.os.getgid()}' in args
    assert 'HOME=/tmp' in args
    assert 'USER=scene' in args
    assert 'SCENE_PROFILE=full' in args
    assert 'SCENE_CLIP_MODEL=ViT-H-14' in args
    assert 'SCENE_CLIP_PRETRAINED=/opt/models/full/open_clip_vit_h14_laion2b_s32b_b79k.bin' in args
    assert 'SCENE_OPEN_VOCAB_CLASSES=chair,table' in args
    assert f'{tmp_path}:/opt/models/full:ro' not in args


def test_unset_model_options_preserve_baked_image_defaults(docker, monkeypatch):
    monkeypatch.setattr(docker, 'gpu_args', lambda: [])
    execute = Mock(return_value=subprocess.CompletedProcess([], 0, '', ''))
    monkeypatch.setattr(docker, 'run', execute)
    monkeypatch.setattr(docker.subprocess, 'run', lambda *a, **k: None)
    docker.image_run('image', ['check'], inference='torch')
    args = execute.call_args.args[0]
    assert not any(arg.startswith('SCENE_CLIP_PRETRAINED=') for arg in args)
    assert not any(arg.startswith('SCENE_CLIP_MODEL=') for arg in args)


def test_cpu_mode_disables_nvidia_injection(docker, monkeypatch):
    monkeypatch.setenv('ROBONIX_FORCE_CPU', '1')
    monkeypatch.setattr(docker, 'run', Mock(side_effect=AssertionError('no GPU runtime needed')))
    assert docker.gpu_args() == ['-e', 'NVIDIA_VISIBLE_DEVICES=void']


def test_runtime_cache_reuses_dependencies_but_separates_hardware(docker, monkeypatch):
    monkeypatch.setattr(docker, 'inspect', lambda image: {'Config': {'Labels': {'io.robonix.scene.base': 'same-base'}}})
    monkeypatch.setattr(docker, 'run', lambda *a, **k: subprocess.CompletedProcess([], 0, 'a' * 64 + ' file', ''))
    first = docker.runtime_key('app-v1')
    assert docker.runtime_key('app-v2') == first
    assert docker.cache_key(first, HARDWARE) == docker.cache_key(first, dict(HARDWARE))
    other = dict(HARDWARE, multiprocessors=4)
    assert docker.cache_key(first, other) != docker.cache_key(first, HARDWARE)
    assert docker.cache_key('different-runtime', HARDWARE) != docker.cache_key(first, HARDWARE)


def build_fixture(docker, monkeypatch, tmp_path):
    monkeypatch.setattr(docker, 'BUILD', tmp_path)
    monkeypatch.setattr(docker, 'gpu_args', lambda: [])
    monkeypatch.setattr(docker, 'stage', Mock())
    monkeypatch.setattr(docker, 'build_image', Mock())
    monkeypatch.setattr(docker, 'source_hash', lambda: 'source')
    monkeypatch.setattr(docker, 'runtime_key', lambda _: 'runtime')
    monkeypatch.setattr(docker, 'inspect', lambda _: {'Id': 'sha256:' + 'b' * 64})
    execute = Mock(return_value=subprocess.CompletedProcess([], 0, '', ''))
    monkeypatch.setattr(docker, 'run', execute)
    monkeypatch.setattr(docker.subprocess, 'run', lambda *a, **k: None)
    return execute


def test_failed_candidate_preserves_previous_image_and_model_cache(docker, monkeypatch, tmp_path):
    execute = build_fixture(docker, monkeypatch, tmp_path)
    old = dict(image='sha256:' + 'a' * 64, inference='torch', cache='c' * 32)
    marker = tmp_path / 'docker-env.json'
    marker.write_text(json.dumps(old))
    original = marker.read_bytes()
    cache = tmp_path / 'docker-models' / old['cache']
    cache.mkdir(parents=True)
    (cache / 'model.trt').write_bytes(b'working engine')
    monkeypatch.setattr(docker, 'check_gpu', Mock(side_effect=[HARDWARE, RuntimeError('inference failed')]))
    with pytest.raises(RuntimeError, match='inference failed'):
        docker.build(PROFILE)
    assert marker.read_bytes() == original
    assert (cache / 'model.trt').read_bytes() == b'working engine'
    assert list((tmp_path / 'docker-models').iterdir()) == [cache]
    execute.assert_not_called()


def test_source_update_keeps_sdk_model_cache_and_pins_previous_image(docker, monkeypatch, tmp_path):
    execute = build_fixture(docker, monkeypatch, tmp_path)
    monkeypatch.setattr(docker, 'check_gpu', lambda *a, **k: HARDWARE)
    docker.build(PROFILE)
    marker = tmp_path / 'docker-env.json'
    first = json.loads(marker.read_text())
    model = tmp_path / 'docker-models' / first['cache'] / 'model.trt'
    model.write_bytes(b'verified engine')
    monkeypatch.setattr(docker, 'inspect', lambda _: {'Id': 'sha256:' + 'd' * 64})
    monkeypatch.setattr(docker, 'source_hash', lambda: 'updated-source')
    docker.build(PROFILE)
    second = json.loads(marker.read_text())
    assert second['image'] != first['image'] and second['cache'] == first['cache']
    assert second['previous']['image'] == first['image']
    assert model.read_bytes() == b'verified engine'
    assert any('robonix-scene-rollback:jetpack-7-orin-torch' in call.args[0] for call in execute.call_args_list)


def test_startup_rejects_an_unverified_image_override(docker, monkeypatch, tmp_path):
    monkeypatch.setattr(docker, 'BUILD', tmp_path)
    selected = dict(image='sha256:' + 'a' * 64, inference='torch', source='source', l4t_prefix='39.2.')
    (tmp_path / 'docker-env.json').write_text(json.dumps(selected))
    monkeypatch.setattr(docker, 'source_hash', lambda: 'source')
    monkeypatch.setattr(docker, 'inspect', lambda _: {'Id': 'different-image', 'Config': {'Labels': {}}})
    monkeypatch.setattr(docker, 'check_gpu', Mock(side_effect=AssertionError('must not launch')))
    monkeypatch.setenv('ROBONIX_SCENE_IMAGE', 'unverified:latest')
    with pytest.raises(RuntimeError, match='differs'):
        docker.check({'l4t': '39.2.1'})


def test_proxy_values_are_redacted_in_failure_messages(docker):
    command = ['docker', 'build', '--build-arg', 'HTTPS_PROXY=http://name:private-password@proxy:3128']
    assert 'private-password' not in docker.format_command(command)
    assert 'HTTPS_PROXY=<configured>' in docker.format_command(command)


def test_verified_bootstrap_artifacts_rebuild_without_network(docker, monkeypatch, tmp_path):
    monkeypatch.setattr(docker, 'BUILD', tmp_path)
    content = b'signed bootstrap package'
    source = dict(url='https://example.invalid/sdk.asc', sha256=hashlib.sha256(content).hexdigest())
    download = Mock(side_effect=lambda spec, target: target.write_bytes(content))
    monkeypatch.setattr(docker, 'download_verified', download)
    destination = tmp_path / 'key.asc'
    docker.stage_download(source, destination)
    docker.stage_download(source, destination)
    assert destination.read_bytes() == content
    assert download.call_count == 1
    next((tmp_path / 'downloads').iterdir()).write_bytes(b'corrupted cache')
    docker.stage_download(source, destination)
    assert download.call_count == 2 and destination.read_bytes() == content


def test_source_changes_require_local_rebuild_before_gpu_checks(docker, monkeypatch, tmp_path):
    monkeypatch.setattr(docker, 'BUILD', tmp_path)
    selected = dict(image='sha256:verified', inference='torch', source='old-source',
                    l4t_prefix='39.2.', cache='c' * 32)
    (tmp_path / 'docker-env.json').write_text(json.dumps(selected))
    monkeypatch.setattr(docker, 'source_hash', lambda: 'changed-source')
    monkeypatch.setattr(docker, 'inspect', Mock(side_effect=AssertionError('must reject before image access')))
    with pytest.raises(RuntimeError, match='rerun'):
        docker.check({'l4t': '39.2.1'})


def test_full_model_choice_reaches_docker_build(docker, monkeypatch):
    monkeypatch.setenv("SCENE_PROFILE", "full")
    monkeypatch.setattr(docker, "inspect", lambda _: {"Id": "sha256:base"})
    execute = Mock()
    monkeypatch.setattr(docker, "run", execute)
    profile = json.loads(docker.PROFILES.read_text())["profiles"][0]
    docker.build_image(profile, "tensorrt", "source", "candidate")
    command = execute.call_args.args[0]
    assert "SCENE_PROFILE=full" in command
    assert "SCENE_CLIP_MODEL=ViT-H-14" in command


def test_dualmap_tensorrt_configuration_is_supported(docker, monkeypatch):
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'tensorrt')
    monkeypatch.setenv('SCENE_PERCEPTION_BACKEND', 'dualmap')
    docker.validate_config()


@pytest.mark.parametrize('inference', ['torch', 'tensorrt'])
def test_dualmap_stage_copies_all_optional_models(docker, monkeypatch, tmp_path, inference):
    from scripts import prepare_dualmap
    pkg = tmp_path / 'scene'
    build = pkg / 'rbnx-build'
    (pkg / 'docker/_weights').mkdir(parents=True)
    for name in ('scene_service', 'scripts'):
        (pkg / name).mkdir()
    (pkg / 'package_manifest.jetson-docker.yaml').write_text('name: scene')
    clip = pkg / 'docker/_weights/clip/ViT-B-32.pt'
    clip.parent.mkdir()
    clip.write_bytes(b'clip')
    profiles = tmp_path / 'profiles.json'
    profiles.write_text(json.dumps({'clip_source': {'sha256': hashlib.sha256(b'clip').hexdigest()}}))
    monkeypatch.setattr(docker, 'PKG', pkg)
    monkeypatch.setattr(docker, 'BUILD', build)
    monkeypatch.setattr(docker, 'PROFILES', profiles)
    monkeypatch.setenv('SCENE_PERCEPTION_BACKEND', 'dualmap')
    fastsam = tmp_path / 'custom-fastsam.pt'
    fastsam.write_bytes(b'custom fastsam')
    monkeypatch.setenv('SCENE_DUALMAP_FASTSAM_WEIGHTS', str(fastsam))
    selected = docker.model_config(weights_dir=pkg / 'docker/_weights')
    for key in ('yolo', 'sam', 'clip_pretrained', 'replica_clip_pretrained'):
        path = Path(selected[key])
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(key.encode())
    (pkg / 'docker/dualmap-inference.patch').write_bytes(b'patch')
    source = tmp_path / 'dualmap'
    source.mkdir()
    (source / 'LICENSE').write_text('license')
    monkeypatch.setattr(prepare_dualmap, 'prepare', lambda: source)
    for relative in ('proto-staging', 'codegen/robonix_mcp_types', 'codegen/ros2_idl/src'):
        (build / relative).mkdir(parents=True)
    api = tmp_path / 'api'
    (api / 'robonix_api').mkdir(parents=True)
    runtime = tmp_path / 'runtime'
    runtime.mkdir()
    monkeypatch.setattr(docker, 'run', lambda command, **kwargs: subprocess.CompletedProcess(
        command, 0, str(api if command[-1] == 'robonix-api' else runtime), ''))
    monkeypatch.setattr(docker, 'stage_download', lambda source, destination: destination.write_bytes(b'download'))
    profile = {'ros_source': {}, 'wheels': [], 'torch': '2.5', 'torchvision': '0.20',
               'docker': {'tensorrt_packages': [], 'tensorrt_suite': ''}}
    context = docker.stage(profile, inference, 'source')
    assert (context / 'models/dualmap/mobileclip_s2_datacompdr.bin').read_bytes() == b'clip_pretrained'
    packaged = context / 'models/dualmap/FastSAM-s.pt'
    assert packaged.exists()
    if packaged.exists():
        assert packaged.read_bytes() == b'custom fastsam'
    assert (context / 'dualmap/LICENSE').is_file()


def test_gpu_runtime_requests_compute_without_graphics_injection(docker, monkeypatch):
    execute = Mock(return_value=subprocess.CompletedProcess([], 0, '{"nvidia": {}}', ''))
    monkeypatch.setattr(docker, 'run', execute)
    assert docker.gpu_args() == ['--runtime', 'nvidia', '-e', 'NVIDIA_DRIVER_CAPABILITIES=compute,utility']
    execute.return_value = subprocess.CompletedProcess([], 0, '{}', '')
    with pytest.raises(RuntimeError, match='NVIDIA runtime is missing'):
        docker.gpu_args()
