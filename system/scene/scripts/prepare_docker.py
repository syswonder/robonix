#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Build and validate a local Jetson Scene image before selecting it."""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import uuid

PKG = Path(__file__).resolve().parents[1]
BUILD = PKG / 'rbnx-build'
PROFILES = Path(__file__).with_name('docker_profiles.json')
sys.path.insert(0, str(PKG))
from scene_service.inference_config import inference_backend, validate_tensorrt_config  # noqa: E402
from scripts.prepare_native import detect, download_verified  # noqa: E402


def select_profile(host: dict, *, check_python: bool = False) -> dict:
    for profile in json.loads(PROFILES.read_text())['profiles']:
        if (host['distro'] == 'ubuntu' and host['arch'] == 'aarch64' and 'orin' in host['model'].lower()
                and host['l4t'].startswith(profile['l4t_prefix'])
                and host['ubuntu'] == profile['ubuntu']
                and (not check_python or host['python'] == profile['python'])):
            return profile
    raise RuntimeError(f'Unsupported Docker build environment: {host}; see scripts/docker_profiles.json')


def format_command(command):
    return shlex.join(re.sub(r'^(HTTP_PROXY|HTTPS_PROXY|NO_PROXY)=.*', r'\1=<configured>', str(arg))
                      for arg in command)


def run(command, *, capture=False, **kwargs):
    if not capture:
        print('[scene/docker] ' + format_command(command), flush=True)
    try:
        return subprocess.run(command, check=True, text=True, capture_output=capture, **kwargs)
    except subprocess.CalledProcessError as exc:
        if capture:
            print((exc.stdout or '') + (exc.stderr or ''), file=sys.stderr)
        raise


def inspect(image: str) -> dict:
    return json.loads(run(['docker', 'image', 'inspect', image], capture=True).stdout)[0]


def force_cpu() -> bool:
    return os.environ.get('ROBONIX_FORCE_CPU') == '1' or os.environ.get('SCENE_CG_FORCE_CPU', '').lower() in ('1', 'true', 'yes')


def validate_config():
    if inference_backend() == 'tensorrt':
        validate_tensorrt_config()
        if force_cpu():
            raise RuntimeError('TensorRT requires GPU passthrough; unset ROBONIX_FORCE_CPU')
    if os.environ.get('SCENE_PERCEPTION_BACKEND', '').strip().lower() not in ('', 'concept_graphs'):
        raise RuntimeError('The packaged Jetson image supports SCENE_PERCEPTION_BACKEND=concept_graphs')


def gpu_args() -> list[str]:
    if force_cpu():
        return ['-e', 'NVIDIA_VISIBLE_DEVICES=void']
    info = json.loads(run(['docker', 'info', '--format', '{{json .Runtimes}}'], capture=True).stdout)
    if 'nvidia' not in info:
        raise RuntimeError('Docker NVIDIA runtime is missing; configure NVIDIA Container Toolkit before building Scene')
    return ['--runtime', 'nvidia', '-e', 'NVIDIA_DRIVER_CAPABILITIES=all']


def file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def source_hash() -> str:
    digest = hashlib.sha256()
    paths = [PKG / 'scene_service', PKG / 'scripts', PKG / 'docker/requirements']
    files = [PKG / 'docker/Dockerfile.jetson', PKG / 'docker/entrypoint.sh',
             PKG / 'package_manifest.jetson-docker.yaml']
    for root in paths:
        files.extend(p for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc')
    for path in sorted(files):
        digest.update(str(path.relative_to(PKG)).encode())
        digest.update(file_hash(path).encode())
    return digest.hexdigest()


def image_run(image: str, command: list[str], *, inference: str, cache=None, capture=False):
    name = 'scene-prepare-' + uuid.uuid4().hex[:12]
    args = ['docker', 'run', '--rm', '--name', name, '--user', f'{os.getuid()}:{os.getgid()}',
            '-e', 'HOME=/tmp', '-e', 'USER=scene', '-e', 'LOGNAME=scene', *gpu_args(),
            '-e', f'SCENE_CG_INFERENCE={inference}', '-e', f"SCENE_PERCEPTION_BACKEND={os.environ.get('SCENE_PERCEPTION_BACKEND', 'concept_graphs')}",
            '-e', f"SCENE_CG_FORCE_CPU={'1' if force_cpu() else ''}",
            '-e', 'SCENE_ONNX_DIR=/scene-models/onnx', '-e', 'SCENE_ENGINE_CACHE_DIR=/scene-models/engines',
            '-e', 'OPENBLAS_NUM_THREADS=1', '-e', 'OMP_NUM_THREADS=2']
    for key in ('SCENE_PROFILE', 'SCENE_CLIP_MODEL', 'SCENE_CLIP_PRETRAINED', 'SCENE_OPEN_VOCAB_CLASSES'):
        if os.environ.get(key):
            args += ['-e', f'{key}={os.environ[key]}']
    if os.environ.get('SCENE_MODELS_DIR'):
        full_models = Path(os.environ['SCENE_MODELS_DIR']).resolve(strict=True)
        args += ['-v', f'{full_models}:/opt/models/full:ro']
    if cache is not None:
        args += ['-v', f'{cache}:/scene-models']
    if 'CUDA_VISIBLE_DEVICES' in os.environ:
        args += ['-e', f"CUDA_VISIBLE_DEVICES={os.environ['CUDA_VISIBLE_DEVICES']}"]
    args += ['--entrypoint', 'bash', image, '-c',
             'set -e; mkdir -p /tmp/Ultralytics; '
             'cp /opt/ultralytics/Ultralytics/settings.json /tmp/Ultralytics/; '
             'export YOLO_CONFIG_DIR=/tmp; source /opt/ros/$ROS_DISTRO/setup.bash; exec "$@"', 'scene-check', *command]
    try:
        return run(args, capture=capture)
    finally:
        subprocess.run(['docker', 'rm', '-f', name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def check_gpu(image, inference, *, quick=False, cache=None):
    command = ['python3', '/scene/scripts/check_docker.py', '--inference', inference]
    if quick:
        command.append('--quick')
    result = image_run(image, command, inference=inference, cache=cache, capture=True)
    # Model libraries can write diagnostics before the final metadata line.
    print(result.stderr, end='', file=sys.stderr)
    return json.loads(result.stdout.strip().splitlines()[-1])


def runtime_key(image: str) -> str:
    frozen = run(['docker', 'run', '--rm', '--network', 'none', '--entrypoint', 'sha256sum',
                  image, '/scene/container-requirements.txt'], capture=True).stdout.split()[0]
    if not re.fullmatch('[0-9a-f]{64}', frozen):
        raise RuntimeError('Invalid dependency fingerprint in candidate image')
    base = inspect(image)['Config']['Labels']['io.robonix.scene.base']
    return hashlib.sha256(json.dumps([base, frozen]).encode()).hexdigest()


def cache_key(runtime_id: str, hardware: dict) -> str:
    return hashlib.sha256(json.dumps([runtime_id, hardware], sort_keys=True).encode()).hexdigest()[:32]


def stage_download(source: dict, destination: Path):
    artifact = BUILD / 'downloads' / (source['sha256'] + destination.suffix)
    artifact.parent.mkdir(parents=True, exist_ok=True)
    if not artifact.exists() or file_hash(artifact) != source['sha256']:
        download_verified(source, artifact)
    shutil.copy2(artifact, destination)


def stage(profile: dict, inference: str, fingerprint: str) -> Path:
    context = PKG / 'docker/_build'
    if context.exists():
        shutil.rmtree(context)
    context.mkdir()
    stage_download(profile['ros_source'], context / 'ros-source.deb')
    (context / 'gpu-wheels.txt').write_text('\n'.join(profile['wheels']) + '\n')
    (context / 'gpu-constraints.txt').write_text(
        f"torch=={profile['torch']}\ntorchvision=={profile['torchvision']}\nnumpy<2\n")
    policy = dict(profile=profile, inference=inference, source=fingerprint)
    (context / 'container-policy.json').write_text(json.dumps(policy, sort_keys=True) + '\n')
    clip = PKG / 'docker/_weights/clip/ViT-B-32.pt'
    clip.parent.mkdir(parents=True, exist_ok=True)
    expected = json.loads(PROFILES.read_text())['clip_source']
    native_clip = BUILD / 'data/ultralytics/weights/clip/ViT-B-32.pt'
    if not clip.exists() and native_clip.exists() and file_hash(native_clip) == expected['sha256']:
        shutil.copy2(native_clip, clip)
    if not clip.exists() or file_hash(clip) != expected['sha256']:
        partial = clip.with_suffix('.part')
        run(['curl', '-fL', '-C', '-', '--retry', '3', '--retry-all-errors',
             '--connect-timeout', '20', '-o', str(partial), expected['url']])
        if file_hash(partial) != expected['sha256']:
            partial.unlink()
            raise RuntimeError('CLIP checkpoint checksum mismatch')
        os.replace(partial, clip)
    models = context / 'models'
    models.mkdir()
    for name in ('yolov8l-world.pt', 'mobile_sam.pt', 'open_clip_pytorch_model.bin'):
        shutil.copy2(PKG / 'docker/_weights' / name, models / name)
    shutil.copy2(clip, models / 'ViT-B-32.pt')
    sdk = profile['docker']
    (context / 'tensorrt-packages.txt').write_text('\n'.join(sdk['tensorrt_packages']))
    key = context / 'tensorrt-key.asc'
    key.write_text('')
    suite = sdk['tensorrt_suite']
    (context / 'tensorrt-source.list').write_text(
        f'deb [signed-by=/usr/share/keyrings/scene-jetson.asc] https://repo.download.nvidia.com/jetson/common {suite} main\n'
        if suite else '')
    if suite and inference == 'tensorrt':
        stage_download(json.loads(PROFILES.read_text())['jetson_key'], key)
    app = context / 'scene'
    app.mkdir()
    ignored = shutil.ignore_patterns('__pycache__', '*.pyc', '.git', '.env*')
    for directory in ('scene_service', 'scripts'):
        shutil.copytree(PKG / directory, app / directory, ignore=ignored)
    shutil.copy2(PKG / 'package_manifest.jetson-docker.yaml', app / 'package_manifest.yaml')
    api = Path(run(['rbnx', 'path', 'robonix-api'], capture=True).stdout.strip())
    shutil.copytree(api / 'robonix_api', context / 'api/robonix_api', ignore=ignored)
    runtime = Path(run(['rbnx', 'path', 'runtime-proto'], capture=True).stdout.strip())
    shutil.copytree(runtime, app / 'rbnx-build/runtime-proto', ignore=ignored)
    shutil.copytree(BUILD / 'proto-staging', app / 'rbnx-build/proto-staging', ignore=ignored)
    for directory in ('robonix_mcp_types', 'ros2_idl/src'):
        shutil.copytree(BUILD / 'codegen' / directory, app / 'rbnx-build/codegen' / directory, ignore=ignored)
    (app / 'rbnx-build/codegen/scene_proto_gen').mkdir()
    return context


def build_image(profile, inference, fingerprint, candidate):
    base = os.environ.get('ROBONIX_SCENE_ROS_BASE_IMAGE') or profile['docker']['base']
    try:
        base_id = inspect(base)['Id']
    except subprocess.CalledProcessError:
        run(['docker', 'pull', base])
        base_id = inspect(base)['Id']
    alias = f"robonix-scene-base:{profile['name']}-{base_id.rsplit(':', 1)[-1][:12]}"
    run(['docker', 'tag', base, alias])
    base = alias
    args = ['docker', 'build', '--network=host', '--pull=false',
            '--build-arg', f'ROS_BASE_IMAGE={base}', '--build-arg', f"ROS_DISTRO={profile['ros']}",
            '--build-arg', f"CUDA_APT_SUFFIX={profile['cuda'].replace('.', '-')}",
            '--build-arg', f'SCENE_CG_INFERENCE={inference}', '--build-arg', f'SCENE_SOURCE_SHA={fingerprint}',
            '--build-arg', f"SCENE_L4T_PREFIX={profile['l4t_prefix']}",
            '--build-arg', f'SCENE_BASE_PIN={base_id}', '--build-arg', 'BUILDKIT_INLINE_CACHE=1', '-f', str(PKG / 'docker/Dockerfile.jetson'),
            '-t', candidate]
    proxy = os.environ.get('RBNX_BUILD_PROXY', '0').lower()
    if proxy not in ('0', 'false', 'no', '1', 'true', 'yes', 'auto'):
        raise RuntimeError('RBNX_BUILD_PROXY must be 0 or 1')
    for name in ('HTTP_PROXY', 'HTTPS_PROXY', 'NO_PROXY'):
        value = (os.environ.get(name) or os.environ.get(name.lower(), '')) if proxy in ('1', 'true', 'yes', 'auto') else ''
        args += ['--build-arg', f'{name}={value}']
    if os.environ.get('RBNX_BUILD_CLEAN') == '1':
        args.append('--no-cache')
    args.append(str(PKG / 'docker'))
    run(args)


def build(profile: dict):
    inference = inference_backend()
    validate_config()
    gpu_args()  # Fail before expensive build work or touching the active image.
    fingerprint = source_hash()
    stage(profile, inference, fingerprint)
    candidate = 'robonix-scene-candidate:' + uuid.uuid4().hex[:16]
    marker = BUILD / 'docker-env.json'
    previous = json.loads(marker.read_text()) if marker.exists() else None
    cache = None
    try:
        build_image(profile, inference, fingerprint, candidate)
        image = inspect(candidate)['Id']
        hardware = check_gpu(image, 'torch', quick=True)
        dependencies = runtime_key(image)
        key = cache_key(dependencies, hardware)
        cache = BUILD / 'docker-models' / key
        cache.mkdir(parents=True, exist_ok=True)
        if inference == 'tensorrt':
            image_run(image, ['python3', '/scene/scripts/prepare_models.py'], inference=inference, cache=cache)
        assert check_gpu(image, inference, cache=cache) == hardware, 'GPU changed during image validation'
        tag = os.environ.get('ROBONIX_SCENE_IMAGE') or f"robonix-scene:{profile['name']}-{inference}"
        if previous and previous['image'] != image:
            run(['docker', 'tag', previous['image'], f"robonix-scene-rollback:{profile['name']}-{previous['inference']}"])
        run(['docker', 'tag', image, tag])
        selected = dict(image=image, tag=tag, source=fingerprint, l4t_prefix=profile['l4t_prefix'],
                        inference=inference, hardware=hardware, runtime=dependencies, cache=key)
        if previous and previous['image'] != image:
            selected['previous'] = {k: v for k, v in previous.items() if k != 'previous'}
        elif previous:
            selected['previous'] = previous.get('previous')
        pending = BUILD / '.docker-env.json.tmp'
        pending.write_text(json.dumps(selected, indent=2) + '\n')
        os.replace(pending, marker)
        keep = {key, (selected.get('previous') or {}).get('cache')}
        for path in (BUILD / 'docker-models').iterdir():
            if path.is_dir() and re.fullmatch('[0-9a-f]{32}', path.name) and path.name not in keep:
                shutil.rmtree(path)
        print(f'[scene/docker] verified local image selected: {image}', flush=True)
    finally:
        subprocess.run(['docker', 'image', 'rm', candidate], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if cache is not None and cache.name not in {p.get('cache') for p in (previous or {}, (previous or {}).get('previous') or {})}:
            if not marker.exists() or json.loads(marker.read_text())['cache'] != cache.name:
                shutil.rmtree(cache, ignore_errors=True)


def check(host: dict):
    validate_config()
    if not (BUILD / 'docker-env.json').exists():
        raise RuntimeError('No verified Docker image; run the Scene jetson-docker build first')
    selected = json.loads((BUILD / 'docker-env.json').read_text())
    if (not host['l4t'].startswith(selected['l4t_prefix'])
            or selected['inference'] != inference_backend()
            or selected['source'] != source_hash()):
        raise RuntimeError('Docker environment changed; rerun the Scene Docker build')
    image = os.environ.get('ROBONIX_SCENE_IMAGE') or selected['image']
    details = inspect(image)
    labels = details['Config'].get('Labels') or {}
    if (details['Id'] != selected['image'] or labels.get('io.robonix.scene.packaged') != '1'
            or labels.get('io.robonix.scene.source') != selected['source']):
        raise RuntimeError('Selected image differs from the verified Docker build')
    cache = BUILD / 'docker-models' / selected['cache']
    hardware = check_gpu(selected['image'], selected['inference'], quick=True)
    if selected['inference'] == 'tensorrt' and hardware != selected['hardware']:
        raise RuntimeError('GPU hardware differs from the verified Docker build; rebuild on this device')
    if selected['inference'] == 'tensorrt':
        image_run(selected['image'], ['python3', '/scene/scripts/prepare_models.py', '--check'],
                  inference=selected['inference'], cache=cache)
    script = BUILD / 'docker-runtime.sh'
    script.write_text('IMG=' + shlex.quote(selected['image']) + '\n' +
                      'SCENE_DOCKER_CACHE=' + shlex.quote(str(cache)) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--plan', action='store_true')
    modes.add_argument('--build', action='store_true')
    modes.add_argument('--check', action='store_true')
    args = parser.parse_args()
    host = detect()
    if args.plan:
        print(json.dumps(select_profile(host), indent=2))
        return
    if args.check:
        check(host)
        return
    BUILD.mkdir(parents=True, exist_ok=True)
    with (BUILD / '.docker-build.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        profile = select_profile(host)
        if os.environ.get('ROBONIX_SCENE_ROS_DISTRO', profile['ros']) != profile['ros']:
            raise RuntimeError(f"The detected Jetson profile requires ROS {profile['ros']}")
        build(profile)


if __name__ == '__main__':
    try:
        main()
    except subprocess.CalledProcessError as exc:
        raise SystemExit(f'[scene/docker] exit {exc.returncode}: {format_command(exc.cmd)}') from exc
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        raise SystemExit(f'[scene/docker] {exc}') from exc
