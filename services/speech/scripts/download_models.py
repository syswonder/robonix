# SPDX-License-Identifier: MulanPSL-2.0
"""Download verified Speech models and publish only complete runtime files."""
from __future__ import annotations

import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile

MODELS = {
    'kws': ('sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20', 'kws-models',
            '68447f4fbc67e70eee3a93961f36e81e98f47aef73ce7e7ca00885c6cd3616a6',
            ('tokens.txt', 'en.phone',
             'encoder-epoch-13-avg-2-chunk-8-left-64.int8.onnx',
             'decoder-epoch-13-avg-2-chunk-8-left-64.onnx',
             'joiner-epoch-13-avg-2-chunk-8-left-64.int8.onnx')),
    'asr': ('sherpa-onnx-streaming-paraformer-bilingual-zh-en', 'asr-models',
            '5462a1fce42693deae572af1e8c4687124b12aa85fe61ff4d3168bb5280e205f',
            ('tokens.txt', 'encoder.int8.onnx', 'decoder.int8.onnx')),
}


def checksum(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def verified(destination: Path, digest: str, required: tuple[str, ...]) -> bool:
    try:
        saved = json.loads((destination / '.verified.json').read_text())
        return saved['archive_sha256'] == digest and all(
            checksum(destination / name) == saved['files'][name] for name in required)
    except (OSError, ValueError, KeyError, TypeError):
        return False


def prepare(root: Path, kind: str) -> None:
    name, tag, digest, required = MODELS[kind]
    root.mkdir(parents=True, exist_ok=True)
    with (root / f'.{name}.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        destination = root / name
        if verified(destination, digest, required):
            return
        official = f'https://github.com/k2-fsa/sherpa-onnx/releases/download/{tag}/{name}.tar.bz2'
        mirror = os.environ.get('RBNX_GH_MIRROR', 'https://ghfast.top/')
        primary = f'{mirror.rstrip("/")}/{official}' if mirror else official
        if kind == 'kws':
            primary = os.environ.get('SHERPA_KWS_MODEL_URL') or primary
        with tempfile.TemporaryDirectory(prefix=f'.{name}-', dir=root) as tmp:
            pending = Path(tmp)
            archive = pending / 'model.tar.bz2'
            for url in dict.fromkeys((primary, official)):
                try:
                    subprocess.run([
                        'curl', '-fL', '--retry', '1', '--connect-timeout', '15',
                        '--max-time', '600', '--speed-limit', '4096', '--speed-time', '30',
                        '-o', str(archive), url,
                    ], check=True, timeout=1250)
                    if checksum(archive) != digest:
                        raise ValueError(f'{kind} archive checksum mismatch')
                    break
                except (OSError, ValueError, subprocess.SubprocessError) as error:
                    print(f'[speech/build] {kind}: {type(error).__name__}; trying next source',
                          file=sys.stderr, flush=True)
            else:
                raise RuntimeError(f'Could not download the verified {kind} model')
            staged = pending / name
            staged.mkdir()
            with tarfile.open(archive, 'r|bz2') as bundle:
                for member in bundle:
                    relative = member.name.removeprefix('./')
                    selected = next((file for file in required if relative == f'{name}/{file}'), None)
                    if selected is None:
                        continue
                    if not member.isfile() or (staged / selected).exists():
                        raise ValueError(f'Invalid model archive entry: {relative}')
                    with bundle.extractfile(member) as source, (staged / selected).open('wb') as target:
                        shutil.copyfileobj(source, target)
            hashes = {file: checksum(staged / file) for file in required}
            (staged / '.verified.json').write_text(json.dumps(
                dict(archive_sha256=digest, files=hashes), sort_keys=True))
            archive.unlink()
            previous = pending / 'previous'
            if destination.exists():
                destination.rename(previous)
            try:
                staged.rename(destination)
            except OSError:
                if previous.exists():
                    previous.rename(destination)
                raise


if __name__ == '__main__':
    prepare(Path(sys.argv[1]), sys.argv[2])
