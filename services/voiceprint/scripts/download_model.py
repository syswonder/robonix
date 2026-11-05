# SPDX-License-Identifier: MulanPSL-2.0
"""Prepare the bundled CAM++ model without publishing partial downloads."""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from urllib.parse import urlsplit

MODEL = '3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx'
SHA256 = 'f682b514c05d947ee3fa91cd6ec6c5c7543479a128373fa29b1faedccd21fd11'


def matches(path: Path) -> bool:
    return path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == SHA256


def download_model(destination: Path) -> None:
    if matches(destination):
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    hf_path = f'csukuangfj/speaker-embedding-models/resolve/main/{MODEL}'
    github = f'https://github.com/k2-fsa/sherpa-onnx/releases/download/speaker-recongition-models/{MODEL}'
    hf = os.environ.get('HF_ENDPOINT') or 'https://hf-mirror.com'
    mirror = os.environ.get('RBNX_GH_MIRROR', 'https://ghfast.top/')
    urls = [f'{hf.rstrip("/")}/{hf_path}', f'https://huggingface.co/{hf_path}',
            f'{mirror.rstrip("/")}/{github}' if mirror else github, github]
    for url in dict.fromkeys(urls):
        with tempfile.NamedTemporaryFile(dir=destination.parent, prefix='.campplus-', delete=False) as file:
            pending = Path(file.name)
        try:
            subprocess.run([
                'curl', '-fL', '--retry', '1', '--retry-max-time', '120',
                '--connect-timeout', '15', '--max-time', '180',
                '--speed-limit', '4096', '--speed-time', '30', '-o', str(pending), url,
            ], check=True, timeout=240)
            if not matches(pending):
                raise ValueError('CAM++ checksum mismatch')
            pending.chmod(0o644)
            os.replace(pending, destination)
            return
        except (OSError, subprocess.SubprocessError, ValueError) as error:
            print(f'[voiceprint/build] {urlsplit(url).hostname}: {type(error).__name__}; trying next source',
                  file=sys.stderr, flush=True)
        finally:
            pending.unlink(missing_ok=True)
    raise RuntimeError('Could not download the verified CAM++ model from any source')


if __name__ == '__main__':
    download_model(Path(sys.argv[1]))
