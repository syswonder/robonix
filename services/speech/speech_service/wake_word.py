from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
import sherpa_onnx


def prepare_keywords_file(
    model_dir: Path,
    output_dir: Path,
    wake_words: Sequence[str],
    *,
    boost: float = 2.0,
    threshold: float = 0.45,
) -> Path:
    """Compile configured phrases into sherpa-onnx tokens without network I/O."""
    phrases = []
    for raw in wake_words:
        phrase = str(raw).strip()
        if not phrase:
            continue
        if any(char in phrase for char in "\r\n:@#"):
            raise ValueError(f"invalid wake phrase: {phrase!r}")
        if phrase not in phrases:
            phrases.append(phrase)
    if not phrases:
        raise ValueError("wake_words must contain at least one non-empty phrase")
    if boost <= 0:
        raise ValueError("wake_word_boost must be positive")
    if not 0 < threshold <= 1:
        raise ValueError("wake_word_threshold must be in (0, 1]")

    output_dir.mkdir(parents=True, exist_ok=True)
    raw_file = output_dir / "keywords_raw.txt"
    token_file = output_dir / "keywords.txt"
    raw_file.write_text(
        "".join(f"{phrase} :{boost:g} #{threshold:g} @{phrase}\n" for phrase in phrases),
        encoding="utf-8",
    )

    cli = Path(sys.executable).with_name("sherpa-onnx-cli")
    if not cli.is_file():
        resolved = shutil.which("sherpa-onnx-cli")
        if not resolved:
            raise RuntimeError("sherpa-onnx-cli is unavailable; rebuild the Speech package")
        cli = Path(resolved)
    subprocess.run(
        [
            str(cli),
            "text2token",
            "--tokens",
            str(model_dir / "tokens.txt"),
            "--tokens-type",
            "phone+ppinyin",
            "--lexicon",
            str(model_dir / "en.phone"),
            str(raw_file),
            str(token_file),
        ],
        check=True,
    )
    return token_file


class WakeWordBackend:
    """Streaming open-vocabulary KWS implemented inside the Speech service."""

    def __init__(self, model_dir: Path, keywords_file: Path, num_threads: int = 2) -> None:
        self._spotter = sherpa_onnx.KeywordSpotter(
            tokens=str(model_dir / "tokens.txt"),
            encoder=str(model_dir / "encoder-epoch-13-avg-2-chunk-8-left-64.int8.onnx"),
            decoder=str(model_dir / "decoder-epoch-13-avg-2-chunk-8-left-64.onnx"),
            joiner=str(model_dir / "joiner-epoch-13-avg-2-chunk-8-left-64.int8.onnx"),
            num_threads=num_threads,
            keywords_file=str(keywords_file),
            provider="cpu",
        )

    def detect(self, pcm_chunks: Iterable[bytes]) -> str:
        stream = self._spotter.create_stream()
        for pcm in pcm_chunks:
            if not pcm:
                continue
            samples = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
            stream.accept_waveform(16_000, samples)
            while self._spotter.is_ready(stream):
                self._spotter.decode_stream(stream)
                result = self._spotter.get_result(stream)
                if result:
                    return str(result)
        return ""
