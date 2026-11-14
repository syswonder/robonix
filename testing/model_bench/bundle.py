# SPDX-License-Identifier: MulanPSL-2.0
"""Build a scrubbed, reproducible submission ZIP from results and selected logs."""

import argparse
import gzip
import ipaddress
import json
import os
import re
import socket
import stat
import tempfile
import zipfile
from pathlib import Path, PurePosixPath


REDACTED = "[REDACTED]"
_CREDENTIAL = r"(?:[\w-]*secret[\w-]*|[\w-]+[_-](?:key|token)|(?:[\w-]+[_-])?(?:api[_-]?key|token|password|authorization))"
_PRIVATE_FIELD = rf"(?:{_CREDENTIAL}|host|hostname|server|endpoint)"
_ENV_NAME = r"[A-Z_][A-Z0-9_]*"
_ASSIGNMENT_KEY = rf"(?i:{_PRIVATE_FIELD})|{_ENV_NAME}"
_ASSIGNMENT = re.compile(
    rf"([\"']?\b(?:{_ASSIGNMENT_KEY})\b[\"']?[ \t]*(?:(?P<colon>:)|=)[ \t]*|"
    rf"[\"']?\b(?i:{_CREDENTIAL})\b[\"']?[ \t]+)"
    r'''(?P<value>"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|(?(colon)[^\r\n,;}]+|(?:\\[^\r\n]|[^\s,;}\\])+))'''
)
_ESCAPED_ASSIGNMENT = re.compile(
    rf'(?P<prefix>(?<!\\)(?P<escape>\\+)"\b(?:{_ASSIGNMENT_KEY})\b(?P=escape)"[ \t]*:[ \t]*(?P=escape)")'
    r'.*?(?P<close>(?<!\\)(?P=escape)")'
)
_AUTHORIZATION = re.compile(r"(?i)(\bauthorization[ \t]*[:=][ \t]*)(?![\"'])[^\r\n,;}]+")
_BEARER = re.compile(r"(?i)\b(?:bearer|basic)\s+[a-z0-9+/_.=:-]+")
_KEY = re.compile(r"\b(?:sk|rk|pk)-(?:proj-|ant-)?[a-zA-Z0-9_-]{8,}\b|\bAKIA[A-Z0-9]{16}\b")
_HOME = re.compile(r"(?:/(?:Users|home)/[^\s/\"'<>]+|/root\b|[A-Za-z]:\\+Users\\+[^\s\\\"'<>]+|~[\w.-]*)(?:[/\\][^\s\"'<>]*)?")
_HOST = re.compile(r"(?i)(?<![\w.-])(?:(?:[a-z0-9_-]+\.)+(?:local|internal|lan|tailscale|ts\.net)|localhost)\b")
_URL_HOST = re.compile(r"(?i)(\b[a-z][a-z0-9+.-]*:(?:\\?/){2})(?:[^/?#\s\"'<>@]+@)?(?:\[[^\]\s]+\]|[^/?#\s\"'<>:]+)")
_HOST_PORT = re.compile(r"(?i)(?<![\w.-])(?:\[[0-9a-f:.%]+\]|[a-z_][a-z0-9_.-]*|\d{1,3}(?:\.\d{1,3}){3})(:\d+)\b")
_IP = re.compile(r"(?<![\w:])(?:[0-9a-fA-F]{0,4}:){2,}[0-9a-fA-F:.]*(?:%[\w.-]+)?|(?<![\w.])(?:\d{1,3}\.){3}\d{1,3}(?![\w.])")
_IMAGE = re.compile(r"data:image\\*/[^\s\"'<>]*", re.I)
_IMAGE_FIELD = re.compile(r"(?:[\w-]+[_-])?image[_-]base64", re.I)
_IMAGE_ASSIGNMENT = re.compile(r'''\b(?:[\w-]+[_-])?image[_-]base64["']?\s*[:=]''', re.I)
_CAMERA_FIELDS = tuple(re.compile(rf'''\b{key}\b["']?\s*[:=]''', re.I)
                       for key in ("width", "height", "encoding", "data"))
_ANSI = re.compile(r"\x1b(?:\[[0-?]*[ -/]*[@-~]|\][^\x07]*(?:\x07|\x1b\\))")
_BINARY_EXTENSIONS = {".png", ".jpg", ".jpeg", ".gif", ".webp", ".bmp", ".tiff", ".ico", ".pdf", ".zip", ".gz", ".mp4", ".bin"}


def _secrets(secrets):
    values = {str(value) for value in secrets if value}
    values.update(value for key, value in os.environ.items() if value and (
        key.upper().endswith(("_KEY", "_TOKEN")) or "SECRET" in key.upper()))
    # Logs can contain JSON inside JSON, including escaped quotes in credentials.
    for original in tuple(values):
        for ascii_only in (False, True):
            value = original
            for _ in range(2):
                value = json.dumps(value, ensure_ascii=ascii_only)[1:-1]
                values.add(value)
    return sorted(values, key=lambda value: (-len(value), value))


def _private_ip(match):
    try:
        address = ipaddress.ip_address(match.group().split("%", 1)[0])
    except ValueError:
        return match.group()
    return "[PRIVATE_IP]" if not address.is_global else match.group()


def _text(text, secrets):
    text = _ANSI.sub("", text)
    text = "".join(line for line in text.splitlines(keepends=True)
                   if not (_IMAGE_ASSIGNMENT.search(line)
                           or all(field.search(line) for field in _CAMERA_FIELDS)))
    for value in secrets:
        text = text.replace(value, REDACTED)
        if "/" in value:
            text = re.sub(re.escape(value).replace("/", r"\\*/"), lambda _: REDACTED, text)
    text = _IMAGE.sub("[IMAGE]", text)
    text = _AUTHORIZATION.sub(lambda match: match[1] + REDACTED, text)
    text = _BEARER.sub(REDACTED, text)
    text = _ESCAPED_ASSIGNMENT.sub(lambda match: match["prefix"] + REDACTED + match["close"], text)
    text = _ASSIGNMENT.sub(lambda match: match[1] + (
        match["value"][0] + REDACTED + match["value"][0] if match["value"][0] in "\"'" else REDACTED), text)
    text = _KEY.sub(REDACTED, text)
    text = _HOME.sub("[HOME]", text)
    text = _URL_HOST.sub(lambda match: match[1] + "[PRIVATE_HOST]", text)
    text = _IP.sub(_private_ip, text)
    text = _HOST_PORT.sub(lambda match: "[PRIVATE_HOST]" + match[1], text)
    text = _HOST.sub("[PRIVATE_HOST]", text)
    hostname = socket.gethostname()
    if hostname:
        text = re.sub(r"(?<![\w.-])" + re.escape(hostname) + r"(?![\w.-])", "[PRIVATE_HOST]", text, flags=re.I)
    return text


def scrub_text(text, secrets=()):
    """Remove private data from plain text while retaining ordinary log lines."""
    return _text(text, _secrets(secrets))


def scrub(value, secrets=()):
    """Scrub JSON values independently so a log line cannot erase its record."""
    known = _secrets(secrets)

    def visit(item):
        if isinstance(item, dict):
            image = (any(str(item.get(key, "")).lower().startswith("image/")
                         for key in ("media_type", "mime_type", "content_type"))
                     or item.get("type") in ("image", "image_url", "input_image")
                     or all(key in item for key in ("width", "height", "encoding", "data")))
            return {(_text(key, known) or REDACTED): (
                "[IMAGE]" if (_IMAGE_FIELD.fullmatch(key)
                    or image and key.lower() in ("data", "base64", "url", "image_url", "source"))
                else REDACTED if re.fullmatch(_ASSIGNMENT_KEY, key) else visit(value)
            ) for key, value in item.items()}
        if isinstance(item, list):
            clean = []
            index = 0
            while index < len(item):
                value = item[index]
                clean.append(visit(value))
                if (isinstance(value, str) and value.startswith("-")
                        and re.fullmatch(_PRIVATE_FIELD, value.lstrip("-"), re.I)
                        and index + 1 < len(item) and isinstance(item[index + 1], str)
                        and not item[index + 1].startswith("-")):
                    clean.append(REDACTED)
                    index += 1
                index += 1
            return clean
        if isinstance(item, str):
            try:
                decoded = json.loads(item)
            except json.JSONDecodeError:
                return _text(item, known)
            if isinstance(decoded, (dict, list, str)):
                clean = visit(decoded)
                if clean != decoded:
                    return json.dumps(clean, ensure_ascii=False, sort_keys=True)
            return _text(item, known)
        return item

    return visit(value)


def _read(path):
    path = Path(path).absolute()
    if path.is_symlink():
        raise ValueError("symlink inputs are not supported")
    path = path.parent.resolve() / path.name
    descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, "O_NOFOLLOW", 0))
    with os.fdopen(descriptor, "rb") as source:
        if not stat.S_ISREG(os.fstat(source.fileno()).st_mode):
            raise ValueError("bundle inputs must be regular files")
        return source.read()


def _member(name):
    path = PurePosixPath(name)
    if (not name or path.is_absolute() or "\\" in name or ":" in name
            or any(part in {"", ".", ".."} for part in name.split("/"))
            or any(ord(char) < 32 for char in name)):
        raise ValueError("invalid bundle member name")
    return path.as_posix()


def _log_bytes(path):
    data = _read(path)
    path = Path(path)
    if path.suffix.lower() == ".gz":
        data = gzip.decompress(data)
        path = path.with_suffix("")
    if (path.suffix.lower() in _BINARY_EXTENSIONS or b"\x00" in data
            or data.startswith((b"\x89PNG", b"\xff\xd8", b"GIF8", b"RIFF", b"BM", b"II*", b"MM\x00*"))):
        return None
    return data


def _scrub_log(data, secrets, jsonl=False):
    text = _ANSI.sub("", data.decode("utf-8", errors="replace"))
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        lines = []
        for line in text.splitlines(keepends=True):
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                lines.append(scrub_text(line, secrets))
            else:
                lines.append(json.dumps(scrub(value, secrets), ensure_ascii=False, sort_keys=True) + "\n")
        return "".join(lines).encode()
    return (json.dumps(scrub(value, secrets), ensure_ascii=False, sort_keys=True,
                       indent=None if jsonl else 2) + "\n").encode()


def write_bundle(destination: Path, results: list[Path], logs: dict[str, Path], secrets=()) -> None:
    """Write only scrubbed JSON and text logs, using neutral result filenames."""
    secrets = tuple(secrets)
    members = {}
    for index, path in enumerate(sorted(map(Path, results), key=str), 1):
        value = scrub(json.loads(_read(path)), secrets)
        members[f"result-{index:03d}.json"] = (json.dumps(
            value, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode()
    for index, (name, path) in enumerate(sorted(logs.items()), 1):
        name = _member(name)
        data = _log_bytes(path)
        if data is None:
            continue
        if Path(path).suffix.lower() == ".gz" and name.lower().endswith(".gz"):
            name = name[:-3]
        if scrub_text(name, secrets) != name:
            name = f"log-{index:03d}.txt"
        name = "logs/" + name
        if name in members:
            raise ValueError("duplicate bundle member name")
        members[name] = _scrub_log(data, secrets, name.lower().endswith(".jsonl"))
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_symlink():
        raise ValueError("symlink destinations are not supported")
    destination = destination.parent.resolve() / destination.name
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as stream:
            temporary = Path(stream.name)
            with zipfile.ZipFile(stream, "w", compression=zipfile.ZIP_STORED) as archive:
                for name, data in sorted(members.items()):
                    info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
                    info.create_system = 3
                    info.external_attr = (stat.S_IFREG | 0o644) << 16
                    archive.writestr(info, data)
        os.replace(temporary, destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+", type=Path)
    parser.add_argument("--logs", action="append", type=Path, default=[])
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--env-file", action="append", type=Path, default=[])
    args = parser.parse_args()
    secrets = []
    for path in args.env_file:
        for line in _read(path).decode("utf-8").splitlines():
            key, separator, value = line.removeprefix("export ").partition("=")
            if separator and (key.strip().upper().endswith(("_KEY", "_TOKEN")) or "SECRET" in key.upper()):
                secrets.append(value.strip().strip("\"'"))
    logs = {}
    for index, directory in enumerate(args.logs, 1):
        if directory.is_symlink() or not directory.is_dir():
            raise ValueError("log inputs must be real directories")
        for root, directories, files in os.walk(directory, followlinks=False):
            if any((Path(root) / name).is_symlink() for name in directories):
                raise ValueError("symlink inputs are not supported")
            for name in files:
                path = Path(root) / name
                logs[f"run-{index:03d}/{path.relative_to(directory).as_posix()}"] = path
    write_bundle(args.out, args.files, logs, secrets)


if __name__ == "__main__":
    main()
