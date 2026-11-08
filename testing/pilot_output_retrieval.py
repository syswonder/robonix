#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Real-model output retrieval test using isolated Atlas, Executor and Pilot.

Synthetic fixtures measure retrieval behavior, not production workload frequency.
Requires built debug binaries, model credentials in environment, and grpcio-tools
for --document. No robot providers are registered. Evidence is never overwritten.
"""

import os, json, subprocess, socket, time, signal, argparse, shutil, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--model", default=os.environ.get("ROBONIX_VLM_MODEL"))
parser.add_argument("--threshold", type=int, default=32768)
parser.add_argument("--lines", type=int, default=1600)
parser.add_argument("--no-rg", action="store_true")
parser.add_argument("--document", action="store_true")
args = parser.parse_args()
OUT = args.output.resolve()
if OUT.exists():
    raise RuntimeError("Refusing to overwrite existing test evidence")
OUT.mkdir(exist_ok=True)
WORK = OUT / "workspace"
WORK.mkdir(exist_ok=True)
secret = "violet-cabinet-72946"
text = (
    "\n".join(
        f"Observation {i}: ordinary marker, no requested calibration value."
        for i in range(args.lines)
    )
    + "\nCalibration target ZX-491: "
    + secret
    + "\n"
)
(WORK / "source-report.txt").write_text(text)
(WORK / "emit-report.sh").write_text(
    '#!/bin/sh\nprintf "invoked\\n" >> invocation-count.txt\ncat source-report.txt\n'
)
(WORK / "emit-report.sh").chmod(0o700)
if (
    not args.model
    or not os.environ.get("ROBONIX_VLM_UPSTREAM")
    or not os.environ.get("ROBONIX_VLM_API_KEY")
):
    raise RuntimeError(
        "Set ROBONIX_VLM_UPSTREAM, ROBONIX_VLM_API_KEY and ROBONIX_VLM_MODEL (or --model)."
    )
env = os.environ.copy()
env.update(
    ROBONIX_WORKSPACE=str(WORK),
    ROBONIX_SESSION_DIR=str(OUT / "sessions"),
    ROBONIX_VLM_MODEL=args.model,
    ROBONIX_VLM_CONTEXT_WINDOW_TOKENS="131072",
    ROBONIX_OUTPUT_INLINE_BYTES=str(args.threshold),
    SCRIBE_FILE_LEVEL="info",
    SCRIBE_CONSOLE_LEVEL="info",
)
if args.no_rg:
    tools = OUT / "fallback-bin"
    tools.mkdir()
    for name in [
        "bash",
        "sh",
        "grep",
        "sed",
        "head",
        "tail",
        "cat",
        "wc",
        "cut",
        "awk",
        "sort",
        "uniq",
        "tr",
        "printf",
        "python3",
    ]:
        located = shutil.which(name)
        if located:
            (tools / name).symlink_to(located)
    env["PATH"] = str(tools)


def port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


ports = [port() for _ in range(3)]
procs = []
logs = []


def start(name, args, p):
    f = (OUT / (name + ".log")).open("w")
    logs.append(f)
    proc = subprocess.Popen(
        [str(ROOT / "target/debug" / name), *args],
        cwd=WORK,
        env=env,
        stdout=f,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    procs.append(proc)
    deadline = time.time() + 30
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(name + " exited at startup")
        try:
            with socket.create_connection(("127.0.0.1", p), timeout=0.2):
                return
        except OSError:
            time.sleep(0.1)
    raise RuntimeError(name + " startup timed out")


report = {
    "model": env["ROBONIX_VLM_MODEL"],
    "threshold_bytes": args.threshold,
    "no_rg": args.no_rg,
    "original_bytes": len(text.encode()),
    "kind": "Synthetic retrieval fixture; real Atlas/Executor/Pilot processes and real model gateway; no robot connected.",
}
try:
    start(
        "robonix-atlas",
        [
            "--listen",
            f"127.0.0.1:{ports[0]}",
            "--capabilities",
            str(ROOT / "capabilities"),
        ],
        ports[0],
    )
    start(
        "robonix-executor",
        ["--atlas", f"127.0.0.1:{ports[0]}", "--listen", f"127.0.0.1:{ports[1]}"],
        ports[1],
    )
    if args.document:
        proto = OUT / "proto"
        proto.mkdir()
        subprocess.run(
            [
                sys.executable,
                "-m",
                "grpc_tools.protoc",
                "-I" + str(ROOT / "system/atlas/proto"),
                "--python_out=" + str(proto),
                "--grpc_python_out=" + str(proto),
                str(ROOT / "system/atlas/proto/atlas.proto"),
            ],
            check=True,
        )
        sys.path.insert(0, str(proto))
        import grpc, atlas_pb2, atlas_pb2_grpc

        channel = grpc.insecure_channel(f"127.0.0.1:{ports[0]}")
        stub = atlas_pb2_grpc.AtlasStub(channel)
        manual = (
            "# Fixture skill manual\n\n"
            + text
            + "\n## Target identity provenance\nOnly use object IDs obtained from the current observation. Never substitute a guessed object ID.\n"
        )
        stub.RegisterSkill(
            atlas_pb2.RegisterRequest(
                id="manual_fixture",
                namespace="robonix/skill/manual_fixture",
                capability_md=manual,
            ),
            timeout=10,
        )
        report["manual_bytes"] = len(manual.encode())
    start(
        "robonix-pilot",
        ["--atlas", f"127.0.0.1:{ports[0]}", "--listen", f"127.0.0.1:{ports[2]}"],
        ports[2],
    )
    prompt = "Use the Executor run_command capability to run ./emit-report.sh exactly once. The command produces a long observation report. Find the calibration value for target ZX-491 in that command's output (search the saved file if the result was retained) and report the exact value and its line number. Search the retained output rather than repeating the command. Do not read the script or source-report.txt to bypass the returned result. Do not modify any files."
    if args.document:
        prompt = "Read the capability manual for provider manual_fixture using read_capability_doc. Find the exact calibration value for target ZX-491 and the rule in Target identity provenance. The complete manual is registered in Atlas; do not guess a local source path. Search the retained result if needed. Report both facts with source line numbers. Do not invoke any robot action."
    with (OUT / "events.jsonl").open("w") as f:
        result = subprocess.run(
            [
                str(ROOT / "target/debug/rbnx"),
                "ask",
                prompt,
                "--server",
                f"127.0.0.1:{ports[0]}",
                "--json",
            ],
            env=env,
            cwd=WORK,
            stdout=f,
            stderr=subprocess.PIPE,
            text=True,
            timeout=300,
        )
    report["client_exit"] = result.returncode
    events = []
    for line in (OUT / "events.jsonl").read_text().splitlines():
        try:
            events.append(json.loads(line))
        except ValueError:
            pass
    report["final_texts"] = [e.get("final_text") for e in events if e.get("final_text")]
    report["answer_contains_target"] = any(secret in t for t in report["final_texts"])
    count = WORK / "invocation-count.txt"
    report["original_command_invocations"] = (
        len(count.read_text().splitlines()) if count.exists() else 0
    )
    report["saved_outputs"] = [
        {"name": p.name, "bytes": p.stat().st_size}
        for p in (WORK / ".robonix/tool-output").glob("*.txt")
    ]
    expected_bytes = (manual if args.document else text).encode()
    report["complete_original_retained"] = any(
        p.read_bytes() == expected_bytes
        for p in (WORK / ".robonix/tool-output").glob("*.txt")
    )
    report["calls"] = [
        call for e in events for call in (e.get("plan") or {}).get("calls", [])
    ]
except Exception as e:
    report["error"] = type(e).__name__ + ": " + str(e)
finally:
    for p in reversed(procs):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGTERM)
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
    for f in logs:
        f.close()
    usage = []
    log = OUT / "robonix-pilot.log"
    if log.exists():
        for line in log.read_text().splitlines():
            if '"event":"vlm_usage"' in line:
                try:
                    usage.append(json.loads(line[line.index("{") :]))
                except ValueError:
                    pass
    report["model_usage"] = {
        "rounds": len(usage),
        "input_tokens": sum(u["input_tokens"] for u in usage),
        "output_tokens": sum(u["output_tokens"] for u in usage),
    }
    (OUT / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))

expected_calls = 0 if args.document else 1
passed = (
    report.get("client_exit") == 0
    and report.get("answer_contains_target")
    and report.get("complete_original_retained")
    and report.get("original_command_invocations") == expected_calls
)
if args.document:
    passed = (
        passed
        and sum(
            c["contract_id"].endswith("/read_capability_doc")
            for c in report.get("calls", [])
        )
        == 1
    )
    passed = passed and any(
        "guessed object" in t.lower() for t in report.get("final_texts", [])
    )
sys.exit(0 if passed else 1)
