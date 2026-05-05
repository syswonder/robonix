#!/usr/bin/env python3
"""End-to-end integration test: Atlas → speech_service → audio_driver.

Prerequisites:
  - Atlas running on localhost:50051
  - speech_service proto stubs generated (proto_gen/)
  - robonix_runtime proto stubs generated (proto_gen/)

Run:
    export PYTHONPATH=$(pwd)/proto_gen:$(pwd):${PYTHONPATH:-}
    SPEECH_CI_MODE=1 python test_integration.py
"""
import os
import sys
import time
import socket
import logging
import threading

logging.basicConfig(level=logging.INFO, format="[test] %(levelname)s %(message)s")
log = logging.getLogger(__name__)

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "proto_gen"))

import grpc
import robonix_runtime_pb2 as rpb
import robonix_runtime_pb2_grpc as rpb_grpc
import speech_service_pb2 as spb
import speech_service_pb2_grpc as spb_grpc

ATLAS_ADDR = os.environ.get("ROBONIX_ATLAS", "localhost:50051")

# ── Helpers ──────────────────────────────────────────────────────────────

def check_port(host, port, timeout=3):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        s.close()
        return True
    except Exception:
        return False


def atlas_stub():
    channel = grpc.insecure_channel(ATLAS_ADDR)
    return rpb_grpc.RobonixRuntimeStub(channel), channel


# ── Test 1: Atlas is reachable ──────────────────────────────────────────

def test_atlas_alive():
    log.info("Test 1: Atlas alive check")
    stub, ch = atlas_stub()
    try:
        # QueryNodes with empty filter should work even with nothing registered
        resp = stub.QueryNodes(rpb.QueryNodesRequest())
        log.info("  Atlas responded: %d nodes registered", len(resp.nodes))
        ch.close()
        return True
    except grpc.RpcError as e:
        log.error("  Atlas NOT reachable: %s", e)
        ch.close()
        return False


# ── Test 2: Register speech_service with Atlas ─────────────────────────

def test_register_speech():
    log.info("Test 2: Register speech_service with Atlas")
    stub, ch = atlas_stub()

    node_id = "com.robonix.system.speech"
    contracts = [
        ("robonix/service/speech/asr",        "asr",        "rpc"),
        ("robonix/service/speech/asr_stream",  "asr_stream", "rpc_bidirectional_stream"),
        ("robonix/service/speech/tts",         "tts",         "rpc"),
        ("robonix/service/speech/tts_stream",  "tts_stream",  "rpc_server_stream"),
        ("robonix/service/speech/dialog",      "dialog",      "rpc_server_stream"),
    ]

    try:
        stub.RegisterNode(rpb.RegisterNodeRequest(
            node_id=node_id,
            namespace="robonix/service/speech",
            kind="service",
        ))
        log.info("  RegisterNode OK: %s", node_id)

        for cid, name, mode in contracts:
            import json
            meta = json.dumps({"transport": "grpc", "contract": {"idl_type": "protobuf", "mode": mode}})
            resp = stub.DeclareInterface(rpb.DeclareInterfaceRequest(
                node_id=node_id, name=name,
                supported_transports=["grpc"],
                metadata_json=meta, listen_port=50060,
                contract_id=cid,
            ))
            log.info("  DeclareInterface OK: %s → endpoint=%s", cid, resp.allocated_endpoint)

        ch.close()
        return True
    except grpc.RpcError as e:
        log.error("  Registration FAILED: %s (%s)", e.code(), e.details())
        ch.close()
        return False


# ── Test 3: Register audio_driver with Atlas ───────────────────────────

def test_register_audio():
    log.info("Test 3: Register audio_driver with Atlas")
    stub, ch = atlas_stub()

    node_id = "com.robonix.primitive.audio"

    try:
        stub.RegisterNode(rpb.RegisterNodeRequest(
            node_id=node_id,
            namespace="robonix/primitive/audio",
            kind="primitive",
        ))
        log.info("  RegisterNode OK: %s", node_id)

        for cid, name, mode, port in [
            ("robonix/primitive/audio/mic",     "mic",     "server_stream", 50070),
            ("robonix/primitive/audio/speaker", "speaker", "client_stream", 50071),
        ]:
            import json
            meta = json.dumps({"transport": "grpc", "contract": {"idl_type": "protobuf", "mode": mode}})
            resp = stub.DeclareInterface(rpb.DeclareInterfaceRequest(
                node_id=node_id, name=name,
                supported_transports=["grpc"],
                metadata_json=meta, listen_port=port,
                contract_id=cid,
            ))
            log.info("  DeclareInterface OK: %s → endpoint=%s", cid, resp.allocated_endpoint)

        ch.close()
        return True
    except grpc.RpcError as e:
        log.error("  Registration FAILED: %s (%s)", e.code(), e.details())
        ch.close()
        return False


# ── Test 4: Discover speech_service via QueryNodes ─────────────────────

def test_discover_speech():
    log.info("Test 4: Discover speech_service via QueryNodes")
    stub, ch = atlas_stub()

    results = {}
    for cid in [
        "robonix/service/speech/asr",
        "robonix/service/speech/tts",
        "robonix/service/speech/dialog",
        "robonix/primitive/audio/mic",
        "robonix/primitive/audio/speaker",
    ]:
        try:
            resp = stub.QueryNodes(rpb.QueryNodesRequest(contract_id=cid))
            if resp.nodes:
                n = resp.nodes[0]
                ifaces = [i.contract_id for i in n.interfaces]
                log.info("  FOUND: %s → node=%s interfaces=%s", cid, n.node_id, ifaces)
                results[cid] = True
            else:
                log.error("  NOT FOUND: %s", cid)
                results[cid] = False
        except grpc.RpcError as e:
            log.error("  QueryNodes FAILED for %s: %s", cid, e)
            results[cid] = False

    ch.close()
    return all(results.values())


# ── Test 5: Start speech_service and call RPCs ─────────────────────────

def test_speech_service_rpc():
    log.info("Test 5: Start speech_service (CI mode) and call RPCs")

    # Start speech_service in CI mode in background
    env = os.environ.copy()
    env["SPEECH_CI_MODE"] = "1"
    env["SPEECH_PORT"] = "50060"
    env["PYTHONPATH"] = f"{os.path.dirname(__file__)}:{os.path.dirname(__file__)}/proto_gen:{env.get('PYTHONPATH', '')}"

    import subprocess
    proc = subprocess.Popen(
        [sys.executable, "-m", "speech_service.service"],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    log.info("  speech_service started (PID=%d)", proc.pid)

    # Wait for it to be ready
    for _ in range(20):
        if check_port("127.0.0.1", 50060):
            break
        time.sleep(0.5)
    else:
        log.error("  speech_service did not start on port 50060")
        proc.terminate()
        return False

    log.info("  speech_service listening on port 50060")

    # Call Recognize
    channel = grpc.insecure_channel("localhost:50060")
    asr_stub = spb_grpc.SpeechAsrStub(channel)

    resp = asr_stub.Recognize(spb.RecognizeRequest(
        audio_data=b"\x00" * 3200,
        encoding="pcm_s16le",
        sample_rate_hz=16000,
        language="zh",
    ))
    log.info("  Recognize: text=%r confidence=%.1f error=%r", resp.text, resp.confidence, resp.error)
    asr_ok = resp.error == "" and len(resp.text) > 0

    # Call Synthesize
    tts_stub = spb_grpc.SpeechTtsStub(channel)
    resp = tts_stub.Synthesize(spb.SynthesizeRequest(
        text="你好世界",
        language="zh",
    ))
    log.info("  Synthesize: audio=%d bytes encoding=%s error=%r", len(resp.audio_data), resp.encoding, resp.error)
    tts_ok = resp.error == "" and len(resp.audio_data) > 0

    channel.close()
    proc.terminate()
    proc.wait(timeout=5)
    return asr_ok and tts_ok


# ── Main ────────────────────────────────────────────────────────────────

def main():
    log.info("=" * 60)
    log.info("Robonix speech integration test suite")
    log.info("=" * 60)

    results = {}

    results["1_atlas_alive"] = test_atlas_alive()
    results["2_register_speech"] = test_register_speech()
    results["3_register_audio"] = test_register_audio()
    results["4_discover"] = test_discover_speech()
    results["5_rpc"] = test_speech_service_rpc()

    log.info("=" * 60)
    log.info("RESULTS:")
    all_pass = True
    for name, ok in results.items():
        status = "PASS" if ok else "FAIL"
        log.info("  %s: %s", name, status)
        if not ok:
            all_pass = False
    log.info("=" * 60)

    if all_pass:
        log.info("ALL TESTS PASSED")
    else:
        log.error("SOME TESTS FAILED")
        sys.exit(1)


if __name__ == "__main__":
    main()
