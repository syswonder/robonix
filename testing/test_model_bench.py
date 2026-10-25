# SPDX-License-Identifier: MulanPSL-2.0
"""Regression tests for provider-observed model benchmark scoring."""

from __future__ import annotations

import importlib
import json
import tempfile
import unittest
import uuid
import zipfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from testing.model_bench import bench
from testing.model_bench.world import scene


CAMERA = "robonix/primitive/camera/snapshot"
LIDAR = "robonix/primitive/lidar/snapshot"
NAV = "robonix/service/navigation/navigate"
MEMORY = "robonix/service/memory/save"


def plan(contract, provider, args=None, call_id=None):
    return {"contract_id": contract, "provider_id": provider,
            "args_json": json.dumps(args or {}), "call_id": call_id or uuid.uuid4().hex}


def record(contract, provider, args=None, start=1, end=2, ok=True, call_id=None):
    return {"case": "case", "contract": contract, "provider": provider,
            "args": args or {}, "start": start, "end": end, "ok": ok,
            "execution_id": uuid.uuid4().hex, "call_id": call_id}


class ModelBenchTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tempdir.cleanup)
        self.world = Path(self.tempdir.name)
        self.args = SimpleNamespace(rbnx="rbnx", server="unused", timeout=10,
                                    images=str(self.world))

    def run_case(self, case, turns, records=()):
        (self.world / "calls.jsonl").write_text(
            "".join(json.dumps({k: v for k, v in r.items() if k != "call_id"}) + "\n" for r in records)
        )
        results = [{"leaf_result": {"call_id": r["call_id"], "success": r["ok"],
                    "output": json.dumps({**r.get("output", {}), "_bench_execution_id": r["execution_id"]}) if r["ok"] else "",
                    "error": "" if r["ok"] else f"timeout [bench_execution_id={r['execution_id']}]"}}
                   for r in records if r["call_id"]]
        turns = [turn + [{"batch_result": {"results": results}}] for turn in turns]
        with patch.object(bench, "ask"), patch.object(
            bench, "events_of", side_effect=turns
        ), patch.object(bench.time, "sleep"):
            return bench.run_case(self.args, self.world, case, "case")

    def test_matched_provider_outputs_and_errors_are_recorded(self):
        records = [record(LIDAR, "lidar", call_id="scan"),
                   record(CAMERA, "camera", ok=False, call_id="failed")]
        records[0]["output"] = {"ranges": [1.5, 2.0], "angle_min": -1.57}
        run = self.run_case({"task": "scan"}, [[
            {"plan": {"calls": [plan(r["contract"], r["provider"], call_id=r["call_id"]) for r in records]}},
            {"event_kind": 4}]], records)
        self.assertEqual(run["calls"][0]["output"], {
            **records[0]["output"], "_bench_execution_id": records[0]["execution_id"]})
        self.assertIsNone(run["calls"][0]["error"])
        self.assertEqual(run["calls"][1]["output"], "")
        self.assertEqual(run["calls"][1]["error"],
                         f"timeout [bench_execution_id={records[1]['execution_id']}]")

    def builtin_call(self, output, error=""):
        contract = "robonix/system/executor/builtin/read_file"
        run = self.run_case({"task": "read"}, [[
            {"plan": {"calls": [plan(contract, "executor", call_id="read")]}},
            {"batch_result": {"results": [{"leaf_result": {
                "call_id": "read", "success": not error, "output": output, "error": error}}]}},
            {"event_kind": 4}]])
        return run["calls"][0]

    def test_builtin_outputs_keep_json_values_and_plain_text(self):
        for value in ({"content": "hello"}, [1, "two"], 0, False, None, "hello"):
            with self.subTest(value=value):
                call = self.builtin_call(json.dumps(value))
                self.assertEqual(call["output"], value)
                self.assertIsNone(call["error"])
        call = self.builtin_call("plain text", "file missing")
        self.assertEqual(call["output"], "plain text")
        self.assertEqual(call["error"], "file missing")

    def test_output_images_and_binary_are_replaced_before_truncation(self):
        payload = "cHJpdmF0ZQ==" * 4000
        value = {"camera": {"width": 640, "height": 480, "encoding": "jpeg", "data": payload},
                 "depth": {"width": 64, "height": 48, "encoding": "16UC1", "data": payload},
                 "nested": json.dumps({"front_image_base64": payload}),
                 "uri": f"frame: data:image/png;base64,{payload}",
                 "file": {"mime_type": "application/octet-stream", "data": payload},
                 "source": {"type": "base64", "data": payload},
                 "encoded": {"base64": payload}, "message": "ordinary text"}
        value["image"] = {"type": "image", "source": payload, "url": payload, "image_url": payload}
        output = self.builtin_call(json.dumps(value))["output"]
        self.assertNotIn(payload, json.dumps(output))
        self.assertNotIn("truncated", output)
        self.assertEqual(output["camera"], {**value["camera"], "data": "[image omitted]"})
        self.assertEqual(output["depth"], {**value["depth"], "data": "[image omitted]"})
        self.assertEqual(output["file"]["data"], "[binary omitted]")
        self.assertEqual(output["image"], {"type": "image", "source": "[image omitted]",
                                          "url": "[image omitted]", "image_url": "[image omitted]"})
        self.assertEqual(output["message"], "ordinary text")
        self.assertEqual(bench.call_output(b"private"), {"binary_bytes": 7})

    def test_output_truncation_bounds_utf8_json_including_escaping(self):
        for value in ("text" * 10000, '\\"\n' * 10000, "\u2603" * 10000,
                      list(range(20000)), {str(i): "value" for i in range(4000)}):
            with self.subTest(type=type(value).__name__):
                output = self.builtin_call(json.dumps(value))["output"]
                self.assertTrue(output["truncated"])
                self.assertEqual(output["json_bytes"], len(json.dumps(value, ensure_ascii=False).encode()))
                self.assertTrue(json.dumps(value, ensure_ascii=False).startswith(output["preview"]))
                self.assertLessEqual(len(json.dumps(output, ensure_ascii=False).encode()), bench.MAX_OUTPUT_BYTES)
        value = "x" * (bench.MAX_OUTPUT_BYTES - 2)
        self.assertEqual(bench.call_output(json.dumps(value)), value)

    def test_cancelled_plan_leaves_do_not_count_as_execution(self):
        planned = [plan(NAV, "navigation", {"x": x}) for x in range(4)]
        case = {"task": "visit four places", "expect": {"at_most": {NAV: 1}}}
        run = self.run_case(case, [[{"plan": {"calls": planned}}, {"event_kind": 4}]],
                            [record(NAV, "navigation", {"x": 0}, call_id=planned[0]["call_id"])])

        self.assertEqual(len(run["planned_calls"]), 4)
        self.assertEqual([c["contract"] for c in run["calls"]], [NAV])
        self.assertEqual(bench.check(case, run), [])
        run = self.run_case({"task": "photo"},
                            [[{"plan": {"calls": [plan(CAMERA, "camera")]}}, {"event_kind": 4}]])
        self.assertEqual(bench.check({"expect": {"calls": [CAMERA]}}, run),
                         [f"missing call {CAMERA}"])

    def test_execution_requires_provider_contract_and_invocation_identity(self):
        planned = [plan(CAMERA, "front", call_id="unrun"),
                   plan(CAMERA, "rear", call_id="ran"),
                   plan(CAMERA, "rear", call_id="duplicate")]
        records = [record(CAMERA, "rear", call_id="ran"), record(CAMERA, "front", {"other": True}),
                   record(LIDAR, "front"), record(CAMERA, "context-read")]
        run = self.run_case({"task": "photo"},
                            [[{"plan": {"calls": planned}}, {"event_kind": 4}]], records)

        self.assertEqual([c["call_id"] for c in run["calls"]], ["ran"])
        self.assertEqual(run["calls"][0]["provider"], "rear")

    def test_prefetch_and_connection_failure_cannot_stand_in_for_execution(self):
        planned = plan(CAMERA, "camera", {"image": "bound"})
        prefetch = record(CAMERA, "camera", {"image": "bound"}, start=0, end=1)
        actual = record(CAMERA, "camera", {"image": "bound"}, start=4, end=5,
                        call_id=planned["call_id"])
        for records, connection_failed, starts in [([prefetch], False, []),
                ([prefetch], True, []), ([prefetch, actual], False, [4])]:
            with self.subTest(connection_failed=connection_failed, starts=starts):
                events = [{"plan": {"calls": [planned]}}, {"event_kind": 4}]
                if connection_failed:
                    events.append({"batch_result": {"results": [{"leaf_result": {
                        "call_id": planned["call_id"], "success": False, "error": "connection refused"}}]}})
                run = self.run_case({"task": "photo"}, [events], records)
                self.assertEqual([c["start"] for c in run["calls"]], starts)
                if starts:
                    self.assertEqual(run["calls"][0]["args"], {"image": "bound"})

    def test_builtin_requires_a_leaf_result_and_remains_subject_to_forbidden_calls(self):
        builtin = "robonix/system/executor/builtin/write_file"
        case = {"task": "write a file", "expect": {"no_calls": [builtin]}}
        for completed in (False, True):
            with self.subTest(completed=completed):
                events = [{"plan": {"calls": [plan(builtin, "executor", call_id="write")]}}]
                if completed:
                    events.append({"batch_result": {"results": [
                        {"leaf_result": {"call_id": "write", "success": True}}]}})
                events.append({"event_kind": 4})
                run = self.run_case(case, [events])
                self.assertEqual(len(run["calls"]), int(completed))
                self.assertEqual(bench.check(case, run),
                                 [f"forbidden call {builtin}"] if completed else [])
                if completed:
                    self.assertIsNone(run["calls"][0].get("start"))
                    self.assertIsNone(run["calls"][0].get("end"))
                    for expectation in ({"orders": [[builtin]]}, {"overlap": [builtin]}):
                        with self.subTest(expectation=expectation):
                            self.assertTrue(bench.check({"expect": expectation}, run))

    def test_execution_start_order_and_overlap_override_plan_and_log_order(self):
        planned = [plan(MEMORY, "memory", {"saved": "$previous.output"}),
                   plan(LIDAR, "lidar"), plan(CAMERA, "camera")]
        case = {"task": "parallel reads then save", "expect": {
            "orders": [[CAMERA, MEMORY], [LIDAR, MEMORY]], "overlap": [CAMERA, LIDAR]}}
        run = self.run_case(case, [[{"plan": {"calls": planned}}, {"event_kind": 4}]],
                            [record(MEMORY, "memory", {"saved": "actual"}, start=6, end=7, call_id=planned[0]["call_id"]),
                             record(LIDAR, "lidar", start=2, end=4, call_id=planned[1]["call_id"]),
                             record(CAMERA, "camera", start=1, end=5, call_id=planned[2]["call_id"])])

        self.assertEqual([c["contract"] for c in run["calls"]], [CAMERA, LIDAR, MEMORY])
        self.assertEqual(run["calls"][-1]["args"], {"saved": "actual"})
        self.assertEqual(bench.check(case, run), [])
        run["calls"][1]["start"] = 5
        self.assertIn("overlap: the calls ran one after another", bench.check(case, run))

    def test_each_submitted_turn_requires_a_final_event_even_with_empty_text(self):
        unfinished = [{"final_text": "not a final event"}]
        finished = [{"event_kind": 4, "final_text": ""}]
        for turns, steer, expected in [
            ([unfinished], False, False),
            ([finished], False, True),
            ([finished, unfinished], True, False),
            ([unfinished, finished], True, False),
            ([finished, finished], True, True),
        ]:
            with self.subTest(steer=steer, turns=turns):
                case = {"task": "start"}
                if steer:
                    case["world"] = {"steer": {"after_s": 0, "task": "stop"}}
                run = self.run_case(case, turns)
                self.assertEqual(run["finished"], expected)
                self.assertEqual(bench.check(case, run), [] if expected else
                                 ["Pilot did not finish every submitted turn"])

    def test_failed_provider_calls_count_but_legacy_unexecuted_rows_do_not(self):
        run = self.run_case({"task": "photo"},
                            [[{"plan": {"calls": [plan(CAMERA, "camera", call_id="failed")]}}, {"event_kind": 4}]],
                            [record(CAMERA, "camera", ok=False, call_id="failed")])
        self.assertFalse(run["calls"][0]["ok"])
        self.assertEqual(bench.check({"expect": {"calls": [CAMERA], "count": {CAMERA: 1}}}, run), [])
        self.assertEqual(bench.check({"expect": {"no_calls": [CAMERA]}}, run),
                         [f"forbidden call {CAMERA}"])
        run["calls"].append({"contract": NAV, "args": {"x": 4}, "start": None, "end": None})
        self.assertEqual(bench.check({"expect": {"no_calls": [NAV], "count": {NAV: 0},
                                                "args_not": {NAV: {"x": 4}}}}, run), [])
        run["calls"] = run["calls"][-1:]
        self.assertEqual(bench.check({"expect": {"no_calls_at_all": True}}, run), [])

    def test_provider_handler_response_and_error_identify_the_logged_invocation(self):
        api = SimpleNamespace(Ok=object, Primitive=object, Service=object, Skill=object)
        with patch.dict("sys.modules", {"robonix_api": api, "scene": scene}), \
                patch.dict("os.environ", {"BENCH_WORLD_DIR": str(self.world)}):
            provider = importlib.import_module("testing.model_bench.world.mock_provider")
            call = provider.handler("lidar", LIDAR, {})
            with patch.object(provider.time, "sleep"):
                (self.world / "state.json").write_text(json.dumps({"case": "case"}))
                output = call({})
                (self.world / "state.json").write_text(json.dumps({"case": "case", "fail": {LIDAR: "timeout"}}))
                with self.assertRaises(RuntimeError) as caught:
                    call({})
            records = bench.lines_of(self.world / "calls.jsonl", "case")
            self.assertEqual(output["_bench_execution_id"], records[0]["execution_id"])
            self.assertIn(f"[bench_execution_id={records[1]['execution_id']}]", str(caught.exception))
            self.assertNotEqual(records[0]["execution_id"], records[1]["execution_id"])
            self.assertEqual([r["ok"] for r in records], [True, False])

    def test_resume_rejects_old_scoring_before_endpoint_probes(self):
        env = self.world / "endpoint.env"
        env.write_text("VLM_BASE_URL=https://example.test/v1\nVLM_API_KEY=unused\n")
        cases = self.world / "cases.yaml"
        cases.write_text("cases: []\n")
        result = self.world / "model_example.test.json"
        for meta in ({}, {"scoring_version": 1}):
            with self.subTest(meta=meta):
                original = json.dumps({"meta": meta, "results": []})
                result.write_text(original)
                argv = ["bench.py", "--env-file", str(env), "--model", "model",
                        "--cases", str(cases), "--out", str(self.world),
                        "--deploy", str(self.world)]
                with patch("sys.argv", argv), patch.object(bench, "check_deploy"), \
                        patch.object(bench, "probe_vision") as vision, \
                        patch.object(bench, "probe_formats") as formats:
                    with self.assertRaisesRegex(SystemExit, "older scoring version"):
                        bench.main()
                    vision.assert_not_called()
                    formats.assert_not_called()
                self.assertEqual(result.read_text(), original)
                self.assertFalse((self.world / "world/target.json").exists())

    def test_rounds_follow_request_order_and_leave_legacy_logs_supported(self):
        rows = [{"case": "case", "status": 200, "start": start, "end": start + 1,
                 "reply": reply, "usage": None, "error": None, "images": 0}
                for start, reply in [(4, "second"), (1, "first")]]
        log = self.world / "vlm.jsonl"
        log.write_text("".join(json.dumps(r) + "\n" for r in rows))
        run = self.run_case({"task": "test"}, [[{"event_kind": 4}]])
        self.assertEqual([r["reply"] for r in run["rounds"]], ["first", "second"])
        self.assertTrue(all("reply" not in r and "start" not in r for r in run["vlm"]))
        self.assertEqual(bench.check({}, run), [])
        log.write_text('{"case":"case","status":200,"usage":null}\n')
        run = self.run_case({"task": "test"}, [[{"event_kind": 4}]])
        self.assertEqual(run["rounds"], [])
        self.assertEqual(run["vlm"], [{"status": 200, "usage": None}])

    def test_reply_budget_truncates_oldest_utf8_text_with_marker(self):
        rounds = [{"reply": "first"}, {"reply": "🐈" * 40000}, {"reply": "last" * 30000}]
        newest = rounds[-1]["reply"]
        bench.cap_rounds(rounds)
        self.assertLessEqual(sum(len(r["reply"].encode()) for r in rounds), bench.MAX_REPLY_BYTES)
        self.assertEqual(rounds[0], {"reply": bench.TRUNCATED_REPLY, "truncated": True})
        self.assertTrue(rounds[1]["truncated"])
        self.assertTrue(rounds[1]["reply"].startswith(bench.TRUNCATED_REPLY))
        self.assertNotIn("\ufffd", rounds[1]["reply"])
        self.assertEqual(rounds[-1], {"reply": newest})
        before = json.dumps(rounds)
        bench.cap_rounds(rounds)
        self.assertEqual(json.dumps(rounds), before)
        rounds.append({"reply": "z" * bench.MAX_REPLY_BYTES})
        bench.cap_rounds(rounds)
        self.assertLessEqual(sum(len(r["reply"].encode()) for r in rounds), bench.MAX_REPLY_BYTES)
        rounds = [{"reply": "x" * (bench.MAX_REPLY_BYTES * 2)}]
        bench.cap_rounds(rounds)
        self.assertTrue(rounds[0]["truncated"])
        self.assertEqual(len(rounds[0]["reply"].encode()), bench.MAX_REPLY_BYTES)

    def test_resume_and_judge_load_old_results_and_retry_rounds_do_not_affect_scoring(self):
        env = self.world / "endpoint.env"
        env.write_text("VLM_BASE_URL=http://example.test\nVLM_API_KEY=unused\n")
        cases = self.world / "cases.yaml"
        cases.write_text("judge: {}\ncases:\n"
                         "- {id: old, task: old, judge: {facts: ok, assert: [answered]}}\n"
                         "- {id: new, task: new, judge: {facts: ok, assert: [answered]}}\n")
        result = self.world / "model_example.test.json"
        old = {"id": "old", "rep": 0, "final": "ok", "fails": [],
               "calls": [{"contract": LIDAR, "args": {}, "start": 1, "end": 2, "ok": True, "executed": True}]}
        result.write_text(json.dumps({"meta": {"scoring_version": 2}, "results": [old]}))
        (self.world / "world").mkdir()
        call = self.builtin_call('{"content": "hello"}')
        base = {"calls": [call], "planned_calls": [], "failed": False, "finished": True,
                "final": "ok", "elapsed_s": 1}
        runs = [{**base, "vlm": [{"status": status}], "rounds": [{"reply": text}]}
                for status, text in [(502, "partial"), (200, "complete")]]
        argv = ["bench.py", "--env-file", str(env), "--model", "model", "--repeat", "1",
                "--cases", str(cases), "--out", str(self.world), "--deploy", str(self.world)]
        with patch("sys.argv", argv), patch.object(bench, "check_deploy"), \
                patch.object(bench, "probe_vision", return_value="text"), \
                patch.object(bench, "probe_formats", return_value={}), \
                patch.object(bench, "run_case", side_effect=runs) as run_case:
            bench.main()
        self.assertEqual(run_case.call_count, 2)
        data = json.loads(result.read_text())
        self.assertEqual(data["results"][0], old)
        self.assertEqual(data["results"][1]["calls"], [{
            "contract": call["contract"], "args": {}, "start": None, "end": None,
            "ok": True, "executed": True, "output": {"content": "hello"}, "error": None}])
        self.assertEqual(data["results"][1]["rounds"], [{"reply": "partial"}, {"reply": "complete"}])
        self.assertEqual(data["results"][1]["fails"], [])
        with patch.dict("sys.modules", {"bench": bench}):
            judge = importlib.import_module("testing.model_bench.judge")
        with patch("sys.argv", ["judge.py", str(result), "--env-file", str(env),
                                 "--model", "fake", "--cases", str(cases)]), \
                patch.object(judge, "ask_judge", return_value=([{"met": True}], {})) as ask_judge:
            judge.main()
        self.assertEqual(ask_judge.call_count, 2)
        self.assertTrue(all(call.args[-1] == "ok" for call in ask_judge.call_args_list))
        judged = json.loads(result.read_text())
        self.assertEqual(judged["results"][0]["calls"], old["calls"])
        self.assertEqual(judged["results"][1]["calls"], data["results"][1]["calls"])
        self.assertNotIn("rounds", judged["results"][0])
        self.assertEqual(judged["results"][1]["rounds"], data["results"][1]["rounds"])

    def test_bundle_includes_provenance_and_all_run_logs_across_resume(self):
        (self.world / "world").mkdir()
        components = self.world / "component-logs"
        components.mkdir()
        pilot_log = components / "pilot.log"
        pilot_log.write_text("before this run\n")
        env = self.world / "model.env"
        env.write_text("VLM_BASE_URL=http://10.0.0.7\nVLM_API_KEY=planted-test-secret\n")
        cases = self.world / "cases.yaml"
        cases.write_text("cases: [{id: hello, task: hello, expect: {answered: true}}]\n")
        serving = self.world / "serving.json"
        serving.write_text(json.dumps({"framework": "fake", "version": "1", "hardware": "CPU",
                                       "precision": "fp32", "max_context": 4096, "sampling": {"temperature": 0}}))
        archive = self.world / "submission.zip"
        argv = ["bench.py", "--env-file", str(env), "--model", "fake", "--repeat", "1",
                "--cases", str(cases), "--out", str(self.world), "--deploy", str(self.world),
                "--bundle", str(archive), "--board-id", "test-board", "--endpoint-kind", "local",
                "--judge-model", "test-judge", "--serving-facts", str(serving), "--component-logs", str(components)]

        def ask(*_):
            state = json.loads((self.world / "world/state.json").read_text())
            record = {"case": state["case"], "start": 1, "end": 2, "status": 200,
                      "reply": "raw answer", "usage": None, "images": 0, "error": None}
            (self.world / "world/vlm.jsonl").write_text(json.dumps(record) + "\n")
            Path(state["request_log"]).write_text(json.dumps({**record, "messages": [{"role": "user", "content": "hello"}]}) + "\n")
            with pilot_log.open("a") as log:
                log.write("new pilot log 10.0.0.7 planted-test-secret\n"
                          "ROBONIX_TEST_SECRET=unpublished-setting\nbench proxy.py recorded this reply\n")
            return SimpleNamespace(communicate=lambda **_: ('{"event_kind":4,"final_text":"hello"}\n', "CLI stderr\n"))

        with patch("sys.argv", argv), patch.object(bench, "check_deploy"), \
                patch.object(bench, "probe_vision", return_value="text"), \
                patch.object(bench, "probe_formats", return_value={}), patch.object(bench, "ask", side_effect=ask) as ask_mock, \
                patch("testing.model_bench.provenance._binary", return_value={"version": "test", "sha256": "0" * 64}), \
                patch("testing.model_bench.provenance._git", side_effect=lambda repo, *args:
                      "a" * 40 if args[0] in ("rev-parse", "log") else ""):
            bench.main()
            bench.main()
        self.assertEqual(ask_mock.call_count, 1)
        with zipfile.ZipFile(archive) as bundle:
            result = json.loads(bundle.read("result-001.json"))
            provenance = result["meta"]["provenance"]
            self.assertEqual(provenance["board"]["id"], "test-board")
            self.assertEqual(provenance["board"]["cases_sha256"], bench.cases_sha256(cases))
            self.assertRegex(provenance["robonix"]["commit"], r"^[a-f0-9]{40}$")
            self.assertEqual(provenance["planner"]["serving"]["hardware"], "CPU")
            self.assertEqual(len(provenance["previous_runs"]), 1)
            self.assertLessEqual(provenance["start"], provenance["end"])
            self.assertEqual(result["results"][0]["rounds"][0]["reply"], "raw answer")
            names = bundle.namelist()
            for suffix in ("requests.jsonl", "pilot.log", ".stdout.jsonl", ".stderr.log", "bench.stdout.log", "bench.stderr.log"):
                self.assertTrue(any(name.endswith(suffix) for name in names), suffix)
            content = "\n".join(bundle.read(name).decode() for name in names)
            for forbidden in ("planted-test-secret", "10.0.0.7", "unpublished-setting", "before this run"):
                self.assertNotIn(forbidden, content)
            self.assertIn("CLI stderr", content)
            self.assertIn("bench proxy.py recorded this reply", content)

    def test_bundle_rejects_missing_custom_log_sources_before_model_requests(self):
        archive = self.world / "submission.zip"
        argv = ["bench.py", "--env-file", "unused", "--model", "fake", "--bundle", str(archive),
                "--board-id", "test", "--endpoint-kind", "cloud", "--judge-model", "fake",
                "--component-logs", str(self.world), "--component-logs", str(self.world / "missing")]
        with patch("sys.argv", argv), patch.object(bench, "check_deploy") as check_deploy:
            with self.assertRaises(SystemExit) as error:
                bench.main()
        self.assertEqual(error.exception.code, 2)
        check_deploy.assert_not_called()
        self.assertFalse(archive.exists())


if __name__ == "__main__":
    unittest.main()
