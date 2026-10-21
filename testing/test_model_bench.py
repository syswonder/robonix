# SPDX-License-Identifier: MulanPSL-2.0
"""Regression tests for provider-observed model benchmark scoring."""

from __future__ import annotations

import importlib
import json
import tempfile
import unittest
import uuid
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
                    "output": json.dumps({"_bench_execution_id": r["execution_id"]}) if r["ok"] else "",
                    "error": "" if r["ok"] else f"timeout [bench_execution_id={r['execution_id']}]"}}
                   for r in records if r["call_id"]]
        turns = [turn + [{"batch_result": {"results": results}}] for turn in turns]
        with patch.object(bench, "ask"), patch.object(
            bench, "events_of", side_effect=turns
        ), patch.object(bench.time, "sleep"):
            return bench.run_case(self.args, self.world, case, "case")

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


if __name__ == "__main__":
    unittest.main()
