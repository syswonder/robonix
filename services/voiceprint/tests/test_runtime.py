from __future__ import annotations

import importlib.util
import json
import os
import sys
import tempfile
import threading
import types
import unittest
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import patch

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SERVICE_PATH = PACKAGE_ROOT / "voiceprint_service" / "service.py"


class _Ok:
    pass


class _Err:
    def __init__(self, message: str) -> None:
        self.message = message


class _Response:
    def __init__(self, **kwargs) -> None:
        self.__dict__.update(kwargs)


class _Service:
    def __init__(self, **_kwargs) -> None:
        self.grpc_servicers = []

    def attach_grpc_servicer(self, contract_id: str, servicer) -> None:
        self.grpc_servicers.append((contract_id, servicer))

    @staticmethod
    def mcp(_contract_id: str):
        return lambda fn: fn

    @staticmethod
    def on_init(fn):
        return fn

    @staticmethod
    def on_activate(fn):
        return fn

    @staticmethod
    def on_deactivate(fn):
        return fn

    @staticmethod
    def on_shutdown(fn):
        return fn

    @staticmethod
    def run() -> None:
        return None


class _ScribeLogger:
    @staticmethod
    def install_stdlib_bridge(*_args, **_kwargs) -> None:
        return None


def _load_service():
    class FakeEngine:
        instances = []

        def __init__(self, device=None) -> None:
            self.device = device
            self.close_calls = 0
            self.__class__.instances.append(self)

        def extract_from_pcm(self, *_args, **_kwargs) -> np.ndarray:
            return np.asarray([1.0, 0.0], dtype=np.float32)

        def close(self) -> None:
            self.close_calls += 1

    robonix_api = types.ModuleType("robonix_api")
    robonix_api.Service = _Service
    robonix_api.Ok = _Ok
    robonix_api.Err = _Err
    robonix_api.scribe_logger = _ScribeLogger

    contracts_pb2 = types.ModuleType("robonix_contracts_pb2")
    contracts_grpc = types.ModuleType("robonix_contracts_pb2_grpc")
    base_servicer = type("_BaseServicer", (), {})
    contracts_grpc.RobonixServiceVoiceprintIdentifyServicer = base_servicer
    contracts_grpc.RobonixServiceVoiceprintEnrollServicer = base_servicer
    contracts_grpc.RobonixServiceVoiceprintDeleteServicer = base_servicer
    contracts_grpc.RobonixServiceVoiceprintListServicer = base_servicer

    voiceprint_pb2 = types.ModuleType("voiceprint_pb2")
    voiceprint_pb2.Identify_Response = _Response
    voiceprint_pb2.Enroll_Response = _Response
    voiceprint_pb2.DeleteEnrolled_Response = _Response
    voiceprint_pb2.ListEnrolled_Response = _Response

    voiceprint_mcp = types.ModuleType("voiceprint_mcp")
    voiceprint_mcp.ListEnrolled_Request = type("ListEnrolled_Request", (), {})
    voiceprint_mcp.ListEnrolled_Response = _Response

    package = types.ModuleType("voiceprint_service")
    package.__path__ = [str(PACKAGE_ROOT / "voiceprint_service")]
    engine = types.ModuleType("voiceprint_service.engine")
    engine.EcapaTdnnEngine = FakeEngine

    module_name = f"voiceprint_service._runtime_test_{uuid.uuid4().hex}"
    spec = importlib.util.spec_from_file_location(module_name, SERVICE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SERVICE_PATH}")
    module = importlib.util.module_from_spec(spec)
    with patch.dict(
        sys.modules,
        {
            module_name: module,
            "robonix_api": robonix_api,
            "robonix_contracts_pb2": contracts_pb2,
            "robonix_contracts_pb2_grpc": contracts_grpc,
            "voiceprint_pb2": voiceprint_pb2,
            "voiceprint_mcp": voiceprint_mcp,
            "voiceprint_service": package,
            "voiceprint_service.engine": engine,
        },
    ):
        spec.loader.exec_module(module)
    return module, FakeEngine


class ThresholdValidationTest(unittest.TestCase):
    def test_accepts_boundaries_and_interior_values(self) -> None:
        module, _ = _load_service()
        for raw, expected in ((0, 0.0), ("0.25", 0.25), (1, 1.0)):
            with self.subTest(raw=raw):
                self.assertEqual(module._validate_threshold(raw), expected)

    def test_rejects_non_finite_out_of_range_and_non_numeric_values(self) -> None:
        module, _ = _load_service()
        for raw in (float("nan"), float("inf"), "-inf", -0.01, 1.01, "bad", None):
            with self.subTest(raw=raw):
                with self.assertRaisesRegex(ValueError, r"finite number in \[0, 1\]"):
                    module._validate_threshold(raw)

    def test_invalid_environment_threshold_fails_init(self) -> None:
        module, _ = _load_service()
        with tempfile.TemporaryDirectory() as tmp, patch.dict(
            os.environ,
            {"VOICEPRINT_THRESHOLD": "nan"},
        ):
            result = module.init({"data_dir": tmp})

        self.assertIsInstance(result, _Err)
        self.assertIn("threshold must be a finite number in [0, 1]", result.message)
        self.assertIsNone(module._db)

    def test_explicit_threshold_takes_precedence_over_invalid_environment(self) -> None:
        module, _ = _load_service()
        with tempfile.TemporaryDirectory() as tmp, patch.dict(
            os.environ,
            {"VOICEPRINT_THRESHOLD": "nan"},
        ):
            result = module.init({"data_dir": tmp, "threshold": 0.4})

        self.assertIsInstance(result, _Ok)
        self.assertEqual(module._threshold_value, 0.4)


class DeviceConfigTest(unittest.TestCase):
    def test_environment_device_remains_a_compatibility_fallback(self) -> None:
        module, fake_engine = _load_service()
        with tempfile.TemporaryDirectory() as tmp, patch.dict(
            os.environ,
            {"VOICEPRINT_DEVICE": "cpu"},
        ):
            self.assertIsInstance(module.init({"data_dir": tmp}), _Ok)
            self.assertIsInstance(module.activate(), _Ok)

        self.assertEqual(fake_engine.instances[0].device, "cpu")

    def test_explicit_device_takes_precedence_over_environment(self) -> None:
        module, fake_engine = _load_service()
        with tempfile.TemporaryDirectory() as tmp, patch.dict(
            os.environ,
            {"VOICEPRINT_DEVICE": "cuda:0"},
        ):
            self.assertIsInstance(
                module.init({"data_dir": tmp, "device": "cpu"}),
                _Ok,
            )
            self.assertIsInstance(module.activate(), _Ok)

        self.assertEqual(fake_engine.instances[0].device, "cpu")

    def test_invalid_device_fails_init(self) -> None:
        module, _ = _load_service()
        with tempfile.TemporaryDirectory() as tmp:
            result = module.init({"data_dir": tmp, "device": ""})

        self.assertIsInstance(result, _Err)
        self.assertIn("device must be a non-empty string or null", result.message)
        self.assertIsNone(module._db)


class EnrolledDBConcurrencyTest(unittest.TestCase):
    def test_concurrent_duplicate_id_enrollment_persists_only_one(self) -> None:
        module, _ = _load_service()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "enrolled.json"
            db = module.EnrolledDB(path)
            workers = 8
            barrier = threading.Barrier(workers)

            def enroll(index: int):
                barrier.wait()
                embedding = np.zeros(workers, dtype=np.float32)
                embedding[index] = 1.0
                return db.enroll_unique(
                    "same-id",
                    f"name-{index}",
                    embedding,
                    1.0,
                )

            with ThreadPoolExecutor(max_workers=workers) as pool:
                results = list(pool.map(enroll, range(workers)))

            self.assertEqual(sum(success for success, _ in results), 1)
            self.assertEqual(set(json.loads(path.read_text(encoding="utf-8"))), {"same-id"})

    def test_concurrent_duplicate_voice_enrollment_persists_only_one(self) -> None:
        module, _ = _load_service()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "enrolled.json"
            db = module.EnrolledDB(path)
            workers = 12
            barrier = threading.Barrier(workers)
            embedding = np.asarray([1.0, 0.0], dtype=np.float32)

            def enroll(index: int):
                barrier.wait()
                return db.enroll_unique(
                    f"user-{index}",
                    f"name-{index}",
                    embedding,
                    0.25,
                )

            with ThreadPoolExecutor(max_workers=workers) as pool:
                results = list(pool.map(enroll, range(workers)))

            self.assertEqual(sum(success for success, _ in results), 1)
            self.assertEqual(len(db), 1)
            self.assertEqual(len(json.loads(path.read_text(encoding="utf-8"))), 1)
            self.assertFalse(path.with_suffix(".json.tmp").exists())

    def test_concurrent_duplicate_name_enrollment_persists_only_one(self) -> None:
        module, _ = _load_service()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "enrolled.json"
            db = module.EnrolledDB(path)
            workers = 8
            barrier = threading.Barrier(workers)

            def enroll(index: int):
                barrier.wait()
                embedding = np.zeros(workers, dtype=np.float32)
                embedding[index] = 1.0
                return db.enroll_unique(
                    f"user-{index}",
                    "same-name",
                    embedding,
                    1.0,
                )

            with ThreadPoolExecutor(max_workers=workers) as pool:
                results = list(pool.map(enroll, range(workers)))

            self.assertEqual(sum(success for success, _ in results), 1)
            self.assertEqual(len(json.loads(path.read_text(encoding="utf-8"))), 1)

    def test_concurrent_distinct_enrollments_all_survive_persistence(self) -> None:
        module, _ = _load_service()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "enrolled.json"
            db = module.EnrolledDB(path)
            workers = 8
            barrier = threading.Barrier(workers)

            def enroll(index: int):
                barrier.wait()
                embedding = np.zeros(workers, dtype=np.float32)
                embedding[index] = 1.0
                return db.enroll_unique(
                    f"user-{index}",
                    f"name-{index}",
                    embedding,
                    0.5,
                )

            with ThreadPoolExecutor(max_workers=workers) as pool:
                results = list(pool.map(enroll, range(workers)))

            self.assertTrue(all(success for success, _ in results))
            payload = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(set(payload), {f"user-{i}" for i in range(workers)})

    def test_failed_persist_rolls_back_in_memory_mutation(self) -> None:
        module, _ = _load_service()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "enrolled.json"
            db = module.EnrolledDB(path)
            db.enroll("existing", "Existing", np.asarray([1.0, 0.0]))

            with patch.object(db, "_save_locked", side_effect=OSError("disk full")):
                with self.assertRaisesRegex(OSError, "disk full"):
                    db.enroll_unique(
                        "new",
                        "New",
                        np.asarray([0.0, 1.0]),
                        0.5,
                    )

            self.assertEqual(db.list_users(), [("existing", "Existing")])
            self.assertEqual(set(json.loads(path.read_text(encoding="utf-8"))), {"existing"})


class LifecycleResourceTest(unittest.TestCase):
    def test_model_loads_on_activate_and_closes_on_deactivate(self) -> None:
        module, fake_engine = _load_service()
        with tempfile.TemporaryDirectory() as tmp:
            init_result = module.init(
                {"data_dir": tmp, "threshold": 0.3, "device": "cpu"},
            )

            self.assertIsInstance(init_result, _Ok)
            self.assertIsNotNone(module._db)
            self.assertIsNone(module._engine)
            self.assertEqual(fake_engine.instances, [])

            activate_result = module.activate()
            self.assertIsInstance(activate_result, _Ok)
            self.assertEqual(len(fake_engine.instances), 1)
            engine = fake_engine.instances[0]
            self.assertIs(module._engine, engine)
            self.assertEqual(engine.device, "cpu")

            deactivate_result = module.deactivate()
            self.assertIsInstance(deactivate_result, _Ok)
            self.assertIsNone(module._engine)
            self.assertEqual(engine.close_calls, 1)

            self.assertIsInstance(module.deactivate(), _Ok)
            self.assertEqual(engine.close_calls, 1)


if __name__ == "__main__":
    unittest.main()
