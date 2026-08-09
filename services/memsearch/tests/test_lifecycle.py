from __future__ import annotations

import importlib.util
import os
import sys
import tempfile
import types
import unittest
import uuid
from pathlib import Path
from unittest.mock import patch


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SERVICE_PATH = PACKAGE_ROOT / "memsearch_service" / "service.py"


class _Ok:
    pass


class _Err:
    def __init__(self, message: str) -> None:
        self.message = message


class _Service:
    def __init__(self, **_kwargs) -> None:
        pass

    @staticmethod
    def mcp(_contract_id: str):
        return lambda fn: fn

    @staticmethod
    def on_init(fn):
        return fn

    @staticmethod
    def run() -> None:
        return None


class _String:
    def __init__(self, data: str = "") -> None:
        self.data = data


class _Empty:
    pass


class _ScribeLogger:
    @staticmethod
    def install_stdlib_bridge(*_args, **_kwargs) -> None:
        return None

    @staticmethod
    def info(*_args, **_kwargs) -> None:
        return None


def _load_service():
    observed_threads: list[str | None] = []

    def configure_onnxruntime() -> None:
        observed_threads.append(os.environ.get("MEMSEARCH_ONNX_THREADS"))

    robonix_api = types.ModuleType("robonix_api")
    robonix_api.Service = _Service
    robonix_api.Ok = _Ok
    robonix_api.Err = _Err
    robonix_api.scribe_logger = _ScribeLogger

    std_msgs_mcp = types.ModuleType("std_msgs_mcp")
    std_msgs_mcp.Empty = _Empty
    std_msgs_mcp.String = _String

    onnx_compat = types.ModuleType("memsearch_service.onnx_compat")
    onnx_compat.configure_onnxruntime = configure_onnxruntime

    module_name = f"memsearch_service._lifecycle_test_{uuid.uuid4().hex}"
    spec = importlib.util.spec_from_file_location(module_name, SERVICE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SERVICE_PATH}")
    module = importlib.util.module_from_spec(spec)
    with patch.dict(
        sys.modules,
        {
            "robonix_api": robonix_api,
            "std_msgs_mcp": std_msgs_mcp,
            "memsearch_service.onnx_compat": onnx_compat,
        },
    ):
        spec.loader.exec_module(module)
    return module, observed_threads


def _run_init(module, memsearch_cls, cfg: dict):
    memsearch = types.ModuleType("memsearch")
    memsearch.MemSearch = memsearch_cls
    with patch.dict(sys.modules, {"memsearch": memsearch}):
        return module.init(cfg)


class _WorkingMemSearch:
    instances = []

    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs
        self.index_calls = 0
        self.__class__.instances.append(self)

    async def index(self) -> None:
        self.index_calls += 1


class _FailingIndexMemSearch(_WorkingMemSearch):
    async def index(self) -> None:
        raise OSError("index unavailable")


class _FailingConstructionMemSearch:
    def __init__(self, **_kwargs) -> None:
        raise PermissionError("database is not writable")


class MemoryLifecycleTest(unittest.TestCase):
    def setUp(self) -> None:
        _WorkingMemSearch.instances.clear()

    def test_driver_config_wins_over_environment_and_builds_index(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            configured_memory = root / "configured-memory"
            configured_db = root / "configured.db"
            module, observed_threads = _load_service()
            with patch.dict(
                os.environ,
                {
                    "AGENT_MEMORY_DIR": str(root / "environment-memory"),
                    "AGENT_MILVUS_URI": str(root / "environment.db"),
                    "MEMSEARCH_ONNX_THREADS": "9",
                },
            ):
                result = _run_init(
                    module,
                    _WorkingMemSearch,
                    {
                        "memory_dir": str(configured_memory),
                        "milvus_uri": str(configured_db),
                        "onnx_threads": 3,
                    },
                )

            self.assertIsInstance(result, _Ok)
            self.assertEqual(module.MEMORY_DIR, str(configured_memory.resolve()))
            self.assertEqual(module.MILVUS_URI, str(configured_db.resolve()))
            self.assertEqual(observed_threads, ["3"])
            self.assertEqual(len(_WorkingMemSearch.instances), 1)
            backend = _WorkingMemSearch.instances[0]
            self.assertEqual(backend.kwargs["paths"], [str(configured_memory.resolve())])
            self.assertEqual(backend.kwargs["milvus_uri"], str(configured_db.resolve()))
            self.assertEqual(backend.index_calls, 1)

    def test_environment_values_remain_compatibility_fallbacks(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            environment_memory = root / "environment-memory"
            environment_db = root / "environment.db"
            module, observed_threads = _load_service()
            with patch.dict(
                os.environ,
                {
                    "AGENT_MEMORY_DIR": str(environment_memory),
                    "AGENT_MILVUS_URI": str(environment_db),
                    "MEMSEARCH_ONNX_THREADS": "4",
                },
            ):
                result = _run_init(module, _WorkingMemSearch, {})

            self.assertIsInstance(result, _Ok)
            self.assertEqual(module.MEMORY_DIR, str(environment_memory.resolve()))
            self.assertEqual(module.MILVUS_URI, str(environment_db.resolve()))
            self.assertEqual(observed_threads, ["4"])

    def test_index_failure_returns_err_and_does_not_publish_backend(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            module, _ = _load_service()
            result = _run_init(
                module,
                _FailingIndexMemSearch,
                {
                    "memory_dir": str(Path(tmp) / "memory"),
                    "milvus_uri": str(Path(tmp) / "memory.db"),
                },
            )

            self.assertIsInstance(result, _Err)
            self.assertIn("index unavailable", result.message)
            self.assertIsNone(module.mem)
            self.assertIsNone(module.MEMORY_DIR)
            self.assertIsNone(module.MILVUS_URI)

    def test_backend_construction_failure_returns_err(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            module, _ = _load_service()
            result = _run_init(
                module,
                _FailingConstructionMemSearch,
                {
                    "memory_dir": str(Path(tmp) / "memory"),
                    "milvus_uri": str(Path(tmp) / "memory.db"),
                },
            )

            self.assertIsInstance(result, _Err)
            self.assertIn("database is not writable", result.message)
            self.assertIsNone(module.mem)

    def test_start_hook_does_not_kill_database_holders(self) -> None:
        manifest = (PACKAGE_ROOT / "package_manifest.yaml").read_text(encoding="utf-8")
        self.assertNotIn("fuser", manifest)
        self.assertNotIn("stale milvus lock", manifest)


if __name__ == "__main__":
    unittest.main()
