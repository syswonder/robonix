# SPDX-License-Identifier: MulanPSL-2.0
"""Expose the calibrated Nova5 cycle as one Robonix skill."""
import os
from pathlib import Path
import signal
import tempfile
import threading
import uuid

from robonix_api import Err, Ok, Skill
from nova5_pick_return_mcp import (Cancel_Request, Cancel_Response,
                                   GetStatus_Request, GetStatus_Response,
                                   Run_Request, Run_Response)


skill = Skill(id="nova5_pick_return", namespace="robonix/skill/nova5_pick_return")
_lock = threading.Lock()
_config = None
_active = False
_run = None
_latched = False
_script = Path(__file__).resolve().parents[3] / "run.py"


def refresh():
    global _latched
    if _run is None or _run["state"] != "RUNNING":
        return
    code = _run["process"].poll()
    if code is None:
        return
    if _run["canceled"]:
        _run["state"] = "CANCELED"
    elif code == 0:
        _run["state"] = "SUCCEEDED"
    else:
        _run["state"] = "FAILED"
    if _run["state"] != "SUCCEEDED":
        _latched = True


@skill.on_init
def initialize(config):
    global _config
    python = Path(config.get("python", ""))
    settings = Path(config.get("settings", ""))
    if not python.is_file() or not settings.is_file() or not _script.is_file():
        return Err("Nova5 workflow Python, settings or script is missing")
    _config = (python, settings)
    return Ok()


@skill.on_activate
def activate():
    global _active
    if _config is None or _latched:
        return Err("Nova5 workflow is unconfigured or needs physical review after a fault")
    _active = True
    return Ok()


@skill.on_shutdown
@skill.on_deactivate
def deactivate():
    global _active
    with _lock:
        _active = False
        refresh()
        cancel_active()
    return Ok()


def cancel_active():
    if _run is not None and _run["state"] == "RUNNING":
        _run["canceled"] = True
        try:
            os.killpg(_run["process"].pid, signal.SIGINT)
        except ProcessLookupError:
            refresh()


@skill.mcp("robonix/skill/nova5_pick_return/run")
def run(request: Run_Request) -> Run_Response:
    """Pick the configured black box, replace it, and return Nova5 home."""
    global _run
    with _lock:
        refresh()
        if request.target != "box":
            raise ValueError('Unsupported target; call run with target="box"')
        if _latched:
            raise RuntimeError(f"Previous Nova5 cycle failed or was canceled; inspect {_run['folder'] / 'run.log'} and the physical state before restarting the skill")
        if not _active:
            raise RuntimeError("Nova5 skill is not active; activate it before calling run")
        if _run is not None and _run["state"] == "RUNNING":
            raise RuntimeError("A Nova5 cycle is already running")
        python, settings = _config
        logs = _script.parent / ".artifacts" / "skill-runs"
        logs.mkdir(parents=True, exist_ok=True)
        if sum(p.is_dir() for p in logs.iterdir()) >= 100:
            raise RuntimeError("100 run records retained; stop the example and archive old artifacts before another run")
        folder = Path(tempfile.mkdtemp(prefix="nova5-skill-", dir=logs))
        process = skill.spawn([str(python), str(_script), "--settings", str(settings)],
                              log=folder / "run.log")
        _run = {"id": uuid.uuid4().hex, "process": process,
                "state": "RUNNING", "canceled": False, "folder": folder}
        return Run_Response(run_id=_run["id"], detail="Nova5 cycle started")


@skill.mcp("robonix/skill/nova5_pick_return/run/status")
def status(request: GetStatus_Request) -> GetStatus_Response:
    with _lock:
        if _run is None or request.run_id != _run["id"]:
            raise ValueError("Unknown Nova5 run ID")
        refresh()
        return GetStatus_Response(state=_run["state"], detail=str(_run["folder"] / "run.log"))


@skill.mcp("robonix/skill/nova5_pick_return/run/cancel")
def cancel(request: Cancel_Request) -> Cancel_Response:
    with _lock:
        if _run is None or request.run_id != _run["id"]:
            raise ValueError("Unknown Nova5 run ID")
        refresh()
        cancel_active()
        return Cancel_Response(accepted=True, detail="Cancellation requested; check physical state")


if __name__ == "__main__":
    skill.run()
