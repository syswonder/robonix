# SPDX-License-Identifier: MulanPSL-2.0
"""MCP service and standard Robonix lifecycle entry point."""
import logging
import time

from robonix_api import Err, Ok, Service, scribe_logger
from verifier_mcp import Verify_Request, Verify_Response

from . import camera, core, vlm
from .directory import CameraDirectory

log = logging.getLogger("vlm_verifier")
ATLAS = CameraDirectory()

service = Service(id="vlm_verifier", namespace="robonix/service/verifier")
CAMERAS = camera.CameraPool(lambda: ATLAS, service.id)
_config = None
_before_images = {}


@service.on_init
def init(cfg: dict):
    """Validate configuration without acquiring a camera or calling the model."""
    global _config
    _before_images.clear()
    try:
        _config = vlm.VlmConfig.parse(cfg.get("vlm"))
    except (ValueError, TypeError) as exc:
        _config = None
        return Err(str(exc))
    return Ok()


@service.mcp("robonix/service/verifier/verify")
async def verify(req: Verify_Request) -> Verify_Response:
    """Capture before execution or verify the completed action with both images."""
    config = _config
    if config is None:
        raise RuntimeError("VLM Verifier is not initialized")
    phase = core.parse_envelope(req.args_json).get("phase")
    if phase not in ("before", "after"):
        raise ValueError("phase must be before or after")

    async def observe(provider_id):
        jpeg = await CAMERAS.capture(provider_id)
        try:
            path = camera.save_verification_frame(f"{req.call_id}:{phase}:{provider_id}", jpeg)
        except OSError as exc:
            log.warning("call_id=%r could not save verification frame (%s)",
                        req.call_id, type(exc).__name__)
        else:
            log.info("call_id=%r verification frame saved: %s", req.call_id, path)
        return jpeg

    async def evaluate(envelope, images):
        now = time.monotonic()
        context = {k: v for k, v in envelope.items() if k not in ("phase", "target_output")}
        if phase == "before":
            _before_images[req.call_id] = (now, context, images)
            return {"passed": True, "detail": "Before image captured; result not yet verified."}
        _, before_context, before = _before_images.pop(req.call_id)
        if context != before_context:
            raise ValueError("before image context mismatch")
        return await vlm.judge(config, envelope, before, images)

    result = await core.verify(req.call_id, req.args_json, observe, evaluate)
    return Verify_Response(**result)


@service.on_shutdown
def shutdown():
    """Release the shared camera subscriptions and their Atlas channels."""
    CAMERAS.close()
    _before_images.clear()
    return Ok()


def main():
    scribe_logger.install_stdlib_bridge("vlm_verifier")
    service.run()


if __name__ == "__main__":
    main()
