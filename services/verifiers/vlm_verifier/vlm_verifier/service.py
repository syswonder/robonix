# SPDX-License-Identifier: MulanPSL-2.0
"""MCP service and standard Robonix lifecycle entry point."""
from robonix_api import Err, Ok, Service, scribe_logger
from verifier_mcp import Verify_Request, Verify_Response

from . import camera, core, vlm
from .directory import CameraDirectory

ATLAS = CameraDirectory()

service = Service(id="vlm_verifier", namespace="robonix/service/verifier")
_config = None


@service.on_init
def init(cfg: dict):
    """Validate configuration without acquiring a camera or calling the model."""
    global _config
    try:
        _config = vlm.VlmConfig.parse(cfg.get("vlm"))
    except (ValueError, TypeError) as exc:
        _config = None
        return Err(str(exc))
    return Ok()


@service.mcp("robonix/service/verifier/verify")
async def verify(req: Verify_Request) -> Verify_Response:
    """Verify a completed action using one fresh image from its configured camera."""
    config = _config
    if config is None:
        raise RuntimeError("VLM Verifier is not initialized")

    async def observe(provider_id):
        return await camera.capture(ATLAS, service.id, provider_id)

    async def evaluate(envelope, jpeg):
        return await vlm.judge(config, envelope, jpeg)

    result = await core.verify(req.call_id, req.args_json, observe, evaluate)
    return Verify_Response(**result)


def main():
    scribe_logger.install_stdlib_bridge("vlm_verifier")
    service.run()


if __name__ == "__main__":
    main()
