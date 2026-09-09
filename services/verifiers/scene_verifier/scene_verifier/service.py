"""Robonix lifecycle entrypoint and request orchestration."""
import logging

from .config import VerifierConfig, parse_config
from .core import parse_envelope, parse_goal, require_string, verify_navigation_result
from .scene_client import fetch_robot_context

VERIFY_CONTRACT = "robonix/service/verifier/verify"
log = logging.getLogger("scene_verifier")


async def verify_request(call_id: str, args_json: str, config: VerifierConfig,
                         consumer_id: str, fetch=fetch_robot_context):
    """Parse before observation; preserve errors for Executor's unavailable path."""
    call_id = require_string(call_id, "call_id")
    envelope = parse_envelope(args_json)
    goal = parse_goal(envelope.target_args, envelope.check_yaw)
    try:
        context = await fetch(consumer_id, envelope.scene_provider_id,
                              config.observation_timeout_s)
        passed, detail = verify_navigation_result(
            goal, context, config, envelope.expected_map_id,
        )
    except Exception:
        log.exception("call_id=%s Scene verification unavailable", call_id)
        raise
    log.info("call_id=%s target_provider=%s scene_provider=%s passed=%s %s",
             call_id, envelope.target_provider_id, envelope.scene_provider_id,
             passed, detail)
    return passed, detail


def create_service():
    """Bind generated request/response types without importing them in pure tests."""
    from robonix_api import Service, Ok, Err, scribe_logger
    from verifier_mcp import Verify_Request, Verify_Response

    scribe_logger.install_stdlib_bridge("scene_verifier")
    service = Service(id="scene_verifier", namespace="robonix/service/verifier")
    config = None

    @service.on_init
    def init(raw: dict):
        """Validate config; repeat initialization must use identical settings."""
        nonlocal config
        try:
            candidate = parse_config(raw)
            if config is not None and candidate != config:
                return Err("SceneVerifier is already initialized with different config")
            config = candidate
            return Ok()
        except (TypeError, ValueError) as exc:
            return Err(str(exc))

    @service.mcp(VERIFY_CONTRACT)
    async def verify(req: Verify_Request) -> Verify_Response:
        """Verify one completed navigation call using the configured Scene."""
        if config is None:
            raise RuntimeError("SceneVerifier is not initialized")
        passed, detail = await verify_request(
            req.call_id, req.args_json, config, service.id,
        )
        return Verify_Response(passed=passed, detail=detail)

    return service


def main() -> None:
    create_service().run()


if __name__ == "__main__":
    main()