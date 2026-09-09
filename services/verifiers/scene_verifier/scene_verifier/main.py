#!/usr/bin/env python3
"""scene_verifier — Robonix skill provider."""
from robonix_api import Skill, Ok

# `id` must equal this entry's `name:` in the deploy robonix_manifest.yaml.
# `namespace` groups the capabilities this provider declares.
provider = Skill(id="scene_verifier", namespace="robonix/skill/scene_verifier")


@provider.on_init
def init(cfg: dict):
    # TODO: initialise hardware / resources; declare capabilities to atlas.
    return Ok()


if __name__ == "__main__":
    provider.run()
