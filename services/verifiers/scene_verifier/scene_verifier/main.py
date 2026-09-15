#!/usr/bin/env python3
"""scene_verifier — Robonix service provider."""
from robonix_api import Service, Ok

# `id` must equal this entry's `name:` in the deploy robonix_manifest.yaml.
# `namespace` groups the capabilities this provider declares.
provider = Service(id="scene_verifier", namespace="robonix/service/scene_verifier")


@provider.on_init
def init(cfg: dict):
    # TODO: initialise hardware / resources; declare capabilities to atlas.
    return Ok()


if __name__ == "__main__":
    provider.run()
