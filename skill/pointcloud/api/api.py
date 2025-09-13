# SPDX-License-Identifier: MulanPSL-2.0
# Copyright (c) 2025, wheatfox <wheatfox17@icloud.com>

from Robonix.manager.eaios_decorators import eaios
from Robonix.uapi.graph.entity import Entity


@eaios.caller
def skl_spatiallm_detect(self_entity: Entity, ply_model: bytes) -> dict:
    pass
