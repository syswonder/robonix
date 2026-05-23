# SPDX-License-Identifier: MulanPSL-2.0
"""Voiceprint service package.

The PR-era version of this file ran a custom `_ensure_stubs()` boot step
that called `grpc_tools.protoc` against `system/voiceprint/proto/voiceprint.proto`
to materialise bespoke `voiceprint_pb2` / `voiceprint_pb2_grpc` modules
on import. That bypassed the standard robonix v0.1 codegen flow
(capabilities/*.toml → atlas-managed proto bundle) and was the root
cause of the "self-written proto vs codegen" issue.

This file is now intentionally empty: gRPC stubs come from the atlas
codegen output at ``rbnx-build/codegen/proto_gen/`` (produced by
``rbnx build -p voiceprint``) and ``robonix_api`` puts that directory
on ``sys.path`` automatically, so ``service.py`` can simply do::

    import robonix_contracts_pb2 as pb
    import robonix_contracts_pb2_grpc as pb_grpc

with no per-package proto bootstrap.
"""
