specVersion: 1
description: >-
  OpenAI-compatible vision model used to judge camera images. The camera is
  chosen per Executor verification rule (verifier_args.camera_provider_id),
  not here.

properties:
  vlm:
    type: object
    description: >-
      Optional model endpoint. A complete block (base_url, api_key and model)
      is explicit and wins over the deployment catalog. A block with only
      model selects that catalog id. Omit the block to resolve the catalog's
      vision target. Without either, startup fails.
    properties:
      base_url:
        type: string
        description: >-
          API base URL, normally ending in /v1. Must use http or https and
          have a host, with no user credentials, query or fragment. Trailing
          slashes are removed and /chat/completions is appended.
      api_key:
        type: string
        description: API credential, sent as a Bearer token.
        x-secret: true
      model:
        type: string
        description: >-
          Model ID that accepts image_url data URIs, or a catalog id when
          base_url is omitted.
