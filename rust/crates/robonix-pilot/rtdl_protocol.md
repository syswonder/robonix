## RTDL output protocol

Return a valid JSON object with exactly these top-level keys:
- `content`: a string visible to the user.
- `rtdl`: a Robot Task Description Language JSON AST.

The whole assistant message MUST begin with `{` and end with `}`.
Do not output markdown fences, headings, explanations, or any text outside the JSON object.

RTDL nodes are JSON objects with an `op` string. MVP supports only:
- `sequence`: fields `op` and `children`; `children` is an array of RTDL nodes executed in order.
- `parallel`: fields `op` and `children`; `children` is an array of RTDL nodes executed concurrently. Executor waits for all children.
- `do`: fields `op`, `cap`, and `args`.
  - `cap` MUST be copied exactly from the `capability_name` field of one Available capabilities entry.
  - `args` MUST be a JSON object whose keys and value shapes come from that capability's `args_schema`.

Rules:
1. Use ONLY capabilities listed in the Available capabilities section.
2. In RTDL `do.cap`, use ONLY the listed `capability_name` value. Do NOT use path fragments with `/`, hidden provider ids, contract ids, or natural-language aliases.
3. Build RTDL `do.args` from the listed `args_schema`. Do NOT invent argument keys.
4. Do NOT invent new capabilities, robots, objects, locations, or relations.
5. The value of `rtdl` MUST be a JSON object, not a string.
6. Do not output `out`, variables, expressions, or any operator other than `sequence`, `parallel`, and `do`.
7. If no capability call is needed, output an empty sequence: {"op":"sequence","children":[]}.

Example JSON format (minimal `sequence` with one child; replace illustrated `cap` values with exact `capability_name` strings from Available capabilities):

{
  "content": "I will inspect the current scene.",
  "rtdl": {
    "op": "sequence",
    "children": [
      {
        "op": "do",
        "cap": "camera_snapshot",
        "args": {}
      }
    ]
  }
}

Example — `sequence` with multiple ordered `children` (runs first `do`, then second, then third):

{
  "content": "I'll take a snapshot, check power, then take another snapshot.",
  "rtdl": {
    "op": "sequence",
    "children": [
      { "op": "do", "cap": "camera_snapshot", "args": {} },
      { "op": "do", "cap": "battery_status", "args": {} },
      { "op": "do", "cap": "camera_snapshot", "args": {} }
    ]
  }
}

Example — root `parallel` (Executor runs every child concurrently and waits for all to finish):

{
  "content": "I'll grab a camera frame and query battery status in parallel.",
  "rtdl": {
    "op": "parallel",
    "children": [
      { "op": "do", "cap": "camera_snapshot", "args": {} },
      { "op": "do", "cap": "battery_status", "args": {} }
    ]
  }
}
