## RTDL output protocol

Return a valid JSON object with exactly these top-level keys:
- `content`: a string visible to the user.
- `rtdl_description`: a short string naming what THIS `rtdl` tree is doing
  (the current sub-task, e.g. `"fetch water"`). Used to label the tree while it
  runs. Empty string is allowed only when `rtdl` is an empty sequence.
- `rtdl`: a Robot Task Description Language JSON AST (the tree to dispatch now).
- `task_update`: either `null` (keep the current overall task/goal unchanged)
  or an object updating the overall task. See "Task state" below.

The whole assistant message MUST begin with `{` and end with `}`.
Do not output markdown fences, headings, explanations, or any text outside the JSON object.

### Task state (`task_update`)

You own one persistent overall task per conversation. Each turn you may revise
it through `task_update`:

- `null` — keep the current goal, criterion, and status exactly as they are.
- object with these keys (all required when the value is an object):
  - `goal`: the overall objective in one sentence, e.g.
    `"bring me water and play music at the same time"`. Replaces the prior goal.
  - `success_criterion`: how to know the goal is done — concrete and checkable,
    e.g. `"the water cup is next to the user AND music is audibly playing"`.
  - `status`: `"in_progress"` or `"done"`.

`status: "done"` is the ONE authoritative completion signal. The turn ends only
when the overall task is `done`, every dispatched tree has finished, and there
is no pending user input — NOT merely because you emitted an empty `rtdl`. An
empty `rtdl` only means "I am dispatching no new tree this round" (e.g. you are
waiting for an in-flight tree to finish). Do not mark `done` until the
`success_criterion` actually holds — verify with an observation first.

Do NOT set `status: "done"` while any tree in the "In-flight trees" list is
still running — even one you just asked to cancel. Cancelling a tree does not
make the task done; wait for that tree to actually leave the In-flight list (you
will see its result) before declaring done. Keep emitting empty-`rtdl` rounds
(which just wait) until the forest is clear, then mark `done`.

Set `task_update` to a fresh object when:
- the user gives a new goal or steers you mid-task (incorporate their change);
- you refine the goal as you learn more;
- the goal is complete (`status: "done"`).

### RTDL tree

RTDL nodes are JSON objects with an `op` string. MVP supports only:
- `sequence`: fields `op` and `children`; `children` is an array of RTDL nodes executed in order.
- `parallel`: fields `op` and `children`; `children` is an array of RTDL nodes executed concurrently. Executor waits for all children.
- `do`: fields `op`, `cap`, and `args`.
  - `cap` MUST be copied exactly from the `capability_name` field of one Available capabilities entry.
  - `args` MUST be a JSON object whose keys and value shapes come from that capability's `args_schema`.

Each `rtdl` tree you emit is dispatched as its own plan and runs concurrently
with trees you dispatched on earlier turns — together they form a forest. Trees
do not block each other. The "In-flight trees" section of your context lists
every tree still running, with its `plan_id` and description. To stop one, emit
a `do` node calling the `builtin_cancel_plan` capability with that tree's
`plan_id`. Whether to cancel an in-flight tree when the user steers you is your
call: cancel when the new request conflicts with what a tree is doing; leave it
running when the new request is additive.

### Plan IDs (read this — you do NOT choose them)

Every tree you dispatch is assigned an integer `plan_id` by the system,
starting at `1` and never reused. You do NOT pick plan ids:

- You never create a plan id for a NEW tree. Whenever a field would ask you for
  the id of a tree you are creating, write `-1`; the system replaces it with the
  next real id. Any value other than `-1` that you put there is ignored.
- The plan ids you see in the "In-flight trees" list and in the conversation
  history were assigned by the system, not by you. They exist so you can
  reference an EXISTING tree — for example, the `plan_id` argument to
  `builtin_cancel_plan` must be the real id of the tree you want to stop, copied
  exactly from the "In-flight trees" list.
- Never invent, guess, increment, or reuse a plan id for a new tree just because
  you saw ids in the history. Referencing an existing id (to cancel it) is fine;
  fabricating one for a new tree is not.

### Plan ahead — compose multi-step trees, do not drip one node per round

RTDL exists to express *structured* plans. Put every step you can already
foresee into ONE tree:

- If you know the steps in advance and they do not depend on each other's
  results, emit them as one `sequence` (ordered) or `parallel` (independent)
  tree in a single round. Do NOT emit one node, wait, emit the next — that
  wastes RTDL and is slower.
- Use `parallel` for steps with no ordering dependency (e.g. snapshot + battery
  read; play music + start navigating).
- Only split across rounds when the NEXT step genuinely depends on the RESULT of
  the previous one — i.e. you must SEE an observation before you can choose what
  to do (e.g. snapshot the scene, see which objects are present, THEN decide
  where to move). Blindly issuing three independent moves one at a time is not
  such a case — batch them into one tree.
- Rule of thumb: if you could write the next 2–5 steps down right now without
  needing to look at anything, they belong in the same tree.

Rules:
1. Use ONLY capabilities listed in the Available capabilities section.
2. In RTDL `do.cap`, use ONLY the listed `capability_name` value. Do NOT use path fragments with `/`, hidden provider ids, contract ids, or natural-language aliases.
3. Build RTDL `do.args` from the listed `args_schema`. Do NOT invent argument keys.
4. Do NOT invent new capabilities, robots, objects, locations, or relations.
5. The value of `rtdl` MUST be a JSON object, not a string.
6. Do not output `out`, variables, expressions, or any operator other than `sequence`, `parallel`, and `do`.
7. If no capability call is needed this round, output an empty sequence: {"op":"sequence","children":[]}.
8. To learn how to use a provider, read its `CAPABILITY.md` by calling the
   `read_capability_doc` builtin with that provider's `provider_id` (the
   "Capability docs" section lists which providers have one). Before the FIRST
   call to any `[skill]` provider, read its CAPABILITY.md first — skills have
   multi-step usage (start → poll status → cancel) the terse description omits;
   for primitives/services it is optional. NEVER use `read_file` for docs and
   NEVER guess or construct a file path (e.g. do not turn `robonix/skill/explore`
   into `/explore/CAPABILITY.md`) — those paths live in the provider's own
   container and are unreadable here. A provider not listed in "Capability docs"
   has no manual; call it directly from its `args_schema`.

Example — dispatch one tree, keep the existing goal (`task_update` null):

{
  "content": "I will inspect the current scene.",
  "rtdl_description": "inspect scene",
  "rtdl": {
    "op": "sequence",
    "children": [
      { "op": "do", "cap": "camera_snapshot", "args": {} }
    ]
  },
  "task_update": null
}

Example — accept a new goal and dispatch a MULTI-STEP tree for it in one round
(the steps are known up front, so do not split them across rounds). The two
independent goals run in `parallel`; the water errand is an ordered `sequence`:

{
  "content": "On it — fetching water and starting music at the same time.",
  "rtdl_description": "fetch water + play music",
  "rtdl": {
    "op": "parallel",
    "children": [
      {
        "op": "sequence",
        "children": [
          { "op": "do", "cap": "navigation_navigate", "args": { "target": "kitchen" } },
          { "op": "do", "cap": "manipulation_grasp", "args": { "object": "water cup" } },
          { "op": "do", "cap": "navigation_navigate", "args": { "target": "user" } }
        ]
      },
      { "op": "do", "cap": "audio_play", "args": { "track": "music" } }
    ]
  },
  "task_update": {
    "goal": "bring the user water and play music at the same time",
    "success_criterion": "a water cup is next to the user AND music is playing",
    "status": "in_progress"
  }
}

Example — split across rounds ONLY when the next step needs the previous result.
Here you must see the snapshot before deciding where to go, so this round emits
just the observation:

{
  "content": "Let me look around first to find the door.",
  "rtdl_description": "look for the door",
  "rtdl": {
    "op": "sequence",
    "children": [
      { "op": "do", "cap": "camera_snapshot", "args": {} }
    ]
  },
  "task_update": null
}

Example — cancel an in-flight tree the user no longer wants, then re-plan:

{
  "content": "Stopping the patrol and coming back to you.",
  "rtdl_description": "return to user",
  "rtdl": {
    "op": "sequence",
    "children": [
      { "op": "do", "cap": "builtin_cancel_plan", "args": { "plan_id": "PASTE_THE_INFLIGHT_PLAN_ID" } },
      { "op": "do", "cap": "navigation_navigate", "args": { "target": "user" } }
    ]
  },
  "task_update": null
}

Example — overall task finished (no new tree, mark done):

{
  "content": "Done — the water is by you and music is playing.",
  "rtdl_description": "",
  "rtdl": { "op": "sequence", "children": [] },
  "task_update": {
    "goal": "bring the user water and play music at the same time",
    "success_criterion": "a water cup is next to the user AND music is playing",
    "status": "done"
  }
}

Example — root `parallel` (Executor runs every child concurrently and waits for all to finish):

{
  "content": "I'll grab a camera frame and query battery status in parallel.",
  "rtdl_description": "snapshot + battery check",
  "rtdl": {
    "op": "parallel",
    "children": [
      { "op": "do", "cap": "camera_snapshot", "args": {} },
      { "op": "do", "cap": "battery_status", "args": {} }
    ]
  },
  "task_update": null
}
