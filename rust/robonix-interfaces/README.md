# robonix-interfaces

This repository is the source of truth for Robonix public interfaces.

## Layout

- `ridl/`
  - handwritten public RIDL interfaces
  - defines `query`, `command`, `stream`, and `event`
  - canonical namespaces live under:
    - `robonix/hal/*`
    - `robonix/system/*`
  - notable system contracts include service-like interfaces and the unified skill invocation contract under `robonix/system/skill/*`
- `lib/`
  - reusable ROS message packages and vendored ROS interface dependencies
  - only reusable data structures belong here
  - do not add `.srv` transport contracts here for new Robonix interfaces

## Rules

- public interaction semantics are defined in RIDL
- reusable payload structures are defined as ROS messages under `lib/`
- new architecture does not require `robonix-sdk`

## Related docs

- package manifest vNext: `../ROBONIX_PACKAGE_MANIFEST.md`
