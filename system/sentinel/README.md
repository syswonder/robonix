# Sentinel - safety supervision

Sentinel is the planned Robonix safety-supervision component. Its job is to
decide whether a requested capability call is allowed under the current robot
state, operator identity, scene context, and policy.

## Current status

Sentinel is not implemented on `dev` yet. There is no `robonix-sentinel`
binary, no `system/executor/src/dispatch/sentinel.rs`, and no `sentinel.yaml`
rule loader in Executor.

Executor currently dispatches validated RTDL `do` nodes directly through Atlas
to the selected primitive, service, or skill provider. The only safety-related
controls in the current path are local to the components that already exist,
for example:

- Executor plan cancellation and stop points (`cancel_plan`, `stop_plan_at`,
  `cancel_all_plans`).
- Liaison access control for text/API users and voiceprint-verified speakers
  before work enters Pilot/TTS.
- Provider-side validation in individual primitives and services.

Do not rely on this directory for runtime enforcement today.

## Intended role

When Sentinel lands, it should sit on the capability-call path before Executor
dispatches side-effecting work. The policy decision should be based on:

- the requested contract and arguments,
- the operator identity and permissions from [keystone](../keystone/),
- current robot/body health from [vitals](../vitals/),
- current environment state from [scene](../scene/),
- explicit safety policy such as allow/deny lists, rate limits, workspace
  limits, stop windows, and emergency-stop state.
