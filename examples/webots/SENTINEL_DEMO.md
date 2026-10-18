# Webots Sentinel demo policy

Both Webots manifests declare two paths:

- `sentinel_rules` is the robot-local policy managed by Sentinel.
- `sentinel_rules_seed` is the checked-in `sentinel-rules.v1.json` baseline.

## Management endpoint

Both Webots manifests bind Sentinel management to `127.0.0.1:50062`. Only an
authenticated administrator on the robot host can list or replace rules.

On the first boot, Executor validates the versioned seed and copies it to the
robot-local path. Later boots never overwrite that file, so rule replacements
made through the administrator API survive rebuilds and alternate worktrees.
To reproduce a fresh demo, move or back up the existing
`~/.robonix/data/sentinel-rules.json` and then run:

```bash
rbnx boot -f examples/webots/robonix_manifest.sentinel-demo.yaml
```

The v1 seed allows Keystone administrators to invoke `robonix/skill/*` and
denies that namespace for everyone else. Calls outside that namespace keep
Sentinel's overlay behavior: they are allowed when no rule matches.

The seed makes no assumptions about movement state, gripper layout, or
component IDs. Deployment-specific argument and trusted-state conditions use
typed RFC 6901 JSON Pointer predicates.

## Scene condition example

Executor exposes a fresh `get_robot_context` response below `/scene/data`.
Current Scene deployments use `region_id` and `region_name`; the former is the
stable value returned by `robonix/system/scene/list_regions`. For example, this
rule denies navigation while the robot is in one selected region:

```json
{
  "id": "deny-navigation-from-selected-region-v1",
  "effect": "deny",
  "priority": 50,
  "conditions": {
    "contract": "robonix/service/navigation/navigate",
    "context": [
      {
        "path": "/scene/data/region_id",
        "kind": "string",
        "operator": "eq",
        "value": "scene.region.anno.315"
      }
    ]
  },
  "reason": "Navigation is disabled in the selected region"
}
```

Replace the example ID with one returned by Scene. If Scene is unavailable or
its robot pose is stale, this matching navigation rule is indeterminate and
Sentinel denies the call.
