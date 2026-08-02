# Webots Sentinel demo policy

Both Webots manifests declare two paths:

- `sentinel_rules` is the robot-local policy managed by Sentinel.
- `sentinel_rules_seed` is the checked-in `sentinel-rules.v1.json` baseline.

On the first boot, Executor validates the versioned seed and copies it to the
robot-local path. Later boots never overwrite that file, so rule replacements
made through the administrator API survive rebuilds and alternate worktrees.
To reproduce a completely fresh demo, move or back up the existing
`~/.robonix/data/sentinel-rules.json` and then run:

```bash
rbnx boot -f examples/webots/robonix_manifest.sentinel-demo.yaml
```

The v1 seed allows Keystone administrators to invoke `robonix/skill/*` and
denies that namespace for everyone else. Calls outside that namespace keep
Sentinel's documented overlay behavior: they are allowed when no rule matches.

The seed intentionally makes no assumptions about a robot's movement state,
gripper layout, or component IDs. Deployment-specific argument and trusted
state conditions continue to use typed RFC 6901 JSON Pointer predicates.
