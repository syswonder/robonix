---
name: Bug Report
about: Report a bug in Robonix
title: '[BUG] '
labels: 'type:bug'
assignees: ''

---

## Bug description
A clear, one-paragraph description of what's wrong.

## Affected component
Tick the closest match — maintainers confirm the final `comp:*` label.

**System** (core OS services)
- [ ] atlas (capability registry)
- [ ] executor
- [ ] pilot (planner / VLM)
- [ ] liaison (voice / user I/O)
- [ ] scene (semantic + spatial map)
- [ ] sentinel (safety) / soma (body model) / scribe (logging) / chronos / keystone / vitals / nexus

**Service** (scene-level capabilities)
- [ ] mapping (`service/map`)
- [ ] navigation
- [ ] speech (ASR / TTS)
- [ ] voiceprint
- [ ] memory

**Skill / Primitive**
- [ ] skill: `__________` (e.g. explore)
- [ ] primitive: `__________` (camera / chassis / lidar / audio / arm)

**Tooling / cross-cutting**
- [ ] rbnx (CLI) / codegen / robonix-api (pylib) / capabilities & contracts / docs / ci
- [ ] Other / not sure

## Steps to reproduce
1. 
2. 
3. 

## Expected behavior
What should happen instead.

## Actual behavior
What actually happens. Include `rbnx caps` output if a provider is in ERROR.

## Environment
- OS / kernel: [e.g. Ubuntu 22.04, Linux 6.8 Jetson Thor]
- rbnx version: [`rbnx --version`]
- ROS 2 distro: [e.g. Humble] — only if a ROS-backed component is involved
- Deployment: [ ] webots sim  [ ] real robot — manifest: [e.g. `examples/webots/robonix_manifest.yaml`]
- Terminal (for `rbnx chat` / TUI issues): [e.g. VSCode integrated / gnome-terminal / TTY]
- Hardware: [robot base, camera, mic, …]

## Logs
```
# relevant lines from rbnx-boot/logs/<component>.log, or `docker logs <container>`
```

## Additional context
Anything else — screenshots, related issues / PRs, suspected cause.
