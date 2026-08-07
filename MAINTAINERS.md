# Robonix MAINTAINERS

Per-component ownership for [syswonder/robonix](https://github.com/syswonder/robonix).
Each maintainer is written `Name (@github, email)` so the contact stays bound to
the person; components with two owners separate them with `;`. A public email is
shown only where the person publishes one on their GitHub profile.

- Project: <https://book.robonix.ai>
- Mailing list: robotos@syswonder.org
- Contributor roster: <https://www.syswonder.org>

Lead maintainer: Yulong Han ([@enkerewpo](https://github.com/enkerewpo), wheatfox17@icloud.com)

## System components

| Component | What it does | Path | Maintainer(s) | Issue |
|-----------|--------------|------|---------------|-------|
| atlas | Capability registry and discovery | `system/atlas` | Yulong Han ([@enkerewpo](https://github.com/enkerewpo), wheatfox17@icloud.com) | - |
| executor | RTDL plan execution and capability dispatch | `system/executor` | Guowei Li ([@kouweilee](https://github.com/kouweilee), 2401213322@stu.pku.edu.cn); Longchun Zhao ([@1mujue](https://github.com/1mujue)) | [#52](https://github.com/syswonder/robonix/issues/52) |
| pilot | VLM planning and decision loop | `system/pilot` | Yulong Han ([@enkerewpo](https://github.com/enkerewpo), wheatfox17@icloud.com) | [#53](https://github.com/syswonder/robonix/issues/53) |
| liaison | Human interaction (chat / voice / TUI) | `system/liaison` | Kaile Liu ([@kaileliu](https://github.com/kaileliu)) | [#70](https://github.com/syswonder/robonix/issues/70) |
| scene | Scene state, semantic map, object registry | `system/scene` | Feiyang Li ([@HeartLinked](https://github.com/HeartLinked), lifeiyang@zju.edu.cn) | [#42](https://github.com/syswonder/robonix/issues/42) |
| soma | Robot self-description (body model) | `system/soma` | Yanting Chen ([@Chenyantt](https://github.com/Chenyantt)) | - |
| sentinel | Rule-based safety gate before dispatch | `system/sentinel` | Zhaobo Zhang ([@HustWolfzzb](https://github.com/HustWolfzzb), hustwolfzzb@gmail.com) | [#69](https://github.com/syswonder/robonix/issues/69) |
| scribe | Structured logging / replay / audit | `system/scribe` | Yunlong Hou ([@ohhhHwH](https://github.com/ohhhHwH)) | [#63](https://github.com/syswonder/robonix/issues/63) |
| vitals | Power and component-health monitoring | `system/vitals` | Jinyang Xu ([@AuYang261](https://github.com/AuYang261)) | [#68](https://github.com/syswonder/robonix/issues/68) |
| keystone | Identity, config, access policy | `system/keystone` | Kaile Liu ([@kaileliu](https://github.com/kaileliu)) | [#67](https://github.com/syswonder/robonix/issues/67) |
| nexus | gRPC / MCP / ROS 2 comms libraries | `system/nexus` | Jun Zeng ([@ZZJJWarth](https://github.com/ZZJJWarth)) | - |
| chronos | Unified clock / timestamp alignment | `system/chronos` | _unassigned_ | [#62](https://github.com/syswonder/robonix/issues/62) |

## Services

| Service | What it does | Path | Maintainer(s) |
|---------|--------------|------|---------------|
| memory | Long-term memory: search / save / compact | `services/memsearch` | Yunlong Hou ([@ohhhHwH](https://github.com/ohhhHwH)) |
| speech | ASR / TTS / dialogue (one-shot and streaming) | `services/speech` | Kaile Liu ([@kaileliu](https://github.com/kaileliu)) |
| voiceprint | Speaker enrollment and recognition | `services/voiceprint` | Kaile Liu ([@kaileliu](https://github.com/kaileliu)) |

## Tools and libraries

| Module | What it does | Path | Maintainer(s) |
|--------|--------------|------|---------------|
| rbnx | Robonix CLI: build / boot / codegen / chat / clean | `tools/rbnx` | Yulong Han ([@enkerewpo](https://github.com/enkerewpo), wheatfox17@icloud.com) |
| codegen | Contract IDL to proto / Rust / MCP-type codegen | `tools/codegen` | Yulong Han ([@enkerewpo](https://github.com/enkerewpo), wheatfox17@icloud.com) |
| robonix-api | Python framework: Primitive/Service/Skill bases, atlas client, Driver lifecycle | `pylib/robonix-api` | Yulong Han ([@enkerewpo](https://github.com/enkerewpo), wheatfox17@icloud.com) |

## Upstream packages

Separate repositories, cloned into a deployment's boot cache (`rbnx-boot/cache/`) at boot.

| Package | What it does | Repo | Maintainer(s) |
|---------|--------------|------|---------------|
| explore_rbnx | Autonomous frontier-based exploration skill | [enkerewpo/explore_rbnx](https://github.com/enkerewpo/explore_rbnx) | Yulong Han ([@enkerewpo](https://github.com/enkerewpo), wheatfox17@icloud.com) |
| mapping_rbnx | SLAM / mapping service (`service/map/*`) | [enkerewpo/mapping_rbnx](https://github.com/enkerewpo/mapping_rbnx) | Yulong Han ([@enkerewpo](https://github.com/enkerewpo), wheatfox17@icloud.com) |
| nav2_wrapper_rbnx | Nav2-backed navigation for Ranger Mini (`service/navigation/*`) | [enkerewpo/nav2_wrapper_rbnx](https://github.com/enkerewpo/nav2_wrapper_rbnx) | Yulong Han ([@enkerewpo](https://github.com/enkerewpo), wheatfox17@icloud.com) |
| simple_nav_rbnx | A* + Pure-Pursuit navigation (no Nav2) | [enkerewpo/simple_nav_rbnx](https://github.com/enkerewpo/simple_nav_rbnx) | Yulong Han ([@enkerewpo](https://github.com/enkerewpo), wheatfox17@icloud.com) |

## Reporting an issue

File the problem under the component's tracking issue above (or open a new one),
and `@`-mention the maintainer so they are notified. For an out-of-band reach,
use the public email where listed, otherwise the project mailing list
(robotos@syswonder.org).
