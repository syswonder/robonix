# Changelog

All notable changes to this project will be documented in this file. The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **CLI (rbnx):** A deployment entry can say `status: disabled` (`enabled`, or no `status`, is on) to stay in the manifest, its `url`, `branch` and `config` kept, while nothing builds, starts or updates it; `rbnx boot` lists it as skipped, soma does not start it. Of the built-in system components pilot, liaison and vitals can be disabled; atlas, executor and soma cannot.
- **Config specs:** A package's `config.spec` may now be strict: YAML with `specVersion: 1` and a small JSON Schema subset (`type`, `description`, `default`, `enum`, `minimum`, `maximum`, `items`, nested `properties`, `required`) plus `x-secret`, `x-group` and `x-provider` (a field that names another entry of the deployment, and the contract it must provide), from which tools such as Robonix Hub build the configuration form. Free-text specs are still accepted. Every system and bundled service now ships a strict spec, checked by `rbnx`'s tests; writing them corrected several defaults and accepted values the old text documented wrongly (Speech, Voiceprint, Scene).

## [1.1.0-beta.1] - 2026-10-01

Pre-release for the invite-only beta of [Robonix Hub](https://hub.robonix.ai), the web console that installs, configures and runs Robonix deployments on a robot. Hub installs this version. From this release on, every Rust crate and Python package carries the release version; Python spells it `1.1.0b1`.

### Added

- **Scene:** Add DualMap as a selectable perception backend, and use it by default where the image carries it (experimental). [#243](https://github.com/syswonder/robonix/pull/243)
- **Scene:** Select a perception profile (lite, full, annotate) from the manifest. [#243](https://github.com/syswonder/robonix/pull/243)
- **Scene:** Add object-correction contracts, also as MCP tools, and evict objects that are visibly gone from the upstream ConceptGraphs map. [#234](https://github.com/syswonder/robonix/pull/234)
- **Scene:** Draw the semantic map with rerun, in a web UI that shows the map rather than the registry. [#248](https://github.com/syswonder/robonix/pull/248)
- **Scene:** Add room-aware semantic goals. [#228](https://github.com/syswonder/robonix/pull/228)
- **Executor:** Add a result-verification framework, a VLM result verifier, and a scene verifier that checks the reached position and yaw. [#242](https://github.com/syswonder/robonix/pull/242)
- **Executor:** Add a VERIFYING state and overlapping verification rules. [#251](https://github.com/syswonder/robonix/pull/251)
- **Capability Contracts and Code Generation:** Add `llm_callable` metadata to control which contracts the planner sees. [#242](https://github.com/syswonder/robonix/pull/242)
- **Pilot:** Archive each session to a transcript and restore it after a restart. [#250](https://github.com/syswonder/robonix/pull/250)
- **Pilot:** Hand the planner a current camera frame each round. [#250](https://github.com/syswonder/robonix/pull/250)
- **Pilot:** Resolve namespaced provider model profiles and require verified model context budgets. [#250](https://github.com/syswonder/robonix/pull/250)
- **Pilot:** Keep the stream alive while a reasoning model thinks. [#250](https://github.com/syswonder/robonix/pull/250)
- **Liaison:** Let a client end voice capture and submit early. [#232](https://github.com/syswonder/robonix/pull/232)
- **Liaison:** Stream hands-free voice events to clients. [#228](https://github.com/syswonder/robonix/pull/228)
- **CLI (rbnx):** Record which package commits a deployment actually ran. [#247](https://github.com/syswonder/robonix/pull/247)
- **CLI (rbnx):** Give components a session directory and keep it on `rbnx clean`. [#250](https://github.com/syswonder/robonix/pull/250)
- **Simulation (Webots):** Register the client audio bridge beside ALSA audio. [#228](https://github.com/syswonder/robonix/pull/228)
- **Simulation (Webots):** Publish the simulator's own robot pose as an evaluation reference. [#243](https://github.com/syswonder/robonix/pull/243)

### Changed

- **Runtime:** Standardize provider lifecycle management; every provider needs a lifecycle driver. [#228](https://github.com/syswonder/robonix/pull/228)
- **Pilot:** Keep per-task context in history bounded by tokens, with a cache-stable prompt layout and bounded history compaction. [#250](https://github.com/syswonder/robonix/pull/250)
- **Pilot:** Describe the RTDL envelope to the provider as a JSON schema. [#250](https://github.com/syswonder/robonix/pull/250)
- **Workspace:** Version the Rust crates and Python packages with the release. Through 1.0.0 the crates read 0.1.0 and the Python packages 0.0.0 to 0.1.0rc1.

### Fixed

- **Runtime:** Report provider exits immediately, reap failed providers, and harden boot cleanup and task cancellation. [#228](https://github.com/syswonder/robonix/pull/228)
- **Scene:** Reuse unchanged VLM inference. [#231](https://github.com/syswonder/robonix/pull/231)
- **Scene:** Survive malformed upstream messages and keep the unauthenticated web UI on loopback by default. [#245](https://github.com/syswonder/robonix/pull/245)
- **Scene:** Place and photograph each object from the frame it was seen in. [1f437909](https://github.com/syswonder/robonix/commit/1f437909995c5f2aef468f9b634982254b7b6115)
- **Speech and Audio:** Split long Tencent TTS input. [#230](https://github.com/syswonder/robonix/pull/230)
- **Scribe:** Build on non-Unix targets. [#236](https://github.com/syswonder/robonix/pull/236)
- **Build:** Build on LoongArch64 by honoring a system `protoc`. [#249](https://github.com/syswonder/robonix/pull/249)
- **Simulation (Webots):** Load remote viewer assets through the proxy. [#256](https://github.com/syswonder/robonix/pull/256)
- **Simulation (Webots):** Give the registered depth image the optical frame it is registered to. [#240](https://github.com/syswonder/robonix/pull/240)
- **Testing:** Wait for a usable map before the post-restart scenario pass. [#244](https://github.com/syswonder/robonix/pull/244)
- **Quickstart:** Let a freshly installed machine run the quickstart. [#248](https://github.com/syswonder/robonix/pull/248)

## [1.0.0] - 2026-08-15

First stable release of Robonix, an operating system for embodied artificial intelligence.

### Added

- **Capability Contracts and Code Generation:** Add package manifests specification. [#27](https://github.com/syswonder/robonix/pull/27)
- **Capability Contracts and Code Generation:** Add capability contracts for primitives, services, and system components. [#27](https://github.com/syswonder/robonix/pull/27)
- **Capability Contracts and Code Generation:** Add contract and ROS IDL code generation for Python and Rust (`rbnx codegen`). [0c4da9b2](https://github.com/syswonder/robonix/commit/0c4da9b21d8aee9698ce0536b0fe92e88b88742a)
- **Capability Contracts and Code Generation:** Add a canonical ROS 2 interface overlay for generated code. [#58](https://github.com/syswonder/robonix/pull/58)
- **Capability Contracts and Code Generation:** Add a contract and ROS IDL reference generator. [#60](https://github.com/syswonder/robonix/pull/60)
- **Capability Contracts and Code Generation:** Add arm primitive contracts. [#142](https://github.com/syswonder/robonix/pull/142)
- **Capability Contracts and Code Generation:** Add dexterous-hand primitive contracts. [#119](https://github.com/syswonder/robonix/pull/119)
- **Capability Contracts and Code Generation:** Add quadruped primitive contracts. [#154](https://github.com/syswonder/robonix/pull/154)
- **Python API (robonix-api):** Publish the robonix-api Python library for building capability providers on PyPI. [#43](https://github.com/syswonder/robonix/pull/43)
- **Atlas:** Add capability registration. [#27](https://github.com/syswonder/robonix/pull/27)
- **Atlas:** Add capability discovery. [#27](https://github.com/syswonder/robonix/pull/27)
- **Atlas:** Add capability lifecycle management. [#31](https://github.com/syswonder/robonix/pull/31)
- **Atlas:** Add package deployment integration. [#27](https://github.com/syswonder/robonix/pull/27)
- **Atlas:** Allow advisory provider namespaces for heterogeneous deployments. [#141](https://github.com/syswonder/robonix/pull/141)
- **Pilot:** Add RTDL planning. [#37](https://github.com/syswonder/robonix/pull/37)
- **Pilot:** Add RTDL forest execution. [#92](https://github.com/syswonder/robonix/pull/92)
- **Pilot:** Add live task graphs. [#92](https://github.com/syswonder/robonix/pull/92)
- **Pilot:** Add mid-task steering. [#92](https://github.com/syswonder/robonix/pull/92)
- **Executor:** Add parallel operation support. [#38](https://github.com/syswonder/robonix/pull/38)
- **Executor:** Add targeted RTDL plan cancellation. [#110](https://github.com/syswonder/robonix/pull/110)
- **Navigation:** Add dynamic speed-limit contracts. [#187](https://github.com/syswonder/robonix/pull/187)
- **Soma:** Add the Soma robot self-description subsystem. [#109](https://github.com/syswonder/robonix/pull/109)
- **Soma:** Share the manifest lifecycle and publish the robot footprint. [771b457d](https://github.com/syswonder/robonix/commit/771b457dc68cc3822f4f7910845a84d1cb5c324e)
- **Vitals:** Add robot health-stream ingestion. [#111](https://github.com/syswonder/robonix/pull/111)
- **Vitals:** Add component-tree normalization. [#111](https://github.com/syswonder/robonix/pull/111)
- **Vitals:** Add health monitoring. [#219](https://github.com/syswonder/robonix/pull/219)
- **Scene:** Add scene-graph relation inference. [#39](https://github.com/syswonder/robonix/pull/39)
- **Scene:** Add stable object persistence across restarts. [#78](https://github.com/syswonder/robonix/pull/78)
- **Scene:** Add hardware-aware perception. [#89](https://github.com/syswonder/robonix/pull/89)
- **Scene:** Add VLM-primary scene graphs. [#116](https://github.com/syswonder/robonix/pull/116)
- **Scene:** Add map-scoped persistent state. [#129](https://github.com/syswonder/robonix/pull/129)
- **Scene:** Add room and object save/restore. [#137](https://github.com/syswonder/robonix/pull/137)
- **Scene:** Add save-token-scoped semantic-map persistence. [#172](https://github.com/syswonder/robonix/pull/172)
- **Scene:** Add stable room-region queries. [#179](https://github.com/syswonder/robonix/pull/179)
- **Scene:** Expose the robot yaw through `list_objects`. [0b2ba7a2](https://github.com/syswonder/robonix/commit/0b2ba7a2998725d24a7af5b7bf997d844be5647a)
- **Liaison:** Add voiceprint identification. [#47](https://github.com/syswonder/robonix/pull/47)
- **Liaison:** Add remote voice access. [#115](https://github.com/syswonder/robonix/pull/115)
- **Liaison:** Add hands-free interaction. [#139](https://github.com/syswonder/robonix/pull/139)
- **Liaison:** Add task interruption. [#139](https://github.com/syswonder/robonix/pull/139)
- **Speech and Audio:** Add `speak`/`list_speakers` MCP tools. [ee45aa6a](https://github.com/syswonder/robonix/commit/ee45aa6a19edd2ce0b1d326e696e1bd2c27881f2)
- **Memory:** Add the memgraph structured memory service: a causal knowledge-graph (CKG) with tag-indexed, vector-searchable nodes, scene-watch memory building, and `remember`/`hybrid_search`/`promote` contracts. [#171](https://github.com/syswonder/robonix/pull/171)
- **Memory:** Add the memsearch vector-search memory backend. [#22](https://github.com/syswonder/robonix/pull/22)
- **Memory:** Auto-detect the ONNX/Ollama embedding backend. [0845dc75](https://github.com/syswonder/robonix/commit/0845dc751336902ff86b2ad42c1cab78e7701870)
- **Scribe:** Add structured system journaling. [#93](https://github.com/syswonder/robonix/pull/93)
- **Deployment:** Add ARM64 deployment support. [#106](https://github.com/syswonder/robonix/pull/106)
- **Deployment:** Add Jetson deployment support. [#106](https://github.com/syswonder/robonix/pull/106)
- **Deployment:** Add Webots simulation scenarios. [#132](https://github.com/syswonder/robonix/pull/132)
- **Deployment:** Add GPU-optimized browser streaming. [#144](https://github.com/syswonder/robonix/pull/144)
- **Deployment:** Add self-contained deployments. [#146](https://github.com/syswonder/robonix/pull/146)
- **CLI (rbnx):** Add `rbnx init` and `rbnx package-new` commands. [#29](https://github.com/syswonder/robonix/pull/29)
- **CLI (rbnx):** Add `rbnx update` with remote-provider freshness checks. [7b54a1fc](https://github.com/syswonder/robonix/commit/7b54a1fcad4fe41d732fd0d12a90680bf2bc6adf)
- **CLI (rbnx):** Select package manifests per deployment target. [f15e2e42](https://github.com/syswonder/robonix/commit/f15e2e42c5b5ca6e653fe169708b07d3f1cbe559)
- **CLI (rbnx):** Scaffold robot deployment manifests. [18d1e78d](https://github.com/syswonder/robonix/commit/18d1e78d7a83ef1bde2170a3995c037fda2ca0e4)
- **CLI (rbnx):** Support offline deployment builds. [#163](https://github.com/syswonder/robonix/pull/163)
- **Simulation (Webots):** Add speech and voiceprint to the base simulation deployment. [a6541c2b](https://github.com/syswonder/robonix/commit/a6541c2bca85da2e8576e63077b8f4b63f61971e)
- **Packages:** Document published robot deployments. [#192](https://github.com/syswonder/robonix/pull/192)
- **Packages:** Document the Webots TIAGo deployment. [efec93b9](https://github.com/syswonder/robonix/commit/efec93b93d76f98e04e63e4b456c60c1bdc0781e)
- **Packages:** Document the package catalog API and contribution flow. [91f1051d](https://github.com/syswonder/robonix/commit/91f1051d7fa621820253fcb3958473a0955449a7), [1e4e4fa1](https://github.com/syswonder/robonix/commit/1e4e4fa177d47467839476fbdf4c86e617f63b3d)

### Changed

- **Atlas:** Adopt the capability-provider contract model (`provider_id`/`provider_kind`, CapabilityProvider/Capability). [27e83d40](https://github.com/syswonder/robonix/commit/27e83d40ad9e616942ae0e482176a630bdc9a41a), [aa57440e](https://github.com/syswonder/robonix/commit/aa57440ee807297255909c3f88cd787aede05cf9)
- **Pilot:** Make planning state-aware. [#174](https://github.com/syswonder/robonix/pull/174)
- **Pilot:** Reduce repeated static planning context. [#210](https://github.com/syswonder/robonix/pull/210)
- **Navigation:** Drive simulated navigation with nav2 and expose an rviz goal relay. [2515f259](https://github.com/syswonder/robonix/commit/2515f259b83942853a076e270ed39b419b4bd38e), [6aff678d](https://github.com/syswonder/robonix/commit/6aff678d0068199f12b676864f1e77f4559e0d03)
- **Scene:** Make the ROS distribution a build-time choice. [403b0ed9](https://github.com/syswonder/robonix/commit/403b0ed9593393588c08321386da41d95b2fe3c0)
- **Scene:** Speed up goal-room queries by stopping at the nearest safe cell. [e180e60e](https://github.com/syswonder/robonix/commit/e180e60e69e13d5282788ae49547bc85b724e239)

### Fixed

- **Atlas:** Mint valid ROS 2 collision endpoints. [5b4ed3d3](https://github.com/syswonder/robonix/commit/5b4ed3d31e128ffc4368d9aa1df351bf1d1218ef)
- **Pilot:** Improve RTDL control reliability. [#174](https://github.com/syswonder/robonix/pull/174)
- **Pilot:** Retain dispatched RTDL plans across replans. [583d1e0b](https://github.com/syswonder/robonix/commit/583d1e0bc37f3bd5d65c892479d8e1f59241f340)
- **Pilot:** Keep the URDF out of the system prompt. [af7f2281](https://github.com/syswonder/robonix/commit/af7f22812ffab8dd6e621ffc5760389b78242081)
- **Navigation:** Configure the simulated navigation speed policy. [7d0ae28c](https://github.com/syswonder/robonix/commit/7d0ae28c6028fc90af8ddfe746ce41fb5ef54e8b)
- **Scene:** Retain reliable occupancy maps for late subscribers. [f9e16ea9](https://github.com/syswonder/robonix/commit/f9e16ea9e8c566ff51bab3a9be314a05137f3b71)
- **Speech and Audio:** Refresh the Tencent ASR runtime configuration and bypass the ambient proxy. [669d4390](https://github.com/syswonder/robonix/commit/669d4390406e5ae0d1fdeb9eb70ebe2d7499c529), [d0eae744](https://github.com/syswonder/robonix/commit/d0eae74462fad8e5f6042613b5f5e9133ac0bff1)
- **Speech and Audio:** Suppress duplicated revised ASR transcripts. [94001d38](https://github.com/syswonder/robonix/commit/94001d384e94abb4da7261cc073c9e8a198acf35)
- **Speech and Audio:** Load prewarmed voiceprint checkpoints locally (ModelScope; JetPack Torch on aarch64). [6312a4d2](https://github.com/syswonder/robonix/commit/6312a4d29118a096b38d9b98619d50ea0c471144), [eb93edb9](https://github.com/syswonder/robonix/commit/eb93edb912ca0795af954607c8d6c868cda03c34), [5ba9e6e6](https://github.com/syswonder/robonix/commit/5ba9e6e6a657906a8bc1a26dbab94079247e96a7)
- **Speech and Audio:** Bind the default speaker and preserve the deploy environment on shutdown. [895f8152](https://github.com/syswonder/robonix/commit/895f8152b91211578798d2641ff59efc5747c777)
- **Speech and Audio:** Route the macOS audio bridge over a direct WebSocket. [3fe683e1](https://github.com/syswonder/robonix/commit/3fe683e1dba1ccc808dd8f19fe885d7f52ab6fa1)
- **Memory:** Surface memsearch startup logging and warm the embedding model at build time. [#114](https://github.com/syswonder/robonix/pull/114)
- **Memory:** Configure ONNX Runtime safely on Jetson. [93bf9b5d](https://github.com/syswonder/robonix/commit/93bf9b5da762e94f9610662722ef87afc2c8a3ef)
- **Memory:** Do not crash the memory service on a stale Scene Hook port. [bbd990b6](https://github.com/syswonder/robonix/commit/bbd990b630759fa1c5768c7dfcf240f6a3c522e6)
- **CLI (rbnx):** Prebuild system packages before registration. [7b13889d](https://github.com/syswonder/robonix/commit/7b13889d9fa2dd31a4fcb470ee0e85a7b6ad6de9)
- **CLI (rbnx):** Preserve colcon overlays during codegen and apply the deploy environment consistently. [cf43e978](https://github.com/syswonder/robonix/commit/cf43e9783958df735bc77d28e8c2caf0d178c650), [fdf85218](https://github.com/syswonder/robonix/commit/fdf852183ecce0a7de0285c4e0ea424005d6d774)
- **Simulation (Webots):** Stabilize multimodal RTAB-Map mapping and preserve depth-only obstacles. [76771012](https://github.com/syswonder/robonix/commit/76771012e3412042f413440ae87ee3c5fcd10777), [a81438d5](https://github.com/syswonder/robonix/commit/a81438d59d6d3dae856da185d463e4ea49fe8dac), [c98e5314](https://github.com/syswonder/robonix/commit/c98e5314ae2d5c037b611ca44579d2748eadf388)
- **Testing:** Stop swallowing timeout failures behind a NameError. [fcd3da15](https://github.com/syswonder/robonix/commit/fcd3da1527563b465bde3199f8dfb815f023a576)
