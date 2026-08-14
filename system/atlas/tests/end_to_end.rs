// SPDX-License-Identifier: MulanPSL-2.0
//
// TODO(task #31): rewrite end-to-end tests against the new Atlas API
// (RegisterPrimitive/Service/Skill / DeclareCapability / Query) once the
// `robonix-proto` crate lands. The previous version drove Atlas through
// `robonix-sdk` — both the SDK and the old Node-* wire schema have been
// retired in this branch, so the tests would not compile against the new
// proto. Original logic preserved in git history before the commit that
// rewrote atlas.proto + service.rs.
