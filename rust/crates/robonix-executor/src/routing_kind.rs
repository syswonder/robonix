// SPDX-License-Identifier: MulanPSL-2.0
// Matches `lib/pilot/msg/ToolRouting.msg` kind field.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum RoutingKind {
    Builtin = 0,
    Mcp = 1,
    Grpc = 2,
}

impl RoutingKind {
    pub fn from_wire(v: u32) -> Self {
        match v {
            1 => Self::Mcp,
            2 => Self::Grpc,
            _ => Self::Builtin,
        }
    }
}
