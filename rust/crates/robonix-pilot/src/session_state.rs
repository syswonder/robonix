// SPDX-License-Identifier: MulanPSL-2.0
// SessionState values match `lib/pilot/msg/SessionStatusEvent.msg` / `SessionInfo.msg`.

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[repr(u32)]
pub enum SessionState {
    #[default]
    Active = 0,
    Completed = 1,
    Failed = 2,
    WaitingInput = 3,
}
