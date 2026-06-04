// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// Flat `CapabilityCallEvent` (`lib/executor/msg/CapabilityCallEvent.msg`).

use crate::pb::executor::{BatchComplete, CapabilityCallEvent, CapabilityCallStarted};
use crate::pb::pilot::CapabilityCallResult;

pub const EVT_STARTED: u32 = 0;
pub const EVT_RESULT: u32 = 1;
pub const EVT_COMPLETE: u32 = 2;

pub fn started(call_id: String, provider_id: String, contract_id: String) -> CapabilityCallEvent {
    CapabilityCallEvent {
        event_kind: EVT_STARTED,
        started: Some(CapabilityCallStarted {
            call_id,
            provider_id,
            contract_id,
        }),
        ..Default::default()
    }
}

pub fn result(r: CapabilityCallResult) -> CapabilityCallEvent {
    CapabilityCallEvent {
        event_kind: EVT_RESULT,
        result: Some(r),
        ..Default::default()
    }
}

pub fn complete(plan_id: String, any_failed: bool) -> CapabilityCallEvent {
    CapabilityCallEvent {
        event_kind: EVT_COMPLETE,
        complete: Some(BatchComplete {
            plan_id,
            any_failed,
        }),
        ..Default::default()
    }
}
