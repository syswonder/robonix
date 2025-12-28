// SPDX-License-Identifier: MulanPSL-2.0
// Exception Handler Module
//
// Handles exceptions from skill/service/primitive execution
// and triggers appropriate state machine transitions

use crate::task_manager::context::{ExecutionState, TaskContext};
use log::{debug, error, warn};

/// Exception types from skill/service/primitive execution
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExceptionType {
    /// Object not found (e.g., object was moved)
    ObjectNotFound,
    /// Service unavailable
    ServiceUnavailable,
    /// Skill execution failed
    SkillFailed,
    /// Primitive execution failed
    PrimitiveFailed,
    /// Timeout
    Timeout,
    /// Unknown error
    Unknown(String),
}

/// Exception Handler - Processes exceptions and decides recovery action
pub struct ExceptionHandler;

impl ExceptionHandler {
    /// Handle exception and determine recovery action
    pub fn handle_exception(
        &self,
        context: &mut TaskContext,
        exception_type: ExceptionType,
        error_message: String,
    ) -> RecoveryAction {
        debug!(
            "handling exception for task {}: type={:?}, message={}, current_retry_count={}",
            context.task_id, exception_type, error_message, context.retry_count
        );

        // Check retry count before incrementing
        let current_retry_count = context.retry_count;
        context.record_exception(error_message.clone());
        debug!(
            "task {} retry_count after record_exception: {}",
            context.task_id, context.retry_count
        );

        match exception_type {
            ExceptionType::ObjectNotFound => {
                debug!(
                    "object not found exception: task={}, will replan",
                    context.task_id
                );
                warn!(
                    "object not found exception for task {}: {}",
                    context.task_id, error_message
                );
                // Object was moved - need to replan
                RecoveryAction::Replan
            }
            ExceptionType::ServiceUnavailable => {
                debug!(
                    "service unavailable exception: task={}, retry_count={}/3",
                    context.task_id, current_retry_count
                );
                error!(
                    "service unavailable exception for task {}: {}",
                    context.task_id, error_message
                );
                // Service unavailable - retry with backoff or fail
                if current_retry_count < 3 {
                    debug!(
                        "service unavailable: will retry (retry_count {} < 3)",
                        current_retry_count
                    );
                    RecoveryAction::Retry
                } else {
                    debug!(
                        "service unavailable: will fail (retry_count {} >= 3)",
                        current_retry_count
                    );
                    RecoveryAction::Fail
                }
            }
            ExceptionType::SkillFailed | ExceptionType::PrimitiveFailed => {
                debug!(
                    "execution failed exception: task={}, retry_count={}",
                    context.task_id, current_retry_count
                );
                error!(
                    "execution failed exception for task {}: {}",
                    context.task_id, error_message
                );
                // Execution failed - if skill/primitive not found, abort immediately
                // Otherwise, this is a runtime error that should fail the task
                // No retry for skill/primitive failures - abort task immediately
                debug!("execution failed: will abort task (skill/primitive failure)",);
                RecoveryAction::Fail
            }
            ExceptionType::Timeout => {
                debug!(
                    "timeout exception: task={}, retry_count={}/3",
                    context.task_id, current_retry_count
                );
                warn!(
                    "timeout exception for task {}: {}",
                    context.task_id, error_message
                );
                // Timeout - retry
                if current_retry_count < 3 {
                    debug!(
                        "timeout: will retry (retry_count {} < 3)",
                        current_retry_count
                    );
                    RecoveryAction::Retry
                } else {
                    debug!(
                        "timeout: will fail (retry_count {} >= 3)",
                        current_retry_count
                    );
                    RecoveryAction::Fail
                }
            }
            ExceptionType::Unknown(msg) => {
                debug!(
                    "unknown exception: task={}, msg={}, retry_count={}/2",
                    context.task_id, msg, current_retry_count
                );
                error!("unknown exception for task {}: {}", context.task_id, msg);
                // Unknown error - fail after too many retries
                if current_retry_count < 2 {
                    debug!(
                        "unknown exception: will retry (retry_count {} < 2)",
                        current_retry_count
                    );
                    RecoveryAction::Retry
                } else {
                    debug!(
                        "unknown exception: will fail (retry_count {} >= 2)",
                        current_retry_count
                    );
                    RecoveryAction::Fail
                }
            }
        }
    }
}

/// Recovery action to take after exception
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecoveryAction {
    /// Retry the current operation
    Retry,
    /// Replan the task (call task_plan again with updated object graph)
    Replan,
    /// Fail the task
    Fail,
    /// Continue execution (no action needed)
    Continue,
}

/// Apply recovery action to task context
pub fn apply_recovery_action(context: &mut TaskContext, action: RecoveryAction) {
    debug!(
        "applying recovery action to task {}: {:?}",
        context.task_id, action
    );

    match action {
        RecoveryAction::Retry => {
            debug!(
                "retry action: resetting instruction pointer for task {}",
                context.task_id
            );
            // Reset instruction pointer to retry current instruction
            // (instruction pointer will be decremented before retry)
            if context.rtdl_instruction_pointer > 0 {
                let old_pointer = context.rtdl_instruction_pointer;
                context.rtdl_instruction_pointer -= 1;
                debug!(
                    "task {} instruction pointer reset: {} -> {}",
                    context.task_id, old_pointer, context.rtdl_instruction_pointer
                );
            }
            // Clear exception but keep retry_count for tracking total retries
            context.clear_exception();
            debug!(
                "task {} exception cleared, staying in Execute state",
                context.task_id
            );
            // Stay in Execute state
        }
        RecoveryAction::Replan => {
            debug!(
                "replan action: clearing RTDL and transitioning to Plan state for task {}",
                context.task_id
            );
            // Clear RTDL and go back to Plan state
            context.rtdl = None;
            context.rtdl_type = None;
            context.rtdl_instruction_pointer = 0;
            // Reset retry_count when replanning
            let old_retry_count = context.retry_count;
            context.retry_count = 0;
            debug!(
                "task {} retry_count reset: {} -> 0",
                context.task_id, old_retry_count
            );
            context.clear_exception();
            context.transition_state(ExecutionState::Plan);
        }
        RecoveryAction::Fail => {
            debug!(
                "fail action: transitioning task {} to Failed state",
                context.task_id
            );
            context.transition_state(ExecutionState::Failed);
        }
        RecoveryAction::Continue => {
            debug!(
                "continue action: no state change for task {}",
                context.task_id
            );
            // No action needed
        }
    }
}
