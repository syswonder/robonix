use crate::spec::ServiceSpec;
use std::collections::HashMap;

pub fn load_services(services: &mut HashMap<String, ServiceSpec>) {
    SRV!(
        services,
        "task_plan",
        "Task planning service converting natural language to RTDL",
        "robonix_sdk/srv/service/task_plan/PlanTask"
    );

    SRV!(
        // TODO
        services,
        "plan_simulate",
        "Plan simulation service for feasibility and safety checking",
        "robonix_sdk/srv/service/plan_simulate/SimulatePlan"
    );

    SRV!(
        // TODO
        services,
        "memory",
        "Cognitive memory service providing long-term and short-term knowledge",
        "robonix_sdk/srv/service/memory/QueryMemory"
    );

    SRV!(
        // TODO
        services,
        "result_feedback",
        "Result feedback service for execution verification",
        "robonix_sdk/srv/service/result_feedback/ResultFeedback"
    );
}
