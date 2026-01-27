use crate::spec::{PrimitiveSpec, ServiceSpec};
use std::collections::HashMap;

pub fn load_primitives(primitives: &mut HashMap<String, PrimitiveSpec>) {
    // AMCL pose primitive - provides PoseWithCovarianceStamped directly from AMCL
    PRM!(primitives, "prm::base.pose.cov", "Get robot pose in map frame from AMCL (PoseWithCovarianceStamped)",
         {},  // No input parameters
         { "pose": "geometry_msgs/msg/PoseWithCovarianceStamped" });
}

pub fn load_services(services: &mut HashMap<String, ServiceSpec>) {
    SRV!(
        services,
        "srv::task_plan",
        "Task planning service converting natural language to RTDL",
        "robonix_sdk/srv/service/task_plan/PlanTask"
    );

    SRV!(
        // TODO
        services,
        "srv::plan_simulate",
        "Plan simulation service for feasibility and safety checking",
        "robonix_sdk/srv/service/plan_simulate/SimulatePlan"
    );

    SRV!(
        // TODO
        services,
        "srv::memory",
        "Cognitive memory service providing long-term and short-term knowledge",
        "robonix_sdk/srv/service/memory/QueryMemory"
    );

    SRV!(
        // TODO
        services,
        "srv::result_feedback",
        "Result feedback service for execution verification",
        "robonix_sdk/srv/service/result_feedback/ResultFeedback"
    );
}
