use crate::mgmt::ManagementModule;
use crate::perception::PerceptionModule;
use crate::planning::PlanningModule;
use std::sync::Arc;

// Robonix core - coordinates all modules
pub struct RobonixCore {
    mgmt: Arc<ManagementModule>,
    perception: Arc<PerceptionModule>,
    planning: Arc<PlanningModule>,
}

impl RobonixCore {
    pub fn new() -> Self {
        let mgmt = Arc::new(ManagementModule::new());
        let perception = Arc::new(PerceptionModule::new());
        let planning = Arc::new(PlanningModule::new());

        Self {
            mgmt,
            perception,
            planning,
        }
    }

    pub fn get_mgmt(&self) -> Arc<ManagementModule> {
        self.mgmt.clone()
    }

    pub fn get_perception(&self) -> Arc<PerceptionModule> {
        self.perception.clone()
    }

    pub fn get_planning(&self) -> Arc<PlanningModule> {
        self.planning.clone()
    }

    /// Register capability or skill (delegates to management module)
    pub async fn register(&self, req: crate::messages::RegisterCapSklRequest) -> crate::messages::RegisterCapSklResponse {
        let resp = self.mgmt.register(req.clone()).await;
        
        // Notify perception module if skill was registered
        if resp.success && req.package_type == "skl" {
            self.perception.on_skill_registered(&req.std_name).await;
        }
        
        resp
    }

    /// Query capability or skill (delegates to management module)
    pub async fn query(&self, req: crate::messages::QueryCapSklRequest) -> crate::messages::QueryCapSklResponse {
        self.mgmt.query(req).await
    }
}
