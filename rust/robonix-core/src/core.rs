use crate::action::ActionModule;
use crate::mgmt::ManagementModule;
use crate::perception::PerceptionModule;
use crate::planning::PlanningModule;
use std::sync::Arc;

// Robonix core - coordinates all modules
pub struct RobonixCore {
    mgmt: Arc<ManagementModule>,
    perception: Arc<PerceptionModule>,
    planning: Arc<PlanningModule>,
    action: Arc<ActionModule>,
}

impl RobonixCore {
    pub fn new() -> Self {
        let mgmt = Arc::new(ManagementModule::new());
        let perception = Arc::new(PerceptionModule::new());
        
        // Create planning module and connect it
        let mut planning = PlanningModule::new();
        planning.set_mgmt(mgmt.clone());
        planning.set_perception(perception.clone());
        let planning = Arc::new(planning);
        
        // Create action module and connect it
        let mut action = ActionModule::new();
        action.set_mgmt(mgmt.clone());
        let action = Arc::new(action);

        Self {
            mgmt,
            perception,
            planning,
            action,
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

    pub fn get_action(&self) -> Arc<ActionModule> {
        self.action.clone()
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
