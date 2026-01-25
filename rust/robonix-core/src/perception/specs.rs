use crate::spec::{PrimitiveSpec, ServiceSpec};
use std::collections::HashMap;

pub fn load_primitives(primitives: &mut HashMap<String, PrimitiveSpec>) {
    PRM!(primitives, "prm::camera.capture", "Capture RGB image from camera",
         {},  // No input parameters
         { "image": "sensor_msgs/msg/Image" });

    PRM!(primitives, "prm::lidar.scan", "Scan environment with lidar",
         {},  // No input parameters
         { "scan": "sensor_msgs/msg/LaserScan" });

    PRM!(primitives, "prm::camera.depth", "Capture depth image from depth camera",
         {},  // No input parameters
         { "depth": "sensor_msgs/msg/Image" });
}

pub fn load_services(services: &mut HashMap<String, ServiceSpec>) {
    SRV!(
        services,
        "srv::spatial_map",
        "Spatial map service providing geometric structure information",
        "robonix_sdk/srv/service/spatial_map/GetSpatialMap"
    );

    SRV!(
        services,
        "srv::semantic_map",
        "Semantic map service providing entity-level representation using VLM for object detection and depth camera for accurate distance measurement",
        "robonix_sdk/srv/service/semantic_map/QuerySemanticMap"
    );
}
