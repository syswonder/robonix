use crate::spec::{PrimitiveSpec, ServiceSpec};
use std::collections::HashMap;

pub fn load_primitives(primitives: &mut HashMap<String, PrimitiveSpec>) {
    PRM!(primitives, "prm::camera.rgb", "Capture RGB image from camera",
         {},  // No input parameters
         { "image": "sensor_msgs/msg/Image" });

    PRM!(primitives, "prm::lidar.scan", "Scan environment with lidar",
         {},  // No input parameters
         { "scan": "sensor_msgs/msg/LaserScan" });

    PRM!(primitives, "prm::camera.depth", "Capture depth image from depth camera",
         {},  // No input parameters
         { "depth": "sensor_msgs/msg/Image" });

    PRM!(primitives, "prm::sensor.pointcloud", "Capture point cloud data from 3D sensor",
         {},  // No input parameters
         { "pointcloud": "sensor_msgs/msg/PointCloud2" });

    PRM!(primitives, "prm::camera.rgbd", "Capture RGB and depth images from RGBD camera",
         {},  // No input parameters
         { "rgb": "sensor_msgs/msg/Image", "depth": "sensor_msgs/msg/Image" });

    PRM!(primitives, "prm::trasform.laserscan", "Transform point cloud to laser scan",
         { "pointcloud": "sensor_msgs/msg/PointCloud2" },
         { "scan": "sensor_msgs/msg/LaserScan" });

    PRM!(primitives, "prm::description.urdf", "Provide robot URDF description",
         {},  // No input parameters
         { "urdf": "std_msgs/msg/String" }); // robot_description parameter as string

    PRM!(primitives, "prm::slam.vision", "Visual SLAM providing map and localization",
         {},  // No input parameters (subscribes to RGB, depth, pointcloud internally)
         { "map": "nav_msgs/msg/OccupancyGrid" }); // Output map
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
