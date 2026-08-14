// SPDX-License-Identifier: MulanPSL-2.0

pub mod config;
pub mod deployment;
pub mod health;
pub mod launcher;
pub mod pb;
pub mod report;
pub mod runtime_monitor;
pub mod runtime_state;
pub mod service;
pub mod store;

pub const GET_YAML_CONTRACT: &str = "robonix/system/soma/get_yaml";
pub const GET_URDF_CONTRACT: &str = "robonix/system/soma/get_urdf";
pub const GET_FOOTPRINT_CONTRACT: &str = "robonix/system/soma/footprint";
pub const GET_HEALTH_CONTRACT: &str = "robonix/system/soma/get_health";
pub const HEALTH_CONTRACT: &str = "robonix/system/soma/health";
pub const SOMA_NAMESPACE: &str = "robonix/system/soma";
