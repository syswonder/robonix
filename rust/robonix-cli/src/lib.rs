// SPDX-License-Identifier: MulanPSL-2.0
// Robonix CLI Library
//
// Core library module for robonix-cli

pub mod config;
pub mod daemon;
pub mod daemon_client;
pub mod daemon_ros2;
pub mod database;
pub mod install;
pub mod output;
pub mod process;
pub mod query;
pub mod recipe;
pub mod recipe_state;
pub mod register;
pub mod task;
pub mod unregister;

pub use config::Config;
pub use database::PackageDatabase;
pub use install::PackageInstaller;
pub use process::ProcessManager;
pub use query::PackageQuery;
pub use recipe::Recipe;
pub use recipe_state::RecipeState;
pub use register::PackageRegistrar;
pub use unregister::PackageUnregistrar;
