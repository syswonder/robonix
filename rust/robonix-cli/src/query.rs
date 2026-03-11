// SPDX-License-Identifier: MulanPSL-2.0
// Query Module
//
// Package query functionality for robonix-cli

use crate::config::Config;
use crate::database::PackageDatabase;
use anyhow::Result;

pub struct PackageQuery {
    config: Config,
}

impl PackageQuery {
    pub fn new(config: Config) -> Self {
        Self { config }
    }

    pub fn list_all(&self) -> Result<Vec<String>> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let packages = db.list_packages();
        Ok(packages.iter().map(|p| p.name.clone()).collect())
    }

    pub fn show_info(&self, name: &str) -> Result<()> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let pkg = db
            .find_by_name(name)
            .ok_or_else(|| anyhow::anyhow!("Package not found: {}", name))?;

        println!("Package: {}", pkg.name);
        println!("Version: {}", pkg.version);
        println!("Manifest kind: {}", pkg.manifest_kind.label());
        println!("Path: {}", pkg.path.display());
        println!("Installed at: {}", pkg.installed_at);
        println!("Source: {:?}", pkg.source);
        if !pkg.nodes.is_empty() {
            println!("\nNodes:");
            for node in &pkg.nodes {
                println!("  - {}", node);
            }
        }
        println!("\nPrimitives:");
        for prm in &pkg.primitives {
            println!("  - {}", prm);
        }
        println!("\nServices:");
        for srv in &pkg.services {
            println!("  - {}", srv);
        }
        println!("\nSkills:");
        for skill in &pkg.skills {
            println!("  - {}", skill);
        }
        if !pkg.provided_interfaces.is_empty() {
            println!("\nProvided interfaces:");
            for interface_id in &pkg.provided_interfaces {
                println!("  - {}", interface_id);
            }
        }
        if !pkg.consumed_interfaces.is_empty() {
            println!("\nConsumed interfaces:");
            for interface_id in &pkg.consumed_interfaces {
                println!("  - {}", interface_id);
            }
        }

        Ok(())
    }

    pub fn find_by_primitive(&self, primitive_name: &str) -> Result<Vec<String>> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let packages = db.find_by_primitive(primitive_name);
        Ok(packages.iter().map(|p| p.name.clone()).collect())
    }

    pub fn find_by_service(&self, service_name: &str) -> Result<Vec<String>> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let packages = db.find_by_service(service_name);
        Ok(packages.iter().map(|p| p.name.clone()).collect())
    }

    pub fn find_by_skill(&self, skill_name: &str) -> Result<Vec<String>> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let packages = db.find_by_skill(skill_name);
        Ok(packages.iter().map(|p| p.name.clone()).collect())
    }
}
