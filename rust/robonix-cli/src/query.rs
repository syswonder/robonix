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
        let pkg = db.find_by_name(name)
            .ok_or_else(|| anyhow::anyhow!("Package not found: {}", name))?;
        
        println!("Package: {}", pkg.name);
        println!("Version: {}", pkg.version);
        println!("Path: {}", pkg.path.display());
        println!("Installed at: {}", pkg.installed_at);
        println!("Source: {:?}", pkg.source);
        println!("\nCapabilities:");
        for cap in &pkg.capabilities {
            println!("  - {}", cap);
        }
        println!("\nSkills:");
        for skill in &pkg.skills {
            println!("  - {}", skill);
        }
        
        Ok(())
    }

    pub fn find_by_capability(&self, cap_name: &str) -> Result<Vec<String>> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let packages = db.find_by_capability(cap_name);
        Ok(packages.iter().map(|p| p.name.clone()).collect())
    }

    pub fn find_by_skill(&self, skill_name: &str) -> Result<Vec<String>> {
        let db = PackageDatabase::load(&self.config.package_storage_path)?;
        let packages = db.find_by_skill(skill_name);
        Ok(packages.iter().map(|p| p.name.clone()).collect())
    }
}

