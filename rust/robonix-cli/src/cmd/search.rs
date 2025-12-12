use crate::{Config, PackageQuery};
use anyhow::Result;

pub async fn execute_cap(config: Config, name: String) -> Result<()> {
    let query = PackageQuery::new(config);
    // Try to find as primitive first, then as service
    let packages_prm = query.find_by_primitive(&name)?;
    let packages_srv = query.find_by_service(&name)?;
    let packages: std::collections::HashSet<String> = packages_prm.into_iter().chain(packages_srv.into_iter()).collect();
    
    if packages.is_empty() {
        println!("No packages found with primitive or service: {}", name);
    } else {
        println!("Packages with primitive or service '{}':", name);
        for pkg in packages {
            println!("  - {}", pkg);
        }
    }
    Ok(())
}

pub async fn execute_skill(config: Config, name: String) -> Result<()> {
    let query = PackageQuery::new(config);
    let packages = query.find_by_skill(&name)?;
    if packages.is_empty() {
        println!("No packages found with skill: {}", name);
    } else {
        println!("Packages with skill '{}':", name);
        for pkg in packages {
            println!("  - {}", pkg);
        }
    }
    Ok(())
}
