use crate::Config;
use anyhow::Result;
use std::path::PathBuf;

pub async fn execute(_config: Config, set_storage_path: Option<PathBuf>, show: bool) -> Result<()> {
    if let Some(new_path) = set_storage_path {
        let mut config = Config::load()?;
        config.package_storage_path = new_path;
        config.save()?;
        config.ensure_storage_dir()?;
        println!(
            "Package storage path updated to: {}",
            config.package_storage_path.display()
        );
    } else if show {
        let config = Config::load()?;
        println!(
            "Package storage path: {}",
            config.package_storage_path.display()
        );
    } else {
        eprintln!("Error: Either --set-storage-path or --show must be specified");
        std::process::exit(1);
    }

    Ok(())
}
