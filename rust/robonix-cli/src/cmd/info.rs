use crate::{Config, PackageQuery};
use anyhow::Result;

pub async fn execute(config: Config, name: String) -> Result<()> {
    let query = PackageQuery::new(config);
    query.show_info(&name)?;
    Ok(())
}
