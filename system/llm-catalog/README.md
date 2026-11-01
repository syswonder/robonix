# robonix-llm-catalog

Rust counterpart of `pylib/robonix-api/robonix_api/llm`: load a feature catalog, resolve a
set of requirements to model endpoints, and connect directly. The crate does
not proxy traffic, track health or manage keys.

```rust
use robonix_llm_catalog::{
    ModelCatalog, Requirements, api_key_for, resolve_from_env,
};

let catalog = ModelCatalog::load("catalog.example.yaml".as_ref())?;
catalog.model("deepseek/flash")?; // lookup by id
let targets = catalog.resolve(&Requirements {
    features: vec!["vision".into(), "tools".into()],
    input_tokens: 12_000,
    max_output_tokens: 512,
    ..Requirements::default()
});
let target = &targets[0];

// Deployment entry point: None when ROBONIX_LLM_CATALOG is unset.
if let Some(target) = resolve_from_env(&Requirements::default())? {
    let key = api_key_for(&target);
}
```

The API mirrors `robonix_api.llm`: load, model lookup, resolve, env helper
and key helper. Matching models are ordered by latency, then combined token
price, then id. When `json_schema` is required and no schema-capable model
fits, the default `allow_downgrade` lets a `json_object` model stand in and
marks every target `downgraded = true`.

Omit `api_key_env` for keyless endpoints. When set, it must match
`[A-Za-z_][A-Za-z0-9_]*` so launchers can forward it as an environment variable.

The shared cases in `pylib/robonix-api/robonix_api/llm/contract/resolve_cases.json` are run by
both this crate and the Python package:

```bash
cargo test -p robonix-llm-catalog
cargo run -p robonix-llm-catalog --example resolve -- \
  pylib/robonix-api/robonix_api/llm/catalog.example.yaml vision,tools 1000 256
```
