# robonix-llm-metrics

Rust writer of the per-request model usage record described in
[`robonix_api.llm`](../../pylib/robonix-api/robonix_api/llm/README.md). Pilot
uses it for every chat-completions request.

```rust
use robonix_llm_metrics::{Call, Status, directory_from_env};

let directory = directory_from_env(); // $ROBONIX_LLM_METRICS_DIR, read once
let mut call = Call::start(directory.as_deref(), "pilot", "plan", &base_url, &model, true);
// call.retry() for each resend; call.observe(&chunk) for each JSON body/chunk.
call.finish(Status::Ok, Some(200));
drop(call); // the record is written here
```

A call dropped without `finish` is recorded as `cancelled`. Records go to
`<directory>/<consumer>-<pid>.jsonl`, each appended with one write; a `None`
directory writes nothing, and a write failure is logged once through Scribe.
`consumer` must match `[a-z0-9_-]+`; `Call::start` panics otherwise, since it
names the file.

```bash
cargo test -p robonix-llm-metrics
```
