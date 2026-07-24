scribe — unified logging facade
Background
Robonix components log ad-hoc today — Rust binaries (atlas / pilot / executor / liaison) and Python services (scene / memory / primitives) each use their own print / logging / log::* with their own format and destination. For an embodied OS running many processes on one robot this makes it hard to (a) read what the whole system is doing, (b) filter by component, (c) collect logs for post-mortem. scribe is a thin, unified logging facade.

Goals
One logging entry point used by all components. A log line = timestamp + level + tag (source) + message.
Dual sink: human-readable console and per-component file.
tag = component name / provider_id, so logs are filterable by who emitted them.
Borrow Android's "single entry + tag + level" idea — do not reimplement logd/logcat.
Non-goals (v1)
Central network aggregation, a structured-log query engine, and log-rotation policy. Keep v1 to the facade + files; revisit if needed.

Proposed design
enum Level { Debug, Info, Warn, Error }

struct LogRecord {
    ts:    i64,        // nanoseconds; later sourced from chronos now() (#62)
    level: Level,
    tag:   String,     // component name / provider_id
    msg:   String,
}

trait Scribe {
    fn log(&self, level: Level, msg: &str);   // + debug/info/warn/error(msg) convenience wrappers
}
Viewing: rbnx logs (filter by tag / level).

Tasks

Define the facade (Rust + Python): Level / LogRecord + convenience methods.

Console formatter + per-component file sink.

Source the timestamp from chronos now() ([FEATURE] chronos — unified time & timestamp-at-source #62) when available; time.time() until then.

Migrate a first batch of components off raw print / logging onto scribe.

rbnx logs command with tag / level filtering.
Acceptance criteria
A component using scribe produces consistently-formatted console + file output, tagged by component.
rbnx logs can filter by tag and level.
Open questions
Plain text vs JSONL log format.
Whether the level set (Debug/Info/Warn/Error) is sufficient.
References
Related: [FEATURE] chronos — unified time & timestamp-at-source #62 (chronos — timestamp source).