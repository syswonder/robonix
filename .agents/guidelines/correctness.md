# Correctness

*Does it do what it claims, including when things fail?*

### no-silent-failure

Don't catch an exception to continue as if nothing happened. Handle it, report it where an operator will see it, or let it propagate. A fallback that hides a failure turns a clear error into a wrong result.

### no-impossible-handling

Don't add checks, retries or defaults for states the code cannot reach. They cost reading time and hide the real invariants.

### contract-first

Read the capability contract (`capabilities/**/*.toml` and its IDL) before calling it. Some contracts are asynchronous: `navigation/navigate` returns when the goal is accepted, and progress comes from `navigate/status`. Long-running robot actions belong in plans the executor can track and cancel, not inside another component.

### bounded-resources

Anything that grows with runtime has a limit: caches, in-memory history, viewer recordings, files written per object or per session. State the bound in code.

### dont-block-the-loop

Keep slow work, blocking I/O and calls that hold the GIL off asyncio event loops and request threads. Use a worker thread for slow Python work and a child process for native servers that do not release the GIL.

### test-the-behaviour

Test what can break and would matter: a bug fix gets one test that fails without it; a new feature gets a few tests of its main behaviour, not one per branch. Tests assert inputs and outputs, never the text of source files, markup or comments. Add tests to the module's existing test file; a new test file only comes with a new module. Don't change production code only to make a test pass.

### end-to-end-for-boundaries

A change that crosses process boundaries (Atlas wire, driver lifecycle, container start-up, web UI to service) is verified by running the processes. Compiling and unit tests are not enough.
