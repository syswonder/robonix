# Retaining long capability outputs

## Problem and design

[Issue #306](https://github.com/syswonder/robonix/issues/306) reports that a capability manual was cut first by Executor's 12,000-byte limit and then by Pilot's 4,096-character limit. File reads and shell stdout/stderr had additional limits. Raising only Pilot's cap cannot recover content already discarded upstream.

Executor keeps the original result for verification. Long model-facing text is saved as a complete file, accompanied by its path, byte count, call identity and preview. Pilot uses the existing `run_command` capability to search with `rg -n` and inspect relevant lines with `sed -n`; POSIX `grep -n -F` is the fallback. Reading saved output does not repeat the original capability or refresh an observation. Image responses retain the existing multimodal path. Topic streams such as map PointCloud2 are not treated as model tool-result text.

The inline threshold is a context-cost decision, separate from storage and process-output resource limits. A failed save explicitly reports incomplete information without changing a successful action into a failure that might trigger repetition. Files persist across process restarts; quota exhaustion does not silently delete earlier results.

## Measured workload

The available release-test corpus contains 383 terminal results: 18 image outputs and 365 text outputs. These are test invocation frequencies, **not production usage frequencies**. Exact contract/payload deduplication leaves 178 text payloads. Timestamps still distinguish otherwise similar observations.

| Category | Calls | Median UTF-8 bytes | P95 bytes | Maximum bytes |
|---|---:|---:|---:|---:|
| Shell command | 9 | 24 | 24 | 24 |
| File read | 27 | 19 | 24 | 24 |
| Scene query | 87 | 481 | 3,252 | 6,963 |
| Lidar snapshot RPC | 18 | 14,200 | 16,019 | 16,019 |
| Other text | 224 | 73 | 2,528 | 3,481 |
| Image payload, separately handled | 18 | 45,719 | 76,956 | 76,956 |

The shell/file samples are too small to establish their tails. Existing upstream truncation can also censor observed lengths. Static assembly of five cached provider manuals, adding one description per manifest capability, ranges from 1,100 to 12,801 bytes. This estimate excludes transport duplication and automatically registered lifecycle contracts; it is not an observed Atlas response.

Thresholds of 2/4/8/12/16 KiB would retain 45/20/18/18/0 of the 365 observed text results as files. Selecting a threshold solely from that table would overfit short CI tasks. Synthetic long-output tasks therefore complement the corpus; they are not mixed into its frequency distribution.

## Real-model threshold comparison

The comparison uses isolated, real Atlas, Executor and Pilot processes with `deepseek/deepseek-v4.1-flash-1017` through an OpenAI-compatible gateway. A script emits a report with a target at its end and increments a counter. Each test checks the exact answer and confirms the script ran once. Token counts below are provider-reported cumulative input tokens; output tokens, latency and caching also affect costs.

| Report bytes | Inline result | File + search | Observation |
|---:|---:|---:|---|
| 5,888 | 2 rounds, 9,221 tokens | 3 rounds, 14,000 tokens | Inline avoids an extra planning round |
| 11,818 | 2 rounds, 10,401 tokens | 3 rounds, 14,003 tokens | Inline uses fewer input tokens |
| 25,678 | 2 rounds, 13,114 tokens | 3 rounds, 14,536 tokens | Inline uses slightly fewer input tokens |
| 52,738 | 2 rounds, 18,409 tokens | 3 rounds, 13,971 tokens | Search uses fewer input tokens |

A **32 KiB candidate** lies between these observed cases. This is an engineering starting point, not a universally optimal threshold: each configuration has one trial, the tasks are synthetic lookup tasks, and model/context/concurrency changes can move the crossover. The threshold remains configurable. Repeated and multi-answer tasks should be included before broader performance claims.

A separate 106,290-byte manual registered in Atlas was read through `read_capability_doc`. DeepSeek recovered the calibration value at line 1603 and the target-identity rule at line 1606 through the retained file. One malformed shell-quoting attempt was followed by a successful `sed` read; final success does not imply every intermediate command succeeded. A 106 KB command-output test with `rg` removed from PATH also completed through `grep`, with one original script invocation.

Final checks at the 32 KiB default covered the manual, shell output with rg, and shell output without rg. All three retained files matched the original fixture bytes. The manual was read once and both shell fixtures executed once. They required 4 / 3 / 3 model rounds and 23,188 / 13,883 / 13,949 cumulative input tokens respectively. These fixtures do not validate physical robot behavior or guarantee that a model will retrieve every relevant fact in an arbitrary task.

## Prior systems and robot-specific choices

- [DeepSeek Harness spill policy](https://github.com/deepseek-ai/deepseek-harness/blob/fcc0ad462172522aaf5c93a7b2e01f6b9f0b0159/packages/spill/spill-policy/src/index.ts) saves complete content before producing a bounded display. Its local backend records session ownership; its shell renderer exposes saved stream paths. Robonix borrows retention and ordinary-tool retrieval, while preserving original results for action verification.
- [OpenCode tool-output handling](https://github.com/anomalyco/opencode/blob/5a47777cc4c7511082e9ca5d4b430cea8113c4bb/packages/core/src/tool-output.ts) saves full text with an output path and separately bounds displayed text. Its defaults are not evidence for a Robonix threshold.
- [Codex captured output](https://github.com/openai/codex/blob/c3936b4b8a66b590b1ee4edd2276314899c87e8a/codex-rs/core/src/unified_exec/output.rs) distinguishes collection loss from display truncation. This supports reporting completeness explicitly; it does not establish universal full-byte persistence in Codex.

No public comparative output-size dataset was measured for those systems. Their mechanisms inform this design. Robot actions can have physical effects and observations age, so searching a saved result must not imply re-execution or current-world validity.

## Reproduce model tests

Build `robonix-atlas`, `robonix-executor`, `robonix-pilot` and `robonix-cli`. Set `ROBONIX_VLM_UPSTREAM`, `ROBONIX_VLM_API_KEY` and `ROBONIX_VLM_MODEL` in the environment. Then run:

```sh
python testing/pilot_output_retrieval.py --output /tmp/retention-test --threshold 32768 --lines 1600 --document
python testing/pilot_output_retrieval.py --output /tmp/retention-no-rg --threshold 32768 --lines 1600 --no-rg
```

The document test requires `grpcio` and `grpcio-tools`. The runner refuses to overwrite evidence, binds local ephemeral ports, registers no robot providers and stops its own processes on exit. A new output directory is required for each trial. Logs contain fixture text, commands, model answers and usage, but not API keys.
