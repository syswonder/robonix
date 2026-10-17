<!-- SPDX-License-Identifier: MulanPSL-2.0 -->
# Webots 中的 RTDL 流式执行复测

主测试用于验证完整链路：**自然语言任务 → Pilot 调用真实大模型生成 RTDL → Pilot 流式提交 Executor → Webots 执行 → 核对结果**。模型配置读取项目根目录的 `env.sh`；测试客户端不预先构造或提交 RTDL plan。真实模型输出具有不确定性，不属于现有 fake VLM CI 的确定性测试。

## 测试分类：输入、输出、目的

本文共有 **8 类测试/检查**：A、B、F、H 让 Pilot 使用真实模型生成 RTDL；C、G 手写 Executor 协议输入（G 还使用可控服务）；D 用模拟模型制造故障；E 检查代码本身。`analyze.py` 是第一类的结果分析步骤，不是另一类任务。

| 类别 | 用例数量 | 是否经过 Pilot | RTDL 从哪里来 | 是否在 Webots 执行动作 |
|---|---|---|---|---|
| A. 真实模型节点计数与流式执行（主测试） | 11 档：0–10 个 `do` | 是 | Pilot 调用 `env.sh` 的真实模型生成 | 是；0 档不执行动作 |
| B. 真实模型导航 | 转动：3 个 `do`；平移到目标后拍照：2 个 `do` | 是 | Pilot 调用同一真实模型生成 | 是 |
| C. Executor 协议边界（补充） | 3 个用例 | 否 | 测试脚本手写节点及控制消息 | 是，调用相机和激光 |
| D. Pilot 生成中失败与取消（补充） | 5 个用例 | 是 | 本地模拟模型返回预设 RTDL 片段 | 否，执行本地可观测命令 |
| E. Rust 工作区检查 | 编译、格式、测试、Clippy 四项 | 不启动上述业务链路 | 不适用 | 否 |
| F. 真实导航中的中断与取消 | 串行计划中断/取消、并行分支、跨会话隔离、同会话多计划隔离及重复执行 | 是 | 导航计划由真实模型生成；自然语言取消也由模型决定控制操作 | 是 |
| G. Executor 取消边界与底层故障（补充） | 21 个用例，含重复与完成竞态 | 否 | 手写协议输入；可控测试服务 | 否 |
| H. Potential advance from streamed node generation | 9 live trials, plus one warmup | Yes | Live model configured by `env.sh` | Yes, lidar and camera |

### A. 真实模型节点计数与流式执行

- **输入：** `webots_rtdl/run.py` 向 Pilot 的 `SubmitTask` 提交自然语言 `Task.text`。例如要求“依次读取一次激光和一次相机，恰好生成两个 `do`”。提示词指定步骤、capability 和参数，不提交现成 RTDL。
- **输出：** Pilot 返回模型生成的计划、每个节点的状态及结果、最终文字回复和任务状态；Executor 日志记录真实 dispatch。原始输入为 `count-XX.prompt.txt`，事件为 `count-XX.jsonl`，检查结果为 `count-XX.summary.json`。
- **在测什么：** Pilot 能否按任务生成正确数量的 RTDL 节点，Executor 是否边生成边执行，以及执行是否有遗漏、重复或失败。`analyze.py` 读取事件和两端日志，核对真实调用顺序与时间。
- **通过条件：** 0 档没有计划或调用；1–10 档每档一个计划、恰好 N 个 `do`、每个节点恰好成功一次、最终任务为 `done`；合计 55 次调用。非零档首次 dispatch 必须早于完整模型输出结束。仅有 `done` 不代表通过。

### B. 真实模型导航

**B1：原地转动（`turn.py`）。**

- **输入：** `webots_rtdl/turn.py` 读取当前 TF 位姿，计算位置不变、朝向增加 45° 的目标，将该目标和“相机 → 导航 → 相机”的自然语言任务提交给 Pilot `SubmitTask`。脚本计算目标参数，不构造 RTDL。
- **输出：** Pilot 生成的三个 `do` 及执行事件、导航验证状态、最终回复，以及脚本独立读取的运动前后 TF 位姿。输入保存在 `turn-input.json`，事件在 `turn.jsonl`，位姿误差和判定在 `turn-summary.json`。
- **在测什么：** 模型生成的计划能否驱动真实仿真运动，并完成导航后的验证；用 TF 确认机器人实际姿态变化，而不只相信文字回复。
- **通过条件：** 一个计划中按要求包含三个节点，三个节点终态均成功且身份匹配；最终朝向误差小于 0.35 rad、位置变化小于 0.25 m。另从事件核对导航的 `VERIFYING` 过程。这不是精确旋转 45° 的测试。

**B2：平移到目标后拍照（`navigation.py complete`）。**

- **输入：** 脚本读取实时 map 和 TF，选择距当前位置 1.2–2.5 m 的目标，将“先导航到此坐标，再拍一次相机快照”的自然语言任务提交给 Pilot；目标坐标是参数，不是手写 RTDL。
- **输出：** 模型生成的导航 → 相机两个 `do`、节点事件、导航验证状态、实际位姿及里程计速度；输入、事件和判定分别为 `navigation-complete-input.json`、`navigation-complete.jsonl`、`navigation-complete-summary.json`。
- **在测什么：** 导航是否真的产生平移、到达指定目标并完成验证，后续相机调用是否执行一次。
- **通过条件：** 两节点身份匹配且各成功一次，观察到 `VERIFYING`，实际平移大于 0.7 m，最终位置误差小于 0.5 m、朝向误差小于 0.35 rad，Executor 活动计划归零。

### C. Executor 协议边界

- **输入：** `webots_rtdl/protocol.py` 绕过 Pilot，直接向 Executor `ExecuteStream` 发送手写 `ExecuteStreamInput`。三个用例分别发送正常嵌套结构、缺少 `END` 的提前 EOF、以及 `parent_index=99` 的非法节点。
- **输出：** Executor 返回节点状态、正常计划完成事件或 gRPC 错误；脚本另查询活动计划。各用例事件及判定写入 `protocol-*.json`，最后的活动计划列表写入 `active-plans-final.json`。
- **在测什么：** Executor 能否在输入尚未结束时执行已收到的节点，正确处理嵌套结构，并在输入异常后清理计划。真实传感器调用不等于真实模型生成：这一类完全不测试 Pilot 的生成能力。
- **通过条件：** 三例的首个激光调用都在发送 `END` 前成功，此时计划尚未完成；正常例最终三个节点成功并完成计划，两个异常例返回 `INVALID_ARGUMENT`；最终无活动计划。

### D. Pilot 生成中失败与取消

- **输入：** `generation_faults.py` 向独立 Pilot 提交任务，由本地模拟 SSE 服务返回预设的两步顺序计划片段：第一步写入 `start` 后等待，第二步写入 `second`。第一步开始后，分别制造错误 JSON、提前 EOF、流空闲超时，或向同一会话发送 `abort_turn`。另加在模型已开始回复、尚未输出任何 RTDL 节点时中断的用例。不调用 `env.sh` 的真实模型。
- **输出：** Pilot 的计划和取消事件、失败用例的 RPC 错误、命令轨迹文件、模型请求次数，以及 Executor 活动计划列表；保存在 `fault-*.json`、`fault-*.trace` 和 `faults.log`。
- **在测什么：** 模型仍在生成时发生失败或主动中止，Pilot 是否取消已提交的动作、阻止后续动作执行并清理计划，以及是否按当前设计停止而不再次请求模型。
- **通过条件：** 每例只有一次模型请求；已产生节点的四例轨迹只有 `start`，没有第一步完成标记或 `second`；两个节点均取消，活动计划为空。三种失败应返回错误，主动中止正常结束；节点产生前中断应为零计划、零执行、零残留。
- **脚本位置：** `testing/webots_rtdl/generation_faults.py`。它使用本地命令验证取消，不测机器人运动取消；也不能证明真实模型会输出正确计划。

### E. Rust 工作区检查

- **输入：** 当前工作区源码（包括未提交修改）和已有 Rust 测试，通过 `cargo build --workspace`、`cargo fmt --all -- --check`、`cargo test --workspace --all-targets`、`cargo clippy --workspace --all-targets -- -D warnings` 检查。
- **输出：** 命令退出码、测试通过/失败/忽略结果和诊断日志；本轮保存为 `build.log`、`fmt.log`、`tests.log`、`clippy.log`。
- **在测什么及通过条件：** 确认代码能编译、格式与静态检查通过、已有测试无失败；忽略项单独报告。这些检查不替代真实模型和 Webots 测试。

### F. 真实导航中的中断与取消

F1、F2 两个串行用例先通过真实模型读取当前导航速度并设置为 100%，再让 Pilot 生成“导航 → 相机”的计划；选择距离 0.8–2.5 m 的目标。扩展取消脚本 `cancel_live.py` 也使用 100%。准备和恢复速度使用独立任务，不计入该导航计划的两个 `do`，结束时恢复原速度。在独立里程计确认已平移超过 0.10 m、线速度大于 0.03 m/s，且 Nav2 action 为 executing 后才触发控制；不能在机器人尚未开始运动时宣称已测试运动中取消。此速度设置按 2026-10-05 用户要求调整，尚未按新速度复测；下文历史结果仍保留当时使用的 20% 速度。

| 用例 | 控制输入 | 控制经过什么路径 | 预期输出 |
|---|---|---|---|
| F1：中断（`navigation.py abort`） | 向同一 Pilot 会话提交 `context_json={"abort_turn":true}` | Pilot 现有中断接口直接请求 Executor 取消；此控制本身不经模型生成 | 中断响应，原导航 action 取消，机器人停止，后续相机不执行 |
| F2：自然语言取消（`navigation.py cancel`） | 向同一会话提交“取消当前导航和后续拍照，立即停止，不重新导航”，带 `steer` 上下文 | Pilot 将新指令交给真实模型，由模型输出 RTDL 计划取消操作 | 取消控制及结果、原导航 action 取消、最终回复；机器人停止，后续相机不执行 |

- **在测什么：** 中断/取消是否从 Pilot 传到 Executor 和导航服务，机器人是否实际停住，旧计划是否清理，以及取消是否影响同一会话后续正常任务。
- **共同通过条件：** 原导航只 dispatch 一次、后续相机零 dispatch，无额外导航计划；原 ROS goal 最终状态为 CANCELED，Executor 活动计划归零。F1 还要求 Pilot 返回 `interrupted`；F2 要求导航及相机两个 RTDL 节点均为 CANCELED（steer 后的终态从控制请求的事件流读取）。独立里程计连续至少 3 秒线速度小于 0.02 m/s、角速度绝对值小于 0.03 rad/s，期间位移小于 0.05 m；只收到“已取消”的回复不算通过。自然语言取消必须等待真实模型生成控制操作，可能比 `abort_turn` 慢，实测分别记录响应和停车时间。
- **恢复检查：** 中断/取消结束后，向同一会话提交“读取一次激光快照”的新自然语言任务，要求模型生成一个 `do` 并成功执行，最终无活动计划。
- **输出文件：** `navigation-abort-*`、`navigation-cancel-*` 分别保存输入、控制输入、响应、判定和 Executor 调用证据；`*-recovery.jsonl` 为恢复任务事件。`navigation-ros.jsonl` 保存独立 ROS 观测，包含位姿、速度及 action 状态。
- **F1/F2 的节点数：** 原计划为 `sequence → [导航 do, 相机 do]`，总共 **3 个节点、2 个 do**；根级 `cancel_plan` 是控制操作，不新增执行计划。下列 F3–F6 扩大并行和隔离覆盖；生成期间由 D 类覆盖，`VERIFYING` 的精确时机由 G 类覆盖。

**F3–F6：由 `cancel_live.py` 执行的扩展用例。** 每项输入仍是给 Pilot 的自然语言，RTDL 由当前 `env.sh` 模型生成；辅助命令仅在测试输出目录写入 `start/end` 标记，用来独立判断另一分支是否真的被停止。

| 用例 | 原始计划及节点数 | 中途输入 | 输出及通过条件 |
|---|---|---|---|
| F3 并行中断 | 一个 parallel，下有两个 sequence：导航→相机；等待命令→激光；**7 总节点、4 do** | 观察到真实平移和命令 start 后，向同会话发 `abort_turn` | 导航和命令停止，相机和激光都未 dispatch，trace 只有 start，无残留计划 |
| F4 并行自然语言取消 | 同 F3，**7 总节点、4 do** | 自然语言要求取消该计划 | 模型输出计划取消操作，四个 do 均 CANCELED；物理停车、命令停止和后续零 dispatch 均成立 |
| F5 跨会话隔离 | A 会话导航→相机；B 会话等待命令→激光；每计划 **3 总节点、2 do**，合计 6 总节点、4 do | 两计划均在执行时，仅对 A 发 `abort_turn` | A 停车且相机不执行；B 命令正常产生 end、激光成功；最终无残留 |
| F6 同会话多计划隔离 | 同一会话先启动导航→相机，再用 steer 明确新增独立的等待命令→激光计划；每计划 **3 总节点、2 do** | 自然语言只取消导航计划的准确 plan_id，保留另一计划 | 导航停止且相机不执行，另一计划继续并成功；不能因为取消回复已结束就要求另一计划立即消失 |

F3–F6 均读取原导航速度、将测试速度设为 100%、最终恢复原值，且独立 ROS 观测须确认停车。若大角度转向无法建立平移前置条件，可用 `RTDL_CANCEL_ALIGN=1` 先让真实模型在原地完成朝向准备；朝向准备允许只读 Scene/Mapping 位姿查询，所有准备调用单独记录，不计入原始取消计划的节点数，不能用它覆盖此前失败。F6 在回复结束后继续通过命令轨迹、Executor dispatch 和 Pilot forest 日志观察另一计划的完成，不另发任务打断它。重复测试以独立 plan/session/trace 标识运行，逐轮保留结果，不能只报告最后一次成功。

**范围限制：** 这些用例每次只有一个物理导航动作，另一分支/计划是命令和传感器；它们证明调度及取消隔离，不证明同一底盘能够并行执行两个导航目标。有限轮数不能排除所有竞态。

**F7：真实模型仍在生成时中断（`cancel_generation_live.py`，独立重复 3 轮）。**

- **输入：** 给 Pilot 自然语言要求一个 sequence、10 个 do：先运行带 start/end 标记的等待命令，再 8 次激光、最后相机；计划仍由 `env.sh` 的真实模型生成。脚本观察到命令 start 且 Pilot 尚未发布完整 Plan 时，立即发 `abort_turn`。
- **输出与判据：** 被提交的计划前缀及其取消终态、命令轨迹、Executor dispatch、活动计划列表；要求已提交 do 全部取消且身份唯一匹配、trace 只有 start，任何激光/相机都未 dispatch，最终无活动计划。
- **节点口径：** 原任务期望完整输出 11 个总节点、10 个 do；中断后按实际已提交前缀计数，不能声称完整 10 个 do 已生成。本例不移动机器人，验证真实模型生成期间的 Pilot → Executor 取消路径。

### G. Executor 取消边界与底层故障

- **输入：** `cancel_boundaries.py` 在独立端口启动真实 Atlas / Executor 和可控测试服务，直接发送手写 `ExecuteStreamInput` / `ControlPlan`。服务复用现有 navigation、camera、verifier 及 lifecycle contracts；没有调用真实模型，也没有操作 Webots。
- **输出：** 每例的控制响应、节点事件、活动计划列表和独立服务运行状态分别写入 `<用例>.json`；服务收到的请求在 `provider-events.jsonl`，汇总在 `summary.json`。
- **在测什么：** 在能够精确控制的时间点验证取消的幂等性、作用范围、结果一致性及底层故障处理，避免把计划从列表移除误认为动作已经停止。

| 输入/时机 | 用例数 | 期望输出和正确性判据 |
|---|---:|---|
| 不存在的 plan_id | 1 | 返回无事可做、已完成，不创建计划或调用服务 |
| 仅发送 root，尚未提交任何 do 时取消 | 1 | 零服务调用、计划清理 |
| parallel 下四个执行中的 do | 1 | 四个 do 各一个取消终态，四个服务运行均停止 |
| 同计划同时发 5 次取消 | 1 | 响应一致，不重复产生终态，无残留 |
| 已正常完成的计划再次取消 | 1 | 幂等无操作，不重写已成功的结果 |
| 一个 do 已 VERIFYING、后续 do 已提交 | 1 | 已完成动作的验证仍给出真实成功/失败，不能改写成取消；另一 do 被取消；短 wait_ms 可先返回 completed=false，之后清理 |
| 两个活动计划，指定一个取消 / 明确 cancel_all | 2 | 指定取消保留另一计划；全局 cancel_all 明确停止两个计划，不混淆两种作用范围 |
| 完成附近 1.8/1.95/2.0/2.05/2.2 秒触发取消 | 5 | 成功先发生则保留成功，取消先发生则取消；每节点唯一终态，无后台运行或残留 |
| 相同正常取消用例独立重复 | 5 | 每轮已执行 do 停止、后续 do 不执行、终态唯一、无残留 |
| 底层拒绝取消 / MCP 报错 / 取消响应延迟 10 秒 | 3 | 不得用清理计划代替确认停止，明确暴露停止失败或未确认；控制请求应有可观察的时限 |

共 **21 例**。故障例即使没有残留计划，只要服务仍在运行，就不能判为“动作取消正确”；脚本会保留响应和后台状态并将此项标为失败。可控服务仅用于复现故障，不把这些结果混称为真实 Webots 停车测试。

A、B、F 的导航计划才是“Pilot 调用真实模型生成 RTDL 并在 Webots 执行”的证据。各类结果分别报告；A 的 55 次调用不包含其他类别。下面先给出 A 的任务明细，再给出执行步骤及实测结果。

### H. Potential advance from streamed node generation

Status: measured on 2026-10-10, 2026-10-13, 2026-10-14, 2026-10-16 and 2026-10-17 using the run-local scripts listed below. The repository's `latency.py` still implements the earlier paired comparison and does not produce this metric.

Measure how much earlier the first executable RTDL node becomes available than the complete planning response within the same live generation. Run one streaming Pilot with the real Webots office world, Atlas, Soma, Executor and real lidar/camera drivers. Read the model configuration from `env.sh`. Pilot generates RTDL and submits nodes through its normal `ExecuteStream` path.

Reuse `run.py` tasks with N=1, 5 and 10: one lidar call for N=1; N−1 lidar calls followed by camera for larger N. Attempt one N=5 warmup, then three formal trials per N, for nine formal attempts. Use a fresh session for each trial and run them sequentially. Wait at least five seconds after each trial and its cleanup before starting the next, including between warmup and formal trials. Record the source revision, model settings and available capability catalog.

#### Timing and metric

Capture the upstream SSE bytes as they arrive, with `time.monotonic_ns()` from one recording process. Associate decoded content deltas with the arrival timestamp of the chunk containing their final bytes. Preserve UTF-8 and SSE boundaries across chunks. Use the production `RtdlIncrementalParser` and the recorded capability targets to identify node availability from these deltas in a read-only analysis.

| Timestamp | Definition |
|---|---|
| `t_request` | Recorded immediately before opening the upstream model request, after the recorder's API cooldown. |
| `t_first_content` | Arrival of the first nonempty content delta in the initial planning response; reasoning deltas do not qualify. |
| `t_first_do` | Arrival of the bytes that let the parser first emit a valid `NODE` carrying a complete executable `do`, including its capability and arguments. A `sequence` or `parallel` root does not qualify. |
| `t_full` | Arrival of the last nonempty content delta in that same initial planning response, confirmed after normal stream termination and successful final parsing of the complete response. This includes the remaining response envelope and `task_update` text. |

For each valid trial, calculate `potential_advance_s = (t_full - t_first_do) / 1e9`. This is the observed generation window during which the first executable node is already available. Subsequent model narration, Executor dispatch, handler entry and action completion are outside this metric.

Also report elapsed time from `t_request` to first content, first executable node and the complete response, and `100 * (t_full - t_first_do) / (t_full - t_request)` as the window's percentage of this model-request interval. Time before first content includes network, queueing, input processing and any reasoning; it is not a measurement of pure thinking time. If the interface exposes reasoning deltas, record their first and last arrivals separately, along with reported reasoning-token usage when available. These are observed delivery times, not server-side inference timing. Pilot prompt preparation and subsequent narration are outside this request interval.

Record normal stream termination separately from `t_full`, so a delayed terminal marker or connection close does not inflate the generation window. Measure arrivals independently of Pilot's waits for Executor submission. If both endpoints arrive in one chunk, report zero at the capture's resolution; do not infer timing inside that chunk or discard the sample.

#### Validity, cleanup and reporting

A valid generation has one complete, parseable sequence with exactly the requested N `do` nodes, matching capability order and arguments, an identifiable first executable node and normal model-stream termination. Retain malformed, incomplete, timed-out and incorrect-plan attempts with their errors; exclude them from the metric and generate no replacement samples. Report execution outcomes separately, so a downstream execution failure does not silently remove a valid generation measurement. The recorder must continue reading upstream within its timeout after Pilot disconnects, including when execution failure interrupts generation.

Allow the initial planning response to finish before any test cleanup. After the plan finishes, use the existing `abort_turn` control if needed to end subsequent narration, wait for the original task RPC to close, and confirm zero active Executor plans before the next trial. If cleanup fails, preserve the measurement and stop before starting another task.

Save task inputs, timestamped raw SSE chunks, decoded deltas, parser node events, both metric endpoints, termination evidence, Pilot events and Executor logs in a fresh output directory. List every attempt in execution order, with N, generation validity, potential advance when available, and execution outcome; label the warmup separately and retain failed attempts. Report valid/attempted counts and generation and execution failures separately. With only three attempts per N, report individual observations without medians or quartiles. The measurement describes the model stream received by the client and does not establish end-to-end user waiting-time savings.

If Pilot corrects a rejected output, retain the initial-response validity result. As a supplementary observation, identify the response that produced the sole published plan by a unique match of tree structure, node descriptions, capability targets and arguments, with complete parsing, normal termination and arrival before plan publication. Report its first-do/full-response window and elapsed time from the trial's first upstream request to those endpoints. This interval includes earlier rejected replies, prompt preparation between requests, the recorder's API cooldowns and upstream delivery; it does not measure pure thinking or reclassify the initial response as valid. Leave the supplementary metric unavailable when the match is missing or ambiguous.

#### Results: 2026-10-10

Source revision `2acef09bde5b`, model `openai/gpt-6-luna` from `env.sh`, Webots `office.wbt` with TIAGo Lite. The deployment contained Atlas, Soma, Executor, one streaming Pilot and the real lidar/camera drivers. One N=5 warmup preceded nine formal attempts in the order `[1, 5, 10]` repeated three times. No failed attempt was replaced. The shortest measured interval after a trial's cleanup was 5.005 s; the recorder also enforced five seconds between upstream model requests, with an observed minimum of 5.002 s. All 19 model POSTs, including subsequent narration, returned HTTP 200; two streams failed to finish.

| Attempt | N | Generation | Potential advance (s) | Successful calls |
|---|---|---|---|---|
| Warmup (`warmup-n5`) | 5 | Valid | 1.932 | 5 / 5 |
| `trial-01-n1` | 1 | Valid | 0.954 | 1 / 1 |
| `trial-02-n5` | 5 | Valid | 2.001 | 5 / 5 |
| `trial-03-n10` | 10 | Valid | 3.243 | 10 / 10 |
| `trial-04-n1` | 1 | Stream idle timeout | Unavailable | 1 / 1 |
| `trial-05-n5` | 5 | Valid | 36.076 | 5 / 5 |
| `trial-06-n10` | 10 | Valid | 2.357 | 10 / 10 |
| `trial-07-n1` | 1 | Valid | 0.760 | 1 / 1 |
| `trial-08-n5` | 5 | Valid | 12.612 | 5 / 5 |
| `trial-09-n10` | 10 | Valid | 2.521 | 10 / 10 |

Valid formal generations: N=1, 2/3; N=5, 3/3; N=10, 3/3. The two large N=5 observations remain in the results: the 36.076 s window includes multiple long content-arrival gaps, the largest 12.156 s. These values include upstream delivery pauses and the remaining initial response envelope; they do not demonstrate stable action-start or task-completion savings, or a monotonic relationship with node count.

`trial-04-n1` stopped receiving content before the initial response completed. Pilot returned `VLM stream idle timeout`; independent capture continued to its 240 s limit without normal termination. That attempt has no generation metric, although its one lidar call had already succeeded. All 48 requested formal sensor calls succeeded exactly once, with matching Executor dispatch order; no leaf action failed or was canceled, and every trial ended with zero active plans. Separately, the subsequent narration for `trial-02-n5` failed to finish within the capture limit; its complete initial planning response remains valid. Neither interrupted stream is reported as a successful full conversation.

Evidence is in `/tmp/robonix-streaming-benefit-20261010.4W2nV6/`: `results.json`, `summary.json`, `validation.json`, `provenance.json`, per-trial task/events/generation/dispatch files, raw `model-*.chunks.jsonl`, and component logs. `runner.py` captures and executes the experiment; `analyze.py` decodes SSE and computes the results; `replay.rs` and the saved `replay-binary` run the production parser without executing actions. The temporary Rust test module was removed before live trials, and production source was unchanged. Parser tests passed (16); an offline fixture verified split UTF-8/SSE, first-do emission, and exclusion of a delayed terminal marker. No live model requests were used for that fixture.

#### Results: 2026-10-13

Source revision `c52884959794`, model `qwen3.8-flash` loaded from the current `env.sh`, with the same office/TIAGo Lite sensor tasks and minimal live deployment. One N=5 warmup and nine formal attempts ran without replacement. Minimum trial spacing was 5.005 s; minimum spacing between upstream POSTs was 5.003 s. All 31 model POSTs returned HTTP 200 and ended normally. Pilot automatically corrected rejected outputs, up to twice; these requests and subsequent narration are included in that POST count.

Only 1/9 formal initial responses met H validity: `trial-09-n10`. Three more tasks produced a valid plan after correction, and five formal tasks produced no executable plan. The warmup also failed. All 17 dispatched formal sensor calls succeeded exactly once, with no extra dispatches; every attempt ended with zero active plans.

Initial-response observations, in seconds from the first upstream request; the reasoning span is the interval between the first and last received reasoning deltas, not pure server inference time.

| Attempt | N | Initial reasoning delivery span (s) | First content (s) | Initial response |
|---|---|---|---|---|
| Warmup | 5 | 55.014 | 56.257 | Wrong operation/tree |
| trial-01-n1 | 1 | 21.826 | 22.776 | Wrong operation/tree |
| trial-02-n5 | 5 | 79.879 | 81.539 | Invalid args |
| trial-03-n10 | 10 | 28.819 | 30.196 | Invalid cap |
| trial-04-n1 | 1 | 11.230 | 12.804 | Wrong operation/tree |
| trial-05-n5 | 5 | 99.449 | 100.811 | Wrong operation/tree |
| trial-06-n10 | 10 | 36.555 | 37.715 | Wrong operation/tree |
| trial-07-n1 | 1 | 11.723 | 14.325 | Wrong operation/tree |
| trial-08-n5 | 5 | 4.164 | 6.731 | Wrong operation/tree |
| trial-09-n10 | 10 | 25.551 | 26.956 | Valid |

The next table describes the uniquely matched response that produced the published plan. First-do and full-response times are measured from the trial’s first upstream request, including earlier rejected outputs and the imposed API cooldowns. The window is within the matched response; its percentage uses the cumulative full-response time as denominator. Unavailable windows remain unavailable, rather than zero.

| Attempt | Plan response POST # | First do (s) | Full plan response (s) | Advance (s) | Window (%) | Successful calls / requested |
|---|---|---|---|---|---|---|
| Warmup | — | — | — | — | — | 0 / 5 |
| trial-01-n1 | 2 | 54.017 | 57.619 | 3.602 | 6.25 | 1 / 1 |
| trial-02-n5 | — | — | — | — | — | 0 / 5 |
| trial-03-n10 | — | — | — | — | — | 0 / 10 |
| trial-04-n1 | 3 | 59.047 | 62.427 | 3.380 | 5.41 | 1 / 1 |
| trial-05-n5 | 3 | 178.372 | 181.001 | 2.629 | 1.45 | 5 / 5 |
| trial-06-n10 | — | — | — | — | — | 0 / 10 |
| trial-07-n1 | — | — | — | — | — | 0 / 1 |
| trial-08-n5 | — | — | — | — | — | 0 / 5 |
| trial-09-n10 | 1 | 36.583 | 39.644 | 3.062 | 7.72 | 10 / 10 |

For the sole valid initial response, N=10, reasoning deltas arrived from 0.751 to 26.302 s; first content arrived at 26.956 s, first executable do at 36.583 s, and the full response at 39.644 s. The 3.062 s window is 7.72% of that request interval. The other three measured windows are conditional corrected-plan observations; their percentages include retries and test cooldowns. These observations do not establish task-completion savings or a general upper bound for this model.

Failures occurred during RTDL generation/validation: examples include missing `args`, missing `cap`, and `cancel_plan` with an invalid or absent target while no plan was active. The raw replies omitted these fields; they did not contain `args: null` or `cap: null`. Independent capture retained the complete upstream replies after Pilot stopped reading. HTTP success did not imply a usable plan.

The recorded requests used `response_format.type=json_schema` with `strict=false`. The generic node schema required only `op`, `op_id` and `description`; it did not conditionally require `cap` and `args` for a `do`. The meta-operation schema required only `op`, allowing `cancel_plan` without `plan_id` at the schema level. Pilot's RTDL validator enforced the missing requirements and checked cancellation against the actual in-flight plans. This gap allows a reply to fit the supplied schema while failing RTDL validation; it does not establish why the model repeatedly chose cancellation instead of the requested sensor work.

Evidence and reproduction scripts are in `/tmp/robonix-streaming-benefit-20261013.cxn3KQ/`: `results.json` retains all per-attempt initial and matched-plan timings, `validation.json` verifies spacing/dispatch/cleanup, and raw SSE, request bodies, parser events and component logs preserve each response. `runner.py`, `analyze.py`, `replay.rs` and the saved `replay-binary` reproduce execution and read-only analysis. Production source was restored before live trials; 16 parser tests and an offline fragmented UTF-8/SSE/reasoning fixture passed. The source revision differs from the 2026-10-10 run, so these runs are not a controlled model-only comparison.

#### Results: 2026-10-13 (updated env.sh)

Source revision `c52884959794`, model `qwen/qwen3.8-flash` and upstream `api.ofox.io` from the updated `env.sh`. Pilot and Executor binaries and the production parser source had the same SHA-256 hashes as the preceding 2026-10-13 run. The office/TIAGo Lite deployment, prompts and parser replay were reused. One N=5 warmup preceded nine formal attempts in the order `[1, 5, 10]` repeated three times, without replacement.

Minimum trial spacing was 5.004 s; minimum upstream POST spacing was 5.002 s. All 19 model POSTs returned HTTP 200. Thirteen ended with normal stop/DONE/HTTP EOF; six reached the recorder timeout. The POST count includes correction requests and subsequent narration.

Only 1/9 formal initial responses met H validity: `trial-02-n5` (N=1: 0/3; N=5: 1/3; N=10: 0/3). Two more tasks succeeded after one correction each, and six formal tasks produced no executable plan. The warmup failed during correction. All 11 dispatched formal sensor calls succeeded exactly once, with no extra dispatches or failed/canceled leaf actions. Every attempt ended with zero active plans.

Initial-response times are in seconds from the first upstream request. The reasoning span measures first-to-last reasoning-delta arrival; for timed-out streams it covers only the received portion before the stall. It does not measure pure server inference time. A dash means no content arrived.

| Attempt | N | Observed initial reasoning span (s) | First content (s) | Initial response |
|---|---|---|---|---|
| Warmup | 5 | 15.899 | 24.113 | Wrong cancellation |
| trial-01-n1 | 1 | 23.687 | 31.335 | Wrong cancellation |
| trial-02-n5 | 5 | 42.027 | 46.844 | Valid |
| trial-03-n10 | 10 | 36.658 | 41.092 | Wrong cancellation |
| trial-04-n1 | 1 | 38.886 | — | Stream timeout; no content |
| trial-05-n5 | 5 | 114.815 | — | Stream timeout; no content |
| trial-06-n10 | 10 | 64.102 | — | Stream timeout; no content |
| trial-07-n1 | 1 | 3.615 | — | Stream timeout; no content |
| trial-08-n5 | 5 | 59.033 | 60.732 | Missing cap and args |
| trial-09-n10 | 10 | 22.835 | — | Stream timeout; no content |

The next table uses the uniquely matched response that produced the published plan. First-do and full-response times start at the trial’s first upstream request and include rejected replies and imposed API cooldowns. The window is within the matched response; its percentage uses the cumulative full-response interval. Failed attempts have unavailable metrics.

| Attempt | Plan response POST # | First do (s) | Full plan response (s) | Advance (s) | Window (%) | Successful calls / requested |
|---|---|---|---|---|---|---|
| Warmup | — | — | — | — | — | 0 / 5 |
| trial-01-n1 | 2 | 91.257 | 93.779 | 2.522 | 2.69 | 1 / 1 |
| trial-02-n5 | 1 | 52.330 | 53.355 | 1.026 | 1.92 | 5 / 5 |
| trial-03-n10 | — | — | — | — | — | 0 / 10 |
| trial-04-n1 | — | — | — | — | — | 0 / 1 |
| trial-05-n5 | — | — | — | — | — | 0 / 5 |
| trial-06-n10 | — | — | — | — | — | 0 / 10 |
| trial-07-n1 | — | — | — | — | — | 0 / 1 |
| trial-08-n5 | 2 | 125.758 | 127.622 | 1.864 | 1.46 | 5 / 5 |
| trial-09-n10 | — | — | — | — | — | 0 / 10 |

For the sole valid initial response, reasoning deltas arrived from 4.816 to 46.844 s, first content at 46.844 s, first executable do at 52.330 s, and the full response at 53.355 s. Its 1.026 s window is 1.92% of the request interval. The other two windows are conditional corrected-plan observations. Their cumulative intervals include the imposed cooldowns. These samples show a small available overlap relative to the recorded waiting time; they do not measure task-completion savings or establish a general model upper bound.

`trial-04-n1`, `trial-05-n5`, `trial-06-n10`, `trial-07-n1` and `trial-09-n10` received reasoning but no content before the stream stalled. Pilot returned `VLM stream idle timeout`; independent capture continued to its 240 s timeout without normal termination. The warmup’s third POST also stalled during correction. `trial-03-n10` instead returned complete replies that attempted cancellation while no plan was active. `trial-08-n5` omitted `cap` and `args` in its initial do nodes and succeeded after correction. The captured schema still used `strict=false` with the same requirements described above. HTTP 200 and the updated model route did not ensure a valid, complete planning response.

Evidence is in `/tmp/robonix-streaming-benefit-20261013-retry.kLZ52V/`: all ten attempts and per-response reasoning/content timings in `results.json`, spacing/dispatch/cleanup checks in `validation.json`, hashes and configuration in `provenance.json`, and raw requests/SSE, parser events and component logs. The saved `runner.py`, `analyze.py`, `validate.py`, `replay.rs` and `replay-binary` preserve execution and analysis. Replay analysis passed for the captured streams. Production Rust source was unchanged, and the test deployment was stopped after the run.

#### Results: 2026-10-14 (Kimi)

Source revision `c52884959794`, model `moonshotai/kimi-k3` and upstream `api.ofox.io` from the updated `env.sh`. Pilot and Executor binaries, production parser source, task prompts and capture/analysis scripts matched the preceding Qwen run. The office/TIAGo Lite deployment and saved production-parser replay were reused. One N=5 warmup preceded nine formal attempts in the order `[1, 5, 10]` repeated three times, without replacement.

Minimum trial spacing was 5.004 s; minimum upstream POST spacing was 5.002 s. All 20 model POSTs returned HTTP 200. Fifteen ended with normal stop/DONE/HTTP EOF; five subsequent narration streams reached the recorder timeout. Every initial planning response ended normally.

All 9/9 formal initial responses met H validity (3/3 for each N), and the warmup was valid. No output correction was needed. All 48 requested formal sensor calls and all five warmup calls succeeded exactly once, with matching dispatch order, no extra dispatches and no failed/canceled leaf actions. Every attempt ended with zero active plans.

Times below are seconds from the initial upstream request. The reasoning span is first-to-last reasoning-delta arrival, not pure server inference time; the token count is reported by the interface. These observations do not establish that the models used equivalent thinking settings.

| Attempt | N | Reasoning delivery span (s) | Reported reasoning tokens | First content (s) | Initial response |
|---|---|---|---|---|---|
| Warmup | 5 | 0.409 | 28 | 4.234 | Valid |
| trial-01-n1 | 1 | 1.023 | 18 | 5.010 | Valid |
| trial-02-n5 | 5 | 1.295 | 36 | 4.408 | Valid |
| trial-03-n10 | 10 | 0.511 | 27 | 4.628 | Valid |
| trial-04-n1 | 1 | 0.716 | 21 | 3.125 | Valid |
| trial-05-n5 | 5 | 1.023 | 43 | 5.306 | Valid |
| trial-06-n10 | 10 | 0.614 | 38 | 3.715 | Valid |
| trial-07-n1 | 1 | 0.174 | 11 | 5.637 | Valid |
| trial-08-n5 | 5 | 0.590 | 25 | 5.113 | Valid |
| trial-09-n10 | 10 | 0.659 | 32 | 5.432 | Valid |

All published plans uniquely matched the first response. First-do and full-response times start at that response’s upstream request. The window is full response minus first do; its percentage uses the full-response request interval. Subsequent narration, action completion and between-trial cooldowns are outside this interval.

| Attempt | N | First do (s) | Full plan response (s) | Advance (s) | Window (%) | Successful calls / requested |
|---|---|---|---|---|---|---|
| Warmup | 5 | 6.282 | 10.609 | 4.327 | 40.78 | 5 / 5 |
| trial-01-n1 | 1 | 7.391 | 9.053 | 1.662 | 18.36 | 1 / 1 |
| trial-02-n5 | 5 | 7.342 | 14.479 | 7.137 | 49.29 | 5 / 5 |
| trial-03-n10 | 10 | 8.505 | 18.216 | 9.711 | 53.31 | 10 / 10 |
| trial-04-n1 | 1 | 4.953 | 7.937 | 2.984 | 37.59 | 1 / 1 |
| trial-05-n5 | 5 | 7.866 | 13.499 | 5.633 | 41.73 | 5 / 5 |
| trial-06-n10 | 10 | 7.242 | 14.243 | 7.001 | 49.16 | 10 / 10 |
| trial-07-n1 | 1 | 19.051 | 20.756 | 1.706 | 8.22 | 1 / 1 |
| trial-08-n5 | 5 | 8.820 | 14.194 | 5.374 | 37.86 | 5 / 5 |
| trial-09-n10 | 10 | 8.374 | 16.780 | 8.406 | 50.09 | 10 / 10 |

`trial-07-n1` retained a 10.649 s gap between content-delta arrivals before the first executable node, giving a 20.756 s complete response and an 8.22% window. This delay remains in the sample. The other formal observations also remain individual; no median or quartile was calculated.

In this batch, short observed reasoning delivery and usable first responses left a larger relative overlap than the preceding Qwen batch. The overlap remains a potential generation window, not measured task-completion savings. Different model routes and default thinking behaviour prevent attributing the comparison to model inference speed alone.

The subsequent narration streams for the warmup, `trial-01-n1`, `trial-04-n1`, `trial-06-n10` and `trial-07-n1` failed to finish within the 240 s capture timeout after the test’s abort control. Their complete initial planning responses remain valid. These attempts are not reported as successfully completed conversations. Cleanup and the original task RPC closure were checked before each next attempt.

Evidence is in `/tmp/robonix-streaming-benefit-kimi-20261014.g3wfhgts/`: `results.json` retains every attempt and initial/matched-response timing, `validation.json` checks spacing/dispatch/cleanup, `provenance.json` records configuration and hashes, and `content-gaps.json` records arrival gaps. Raw requests/SSE, parser events and component logs are preserved with `runner.py`, `analyze.py`, `validate.py`, `replay.rs` and the saved `replay-binary`. Captured-stream replay and validation passed. Production Rust source was unchanged, and the test deployment was stopped.

#### Results: 2026-10-16 (DeepSeek)

Source revision `c52884959794`, model `deepseek/deepseek-v4.1-flash` and upstream `api.ofox.io` from the updated `env.sh`. Pilot and Executor binaries, production parser source, task prompts and capture/analysis scripts matched the preceding Kimi run. The office/TIAGo Lite deployment and saved production-parser replay were reused. One N=5 warmup preceded nine formal attempts in the order `[1, 5, 10]` repeated three times, without replacement.

Minimum trial spacing was 5.005 s; minimum upstream POST spacing was 5.002 s. Nineteen model POSTs were attempted. Seventeen received HTTP 200 headers; two subsequent narration requests timed out before upstream response headers arrived. Fifteen POSTs ended with normal stop/DONE/HTTP EOF. The four timeout cases are described below.

Formal initial validity was 8/9 (N=1: 3/3; N=5: 2/3; N=10: 3/3); the warmup was valid. Every published plan uniquely matched the initial response, with no output correction. All 43 dispatched formal sensor calls and five warmup calls succeeded exactly once, with matching dispatch order, no extra calls and no failed/canceled leaf actions. `trial-08-n5` dispatched no calls. Every attempt ended with zero active plans. Successful leaf actions are reported separately from task RPC status.

Times below are seconds from the initial upstream request. The reasoning span measures first-to-last reasoning-delta arrival, not pure server inference time. For the timed-out sample it covers only the received portion before the stall; reasoning-token usage is unavailable. Token counts for other rows are reported by the interface.

| Attempt | N | Reasoning delivery span (s) | Reported reasoning tokens | First content (s) | Initial response |
|---|---|---|---|---|---|
| Warmup | 5 | 0.785 | 116 | 2.288 | Valid |
| trial-01-n1 | 1 | 2.048 | 289 | 3.962 | Valid |
| trial-02-n5 | 5 | 1.419 | 95 | 27.649 | Valid |
| trial-03-n10 | 10 | 2.049 | 282 | 4.230 | Valid |
| trial-04-n1 | 1 | 1.520 | 235 | 17.168 | Valid |
| trial-05-n5 | 5 | 0.546 | 65 | 2.540 | Valid |
| trial-06-n10 | 10 | 2.764 | 334 | 4.341 | Valid |
| trial-07-n1 | 1 | 3.139 | 104 | 9.821 | Valid |
| trial-08-n5 | 5 | 11.423 | — | — | Stream timeout; no content |
| trial-09-n10 | 10 | 0.716 | 110 | 3.263 | Valid |

First-do and full-response times start at the initial upstream request. The window is full response minus first do; its percentage uses that full-response request interval. Subsequent narration, action completion and between-trial cooldowns are outside this interval. Unavailable endpoints remain unavailable.

| Attempt | N | First do (s) | Full plan response (s) | Advance (s) | Window (%) | Successful calls / requested |
|---|---|---|---|---|---|---|
| Warmup | 5 | 3.112 | 4.951 | 1.839 | 37.14 | 5 / 5 |
| trial-01-n1 | 1 | 4.846 | 6.382 | 1.536 | 24.07 | 1 / 1 |
| trial-02-n5 | 5 | 37.645 | 40.927 | 3.282 | 8.02 | 5 / 5 |
| trial-03-n10 | 10 | 5.367 | 7.309 | 1.942 | 26.56 | 10 / 10 |
| trial-04-n1 | 1 | 17.915 | 18.444 | 0.529 | 2.87 | 1 / 1 |
| trial-05-n5 | 5 | 3.775 | 5.385 | 1.610 | 29.89 | 5 / 5 |
| trial-06-n10 | 10 | 5.570 | 8.479 | 2.909 | 34.31 | 10 / 10 |
| trial-07-n1 | 1 | 10.090 | 19.139 | 9.050 | 47.28 | 1 / 1 |
| trial-08-n5 | 5 | — | — | — | — | 0 / 5 |
| trial-09-n10 | 10 | 4.287 | 7.181 | 2.894 | 40.30 | 10 / 10 |

`trial-02-n5` first received reasoning at 26.230 s and content at 27.649 s; a 6.246 s content-arrival gap occurred before its first executable node. `trial-07-n1` retained an 8.845 s gap after its first do, accounting for most of its 9.050 s window. These arrival pauses remain in the individual samples and do not establish stable savings or pure inference timing. No median or quartile was calculated. Potential generation overlap is not measured task-completion savings.

`trial-08-n5` received reasoning but no content, then stopped receiving data. Pilot returned `VLM stream idle timeout`; independent capture continued to its 240 s timeout without normal termination. A truncated SSE record produced a decode error; there was no complete planning response or executable plan to measure.

The subsequent narration requests for `trial-02-n5` and `trial-07-n1` timed out before upstream headers arrived. The recorder returned HTTP 502 locally and the original task RPCs failed, although their initial responses were valid and their requested calls had already succeeded. `trial-04-n1` instead received part of its subsequent narration stream before capture timed out; its initial response and one call succeeded. These attempts are not reported as successfully completed conversations. Cleanup and RPC closure were checked before starting each next trial.

Evidence is in `/tmp/robonix-streaming-benefit-deepseek-20261016.3ut3qtxs/`: `results.json` retains every attempt, timing and task RPC status; `validation.json` checks spacing/dispatch/cleanup; `provenance.json` records configuration and hashes; `content-gaps.json` records content-arrival gaps. Raw requests/SSE, parser events and component logs are preserved with `runner.py`, `analyze.py`, `validate.py`, `replay.rs` and the saved `replay-binary`. Captured-stream replay and validation passed. Production Rust source was unchanged, and the test deployment was stopped.

#### Results: 2026-10-17 (Moonshot API)

Source revision `c52884959794`, model `kimi-k3` and upstream `api.moonshot.cn` from the updated `env.sh`. Pilot and Executor binaries, production parser source, task prompts and capture/analysis scripts matched the preceding runs. The office/TIAGo Lite deployment and saved production-parser replay were reused. One N=5 warmup preceded nine formal attempts in the order `[1, 5, 10]` repeated three times, without replacement.

Minimum trial spacing was 5.004 s; minimum upstream POST spacing was 5.002 s. Twenty-two POSTs were attempted: twenty returned HTTP 200 and ended with normal stop/DONE/HTTP EOF, and two returned HTTP 429. There were no stream timeouts or cleanup errors.

All 9/9 formal attempts produced valid first HTTP 200 planning responses (3/3 for each N), and the warmup was valid. `trial-02-n5` received two HTTP 429 errors before that response; it was not successful on its first upstream POST. No RTDL output correction was needed. All 48 requested formal sensor calls and five warmup calls succeeded exactly once, with matching dispatch order, no extra calls and no failed/canceled leaf actions. All original task RPCs closed without errors; every attempt ended with zero active plans.

The two rate-limit responses reported an account limit of three requests per minute. The existing transport retry mechanism obtained a planning response on the third POST for `trial-02-n5`. The five-second minimum spacing did not prevent those errors. Both failed requests and their cooldowns remain in the record.

First-content times below start at each trial’s earliest upstream POST, including rate-limit retry waiting. Reasoning delivery spans measure first-to-last reasoning-delta arrival within the successful planning response, not pure server inference time. Token counts are reported by the interface.

| Attempt | N | Reasoning delivery span (s) | Reported reasoning tokens | Cumulative first content (s) | Planning response |
|---|---|---|---|---|---|
| Warmup | 5 | 1.236 | 49 | 3.741 | Valid on first POST |
| trial-01-n1 | 1 | 0.204 | 16 | 2.661 | Valid on first POST |
| trial-02-n5 | 5 | 14.986 | 521 | 29.128 | Valid after two HTTP 429 retries |
| trial-03-n10 | 10 | 1.024 | 35 | 10.463 | Valid on first POST |
| trial-04-n1 | 1 | 1.016 | 44 | 4.143 | Valid on first POST |
| trial-05-n5 | 5 | 0.512 | 30 | 4.651 | Valid on first POST |
| trial-06-n10 | 10 | 0.661 | 31 | 9.721 | Valid on first POST |
| trial-07-n1 | 1 | 0.582 | 22 | 4.006 | Valid on first POST |
| trial-08-n5 | 5 | 1.535 | 61 | 5.020 | Valid on first POST |
| trial-09-n10 | 10 | 0.284 | 30 | 11.066 | Valid on first POST |

The next table also starts at the earliest upstream POST. The published plan uniquely matches the first successful model response. The window is within that response; its percentage uses cumulative full-response time, including preceding rate-limit requests and cooldowns. Subsequent narration, action completion and between-trial cooldowns are outside this interval.

| Attempt | Plan response POST # | First do (s) | Full plan response (s) | Advance (s) | Window (%) | Successful calls / requested |
|---|---|---|---|---|---|---|
| Warmup | 1 | 7.836 | 13.571 | 5.734 | 42.26 | 5 / 5 |
| trial-01-n1 | 1 | 4.529 | 8.075 | 3.546 | 43.91 | 1 / 1 |
| trial-02-n5 | 3 | 34.887 | 43.326 | 8.439 | 19.48 | 5 / 5 |
| trial-03-n10 | 1 | 14.144 | 24.765 | 10.620 | 42.89 | 10 / 10 |
| trial-04-n1 | 1 | 6.702 | 10.082 | 3.380 | 33.53 | 1 / 1 |
| trial-05-n5 | 1 | 7.424 | 12.237 | 4.813 | 39.33 | 5 / 5 |
| trial-06-n10 | 1 | 12.238 | 20.466 | 8.229 | 40.21 | 10 / 10 |
| trial-07-n1 | 1 | 6.157 | 9.229 | 3.072 | 33.29 | 1 / 1 |
| trial-08-n5 | 1 | 8.674 | 13.496 | 4.822 | 35.73 | 5 / 5 |
| trial-09-n10 | 1 | 14.321 | 22.612 | 8.291 | 36.67 | 10 / 10 |

For `trial-02-n5`, the successful request alone took 33.073 s to deliver the full response, with an 8.439 s window (25.52%). Earlier HTTP 429 requests and waiting added 10.253 s, giving the cumulative 43.326 s interval and 19.48% shown above. `results.json` retains per-successful-request timing; `request-intervals.json` records the cumulative first-content timing and request offset. Transport retries are distinct from correcting an invalid RTDL reply.

The largest gap between content-delta arrivals in any initial planning response was 0.588 s. All individual observations are retained without medians or quartiles. These are potential generation-overlap windows, not measured task-completion savings. Changing the endpoint and model identifier does not establish a controlled comparison of backend inference speed or identical model versions.

Evidence is in `/tmp/robonix-streaming-benefit-moonshot-20261017.qrcwmioj/`: `results.json` retains every attempt, per-response timing and task RPC status; `validation.json` checks spacing/dispatch/cleanup; `provenance.json` records configuration and hashes; `request-intervals.json` and `content-gaps.json` record cumulative timing and arrival gaps. Raw requests/SSE, HTTP 429 bodies, parser events and component logs are preserved with `runner.py`, `analyze.py`, `validate.py`, `replay.rs` and the saved `replay-binary`. Captured-stream replay and validation passed. Production Rust source was unchanged, and the test deployment was stopped.

## A 类任务明细与覆盖范围

`N` 只统计 `do`，不包含 `sequence` / `parallel`。通过自然语言要求模型生成计划，不能用手写计划代替这部分测试。为控制计数，提示词明确指定步骤、capability、参数和节点数；这是有明确约束的模型生成测试，不衡量开放任务规划能力。

例如 N=2 时，提交文字要求“同一个 sequence 中恰好执行 2 个 do：读取一次激光快照，然后读取一次相机快照”。客户端只发送该任务文本；RTDL JSON、节点及调用标识由 Pilot 的模型生成流程产生。原始任务保存在 `count-02.prompt.txt`，Pilot 返回的计划和执行事件保存在 `count-02.jsonl`，模型输出可在 `pilot.log` 的 `raw_content` 中核对。

| N | 请求任务 | 预期 |
|---|---|---|
| 0 | 仅回复“零动作测试完成” | 没有执行计划和节点，任务为 `done` |
| 1 | 一次激光快照 | 1 个 `do` |
| 2–10 | 同一 sequence：N−1 次激光快照，然后一次相机快照 | 恰好 N 个 `do` |

每档使用独立会话。非零档要求一个执行计划，所有叶节点恰好返回一次 `SUCCEEDED`，终态的 `(plan_id, op_id)` 与计划完全对应，最后任务为 `done`。同时核对 Executor dispatch 日志中的 `call_id` 顺序，排除重复和遗漏。总计应为 55 次调用。

流式判据：同一计划第一次真实 dispatch 的时间，早于 Pilot 收完该轮模型输出、打印 `raw_content` 的时间。不能只凭收到 Plan 事件或任务最后成功判断流式执行，因为 Pilot 可能缓存执行事件后才转发给客户端。

上述计数用例不覆盖运动、导航、运动后的验证、`VERIFYING` overlap、执行中取消或 steer；导航转动另见下方可选用例。传感器接口返回当前缓存帧，多次调用可能取得同一时间戳；调用独立执行不等于每次产生新帧。Pilot 自动进行的 memory retrieval 不计入模型生成的 RTDL `do`。

## 1. 编译与准备

所有命令从仓库根目录执行。需要 Docker、Webots 镜像/资源以及项目正常 build 所需的依赖。Python 需要 `grpcio`、`grpcio-tools` 和 `protobuf`；优先使用已有开发环境，不要为本测试升级项目依赖。

```bash
cargo build --workspace
cargo test -p robonix-pilot -p robonix-executor --all-targets
python3 -c 'import grpc, grpc_tools, google.protobuf'
export RTDL_TEST_OUT="$(mktemp -d /tmp/robonix-rtdl-webots.XXXXXX)"
git rev-parse HEAD > "$RTDL_TEST_OUT/commit.txt"
git status --short > "$RTDL_TEST_OUT/worktree.txt"
```

记下输出目录；后续终端使用同一个绝对路径设置 `RTDL_TEST_OUT`。每轮用新目录，保留失败证据，不能覆盖旧结果后宣称首次通过。附件默认连接本机 Atlas `50051`、Executor `50061`、Pilot `50071`，请在没有其他任务的独占测试实例上运行。

从本次编译产物生成测试客户端，避免使用旧 protobuf：

```bash
python3 - <<'PY'
import os
import pathlib
import subprocess

out = pathlib.Path(os.environ['RTDL_TEST_OUT']) / 'proto'
out.mkdir()
candidates = list(pathlib.Path('target/debug/build').glob(
    'robonix-pilot-*/out/robonix_contracts.proto'))
if not candidates:
    raise SystemExit('先 cargo build -p robonix-pilot')
src = max(candidates, key=lambda p: p.stat().st_mtime).parent
subprocess.run([
    'python3', '-m', 'grpc_tools.protoc', '-I', str(src),
    '--python_out=' + str(out), '--grpc_python_out=' + str(out),
    *map(str, src.glob('*.proto')),
], check=True)
PY
```

## 2. 启动真实 Webots 和 Robonix

终端 A 启动默认 office / TIAGo Lite。以下命令为仿真清除 ALL_PROXY（尤其是 SOCKS 代理），避免本地 Webots 监听端口被代理干扰；保留 HTTP/HTTPS 代理供资源下载使用：

```bash
env -u ALL_PROXY -u all_proxy \
    ROBONIX_SIM_STREAM=1 bash examples/webots/sim/start.sh \
    > "$RTDL_TEST_OUT/sim.log" 2>&1
```

等日志出现 `live odom, RGB, and lidar data received` 和 `webots stream ready` 后，再在终端 B 启动服务：

```bash
set +x
source env.sh
unset ALL_PROXY all_proxy
export PATH="$PWD/target/debug:$PATH"
export ROBONIX_SOURCE_PATH="$PWD"
export SCRIBE_FILE_LEVEL=debug
bash examples/webots/boot.sh --no-update-check \
    > "$RTDL_TEST_OUT/boot.log" 2>&1
```

`env.sh` 应导出 `VLM_BASE_URL`、`VLM_API_KEY`、`VLM_MODEL`；不要输出密钥。每次重新 source 当前文件，并只记录模型名称（不要记录密钥），不能沿用上一轮模型配置。这里使用当前源码编译的二进制，不依赖 `~/.cargo/bin` 的旧版本。`--no-update-check` 避免启动时改变外部包版本；若需要升级外部包，单独处理并记录版本。

在终端 C 确认 boot 结束，检查日志中是否有失败项，并保存能力目录：

```bash
target/debug/rbnx caps --json > "$RTDL_TEST_OUT/capabilities.json"
```

至少确认 Pilot、Executor、`tiago_camera`、`tiago_lidar` 已就绪。完整部署中其他组件失败必须记录；不能仅依据“component(s) up”宣称整栈正常。客户端已配置 `grpc.enable_http_proxy=0`，本地 gRPC 不走代理。

## 3. 执行 A 类主测试：让 Pilot 生成并执行 0–10 个 do

```bash
python3 testing/webots_rtdl/run.py > "$RTDL_TEST_OUT/counts.log" 2>&1
```

逐条检查退出码；任一命令失败时，先看对应日志再继续。`run.py` 默认依次执行 0–10，单档超时 240 秒，首次失败停止；可传入 `4 5 6` 等参数在新目录运行指定档位。执行超时、断连或中断后，不应假定任务已取消；先查询活动计划或关闭本轮实例，再重新运行。

### 执行 B 类可选测试：让 Pilot 生成导航转动计划

底盘 `move` 是供 service 调用的 gRPC 接口，不是模型工具。原地转动测试应走现有 `navigation/navigate`，保留当前 map 位置并修改目标朝向，不能要求模型调用不存在的 `cmd`。

确认 mapping、nav2、Scene 和 scene_verifier 均就绪，且没有其他运动任务后运行：

```bash
python3 testing/webots_rtdl/turn.py > "$RTDL_TEST_OUT/turn.log" 2>&1
```

脚本读取实时 `map → base_link` TF，将朝向增加 45° 的目标写入自然语言任务，通过 Pilot `SubmitTask` 要求模型生成 camera → navigate → camera 三个 `do`；不直接构造 RTDL 或调用 Executor。它保存完整事件及转动前后的独立 TF 观测；通过条件为三个节点全部成功、终态身份匹配、最终朝向误差小于 0.35 rad、位置变化小于 0.25 m。目标表示最终朝向，不保证完整一圈或指定旋转方向；位姿误差容限与逐度精确转动不同。测试超时后先取消活动计划或关闭本轮实例再重试。

2026-10-02 补测：三个节点成功，导航经 `VERIFYING` 后成功；TF 实测转动 31.50°、目标误差 13.50°、位置变化 0.00066 m，符合当前 Nav2 的 0.25 rad 朝向容限，但不代表精确转动 45°。此前一次因验证器继承 SOCKS 代理且缺少 `socksio` 而失败，启动服务时清除 `ALL_PROXY/all_proxy` 后复测通过。声纹服务显存不足未启动，本用例不依赖它。底盘实现和旧文档均与当时 `origin/dev` 一致，未同步不是原先模型找不到 `move` 的原因；本次未更新导航缓存。

### 执行 B2、F 类：平移导航、运动中中断及取消

沿用已启动的 Webots / Robonix 和 `env.sh` 模型。需要 mapping、nav2、Scene、scene_verifier 均就绪，且没有其他任务。每轮使用新的 `RTDL_TEST_OUT` 并按第 1 节生成 proto，然后运行：

```bash
python3 testing/webots_rtdl/navigation.py > "$RTDL_TEST_OUT/navigation.log" 2>&1
```

默认依次运行 `complete`、`abort`、`cancel`；也可在新输出目录中传入一个或多个用例名，例如 `navigation.py abort cancel`。脚本首项失败即停止，保留事件和失败原因；已提交任务时会尝试通过 Pilot 中断并记录清理结果，不以客户端断连代替取消。

脚本将 `ros_observer.py` 复制到仿真容器，启动只读 ROS 观察进程并在退出时关闭。目标选择要求路线距已知障碍至少 0.35 m（部署机器人半径为 0.22 m），近身 0.7 m 内允许传感器盲区的未知栅格，其余路线要求已知空闲；无合适路线时报告前置条件失败，不自动探索。Nav2 仍负责实际规划和避障。选点地图另存为 `navigation-<用例>-map.json`。

成功与否同时根据 Pilot 事件、Executor dispatch 日志和独立 ROS 数据判定。F 类恢复检查也调用真实模型。测试结果和初次失败证据应分别保留；这组测试会实际移动机器人，结束时不保证回到起点。

### 执行 F3–F6、D、G 类扩展测试

真实模型与 Webots 用例在上述部署全部就绪后运行；同一底盘上的测试必须串行：

```bash
python3 testing/webots_rtdl/cancel_live.py > "$RTDL_TEST_OUT/cancel-live.log" 2>&1
```

默认运行 `parallel_abort parallel_cancel sessions plans`；可在新目录传入指定项或重复列表，例如 `parallel_abort parallel_cancel parallel_abort parallel_cancel`。遇到低速转向前置条件问题时，可在命令前添加 `RTDL_CANCEL_ALIGN=1`；这会额外调用真实模型生成一个原地调整朝向的准备任务。F1/F2 仍由 `navigation.py abort cancel` 执行。F7 在独占、空闲的部署上，用新输出目录运行 `python3 testing/webots_rtdl/cancel_generation_live.py`，默认三轮；不要与其他测试同时执行，否则全局活动计划和 dispatch 证据会混淆。

D、G 使用独立本地进程，不需要 Webots。每组设立新的输出目录并按第 1 节生成 proto；G 还需要 Atlas 客户端和 Python `mcp` 包：

```bash
python3 -m grpc_tools.protoc -I system/atlas/proto \
    --python_out="$RTDL_TEST_OUT/proto" --grpc_python_out="$RTDL_TEST_OUT/proto" \
    system/atlas/proto/atlas.proto
# 以下两条分别在各自新的 RTDL_TEST_OUT 中执行：
python3 testing/webots_rtdl/generation_faults.py > "$RTDL_TEST_OUT/generation.log" 2>&1
python3 testing/webots_rtdl/cancel_boundaries.py > "$RTDL_TEST_OUT/boundaries.log" 2>&1
```

D 占用 52151/52161/52171/52181，G 占用 53151/53161/53181/53182；退出时关闭它们自己的进程。真实模型仍读取当前 `env.sh`，独立 D 类则刻意使用本地模拟模型，G 不使用模型。汇总必须分别列出：通过、产品行为失败、模型生成失败和测试前置条件失败。

## 4. 分析 A 类结果：核对真实执行时序

在本轮服务仍运行、日志尚未被下一轮启动覆盖时复制日志：

```bash
cp examples/webots/rbnx-boot/logs/pilot.log "$RTDL_TEST_OUT/pilot.log"
cp examples/webots/rbnx-boot/logs/executor.log "$RTDL_TEST_OUT/executor.log"
python3 testing/webots_rtdl/analyze.py > "$RTDL_TEST_OUT/analysis.log" 2>&1
```

分析脚本要求全部 11 档结果齐全，并检查调用顺序、次数与流式时间差。它将结果写回 `count-XX.summary.json` 的 `execution_checks` / `streaming_pass`；全部通过退出码才为 0。必须有 debug 级 `raw_content`，缺失时不能证明流式时序。时间解析使用本机时区，应在启动服务的同一机器和时区运行分析，不要对异机时钟直接比较。

正时间差表示首次 dispatch 早于完整生成结束。若模型或网关一次性缓冲了整个响应，可能只通过执行检查而不满足这个时序判据；应检查上游 SSE 分块，报告“本次未证明边生成边执行”，不要直接归因于 Executor。

还需人工检查：

- `count-XX.jsonl` 中 lidar 的 `ranges` 非空，camera 为有效 640×480 图像；比较传感器时间戳，注明重复帧。
- `final_text`、`task_state`、节点描述是否出现 `�`，自然语言汇报是否与实际结果一致。
- 回复“无法执行/未完成”时不能仅因 `status=done` 判为测试通过；本脚本还核对实际调用数量。
- 输出目录包含图像和完整 prompt/response；分享日志前检查敏感信息，不复制 `env.sh`。

## 5. 执行 C 类补充测试：直接检查 Executor 协议

此项独立于第 3–4 节主测试。`protocol.py` 绕过 Pilot 和模型，直接构造节点及控制消息发送给 Executor；lidar / camera 调用仍在真实 Webots 中执行。这样可以精确控制缺少 `END`、非法父节点等输入，验证 Executor 的协议行为。

```bash
python3 testing/webots_rtdl/protocol.py > "$RTDL_TEST_OUT/protocol.log" 2>&1
```

脚本运行三个独立用例（正常嵌套、提前 EOF、非法父节点），每例先执行相同的成功前缀，再分别走对应分支；下面不是在一个计划上依次制造三个结果：

1. 先发送 sequence 和一个真实 lidar 调用，暂不发送 `END`；必须已经收到该调用的成功终态，且没有 `plan_complete`。
2. 正常用例追加 parallel（lidar + camera），依次关闭内外 operator，发送 `END`；三个叶节点成功，计划完成并清理。
3. 提前 EOF 用例在第一个叶节点完成后直接结束输入；期望 `INVALID_ARGUMENT`，计划清理。
4. 非法父节点用例在成功前缀后追加 `parent_index=99`；期望 `INVALID_ARGUMENT`，计划清理。
5. 最终 `ListActivePlans` 必须为空，保存为 `active-plans-final.json`。

提前 EOF 测试覆盖已完成前缀的清理，不代表验证了执行中的异步动作取消。嵌套 parallel 用例检查完成和收尾，不测量两个短调用的并发速度。

## 6. 清理

先归档结果，再关闭本轮启动的服务和仿真。以下命令针对默认 Webots 部署；不要关闭其他人的实例：

```bash
target/debug/rbnx shutdown -f examples/webots/robonix_manifest.yaml
docker compose --project-name robonix_tiago_sim \
    -f examples/webots/sim/compose.yaml down
```

确认测试进程已退出、容器已停止。不要在复测失败时直接重复 boot，遗留进程会干扰端口和计数。

## 扩展取消实测：2026-10-04（含未通过项）

重新读取本轮 `env.sh`，实际模型为 **`openai/gpt-6-luna`**；代码为 `08e68457` 加现有暂存的 Pilot 修改，未修改生产代码。完整报告在本机 `/tmp/robonix-cancel-20261004/report.txt`，下列路径均相对此目录；所有尝试保存在 `live-attempts.json`，不只保留成功样本。

| 类别 | 结果 | 证据目录 |
|---|---|---|
| F1 串行导航中断 | 1 轮通过，3 总节点 / 2 do | `live-baseline-ready/` |
| F2 串行自然语言取消 | 本次导航计划生成/解析失败，未进入取消阶段；不能记为通过 | `live-baseline-ready/`、`pilot-generation-errors.json` |
| F3 并行中断 | 2 轮完整通过，每轮 7 总节点 / 4 do；两个执行中动作停止、两个后续传感器动作未 dispatch | `live-extended/`、`live-repeat-final/` |
| F4 并行自然语言取消 | 3 轮完整通过，每轮 7 总节点 / 4 do；四个 do 均取消，并独立确认停车 | `live-extended/`、`live-repeat/`、`live-repeat-final/` |
| F5 跨会话隔离 | 通过；A 停止，B 命令和激光自然完成；每计划 3 总节点 / 2 do | `live-extended/sessions-*` |
| F6 同会话多计划隔离 | 通过；准确取消导航计划，独立计划自然完成；每计划 3 总节点 / 2 do | `live-aligned/plans-898a6b52-*` |
| F7 真实模型生成期间中断 | 3/3 通过；实际提交前缀分别为 3/2/1 do（总节点 4/3/2），全部取消，后续传感器零调用 | `live-generation/` |
| D 可控模型故障 | 5/5 通过，含尚无节点和已开始执行两种中断时机 | `generation-final/` |
| G Executor 边界和底层故障 | **21 项中 18 通过、3 未通过**；失败详情见下表 | `boundaries-final/` |

**不能据此说 cancel 全部正确。** 三个底层故障用例均出现“计划被清理，但服务还在运行”：

| 注入输入 | 实际输出 | 问题及证据 |
|---|---|---|
| cancel 返回 `accepted=false` | `success=true, completed=true`，叶节点 CANCELED，活动计划 0；服务仍 RUNNING | 未识别底层拒绝；`provider_reject.json` |
| cancel 抛出 MCP 错误 | 仍为 `success=true, completed=true`；message 有失败说明，服务仍 RUNNING | 结构化状态和计划清理不能作为已停车证据；`provider_error.json` |
| cancel 延迟 10 秒 | `wait_ms=2000` 未限制此前的底层调用，客户端 4 秒期限到达；计划清理而服务仍 RUNNING | 取消请求时限和实际停止均不能由计划状态保证；`provider_delay.json` |

这些是在独立服务中注入的失败，不表示本轮真实 Nav2 发生了相同拒绝/超时。源码检查对应 `system/executor/src/active_plans.rs` 的 `cancel_async_call`、`cancel_plan_control` 及 `grpc_server.rs` 的控制响应；本轮只测试和记录，没有修复生产代码。

真实模型的 19 个完整尝试记录中，11 个通过，另外 8 个须分别解释：两次生成/解析报错（`unexpected RTDL field :{` / `{`，缺少失败时的原始模型字节，尚未确认模型还是解析器根因）；两次低速导航未在 60 秒内达到平移触发条件；一次测试脚本误把取消回复结束当成所有独立计划结束；三次朝向准备多出了只读位姿查询，与原先“一次调用”的准备条件不符。后两类调整测试观察方式/明确允许的准备查询后复测通过，原记录仍保留。较早的未就绪观察器超时、测试服务 lifecycle 注册配置错误另行保留，不算业务测试通过。

G 的完成竞态覆盖到成功和取消两种先后顺序，均只有一个叶子终态；`VERIFYING` 用例保留已完成动作的验证成功结果，同时取消后续动作，短等待先返回 `completed=false` 后清理。它们是可控输入测试，不是实际 Nav2 验证阶段竞态的完整覆盖。

本轮重新编译和新增 Python 语法检查通过，最终恢复部署原速度 100%，`active-final.json` 为 0。有限重复仍不能排除全部时序问题；多计划测试只含一个物理导航，另一计划是本地命令与传感器，不覆盖同一底盘同时执行两个导航目标。此前 0–10 主测试未在本轮重跑。

## 导航、中断及取消实测：2026-10-03

本轮在 `08e68457` 加现有暂存的 Pilot 修改上重新编译，使用 `env.sh` 的真实模型和 Webots。三项均通过；完整报告为本机 `/tmp/robonix-nav-control-20261003/report.txt`，以下证据均相对此目录。输出为临时产物，未纳入仓库。

| 测试 | 实测输出与判定 | 证据 |
|---|---|---|
| B2 平移导航 → 相机 | 两个 `do` 各执行一次并成功，观察到 `VERIFYING`；实际平移 1.247 m，位置误差 0.205 m、朝向误差 0.158 rad | `run4/navigation-complete-summary.json`、对应 `.jsonl` |
| F1 运动中 `abort_turn` | Pilot 返回 `interrupted`，原 ROS goal 为 CANCELED；导航一次、相机零次；约 0.21 s 后观察到 action 取消，约 0.41 s 后进入稳定静止，额外位移约 0.007 m | `run5/navigation-abort-summary.json`、`navigation-abort-control.jsonl` |
| F2 运动中自然语言取消 | 真实模型生成 `cancel_plan`；两个 RTDL 叶节点均 CANCELED，原 ROS goal 取消；导航一次、相机零次；约 5.30 s 后观察到 action 取消，约 5.50 s 后进入稳定静止，额外位移约 0.136 m | `run5/navigation-cancel-summary.json`、`navigation-cancel-control.jsonl` |

两个控制用例均确认连续至少 3 秒静止、活动计划归零，同会话后续激光任务成功；导航速度由测试前读取的 100% 临时降至 20%，结束后恢复 100%。独立位姿、速度和 action 状态保存在各轮 `navigation-ros.jsonl`；`results.json` / `summarize.py` 保存时延与位移统计。观测频率约 5 Hz，额外位移从控制请求前最近一次采样计算，数字是本次观测值，不是实时性能保证。

**自然语言取消要等待模型生成控制操作，不能视为立即停车。** 本轮 `abort_turn` 更快。此前一轮也通过两种控制，但自然语言取消约 6.29 s 后才进入稳定静止；因此不能只检查最终“已取消”而忽略停车延迟。

准备过程中发现的测试脚本问题也保留：地图近身未知栅格及原 0.45 m 选点余量导致部分用例在提交任务前停止；地图增大曾超过观察进程读取器的默认行长。调整选点规则及读取上限后复测；`run1`–`run4` 的前置失败记录未覆盖，不能算作对应控制用例通过。最终平移证据在 `run4`、两项控制证据在 `run5`。本轮只新增并实测 B2、F，没有重跑 A、C、D；下节是此前完整流式复测结果。

## 此前流式基线复测：2026-10-03

测试代码为 `08e68457` 加当时暂存的 `planner.rs`、`stream_execution.rs` 修改。结果保存在本机 `/tmp/robonix-rtdl-20261003/`，完整报告为 `report.txt`；该目录是临时测试产物，未纳入仓库。

| 类别 | 本轮结果 | 证据文件（上述目录内） |
|---|---|---|
| 真实模型主测试 | 0–10 档通过，55 个 `do` 各成功一次；首次 dispatch 领先完整输出 0.55–2.60 秒 | `run.log`、`count-*.prompt.txt`、`count-*.jsonl`、`analysis.log` |
| 真实模型转动 | 三个节点成功，导航经 `VERIFYING`；目标 45°，实测 31.59°，误差 13.41°、位移 0.0232 m，满足现有容差，不代表精确转动 45° | `turn-input.json`、`turn.jsonl`、`turn-summary.json` |
| 手写 Executor 协议输入 | 三个用例通过，活动计划清理 | `protocol.log`、`protocol-*.json` |
| 额外模拟模型故障注入 | 异常/不完整输出、EOF、流超时、主动中止四项通过；已开始动作取消，后续动作未执行，计划清理 | `faults.py`、`faults.log`、`fault-*.json` |
| 工作区检查 | build、fmt、全部测试、Clippy 通过；两个既有 CLI fixture 测试忽略 | `build.log`、`fmt.log`、`tests.log`、`clippy.log` |

故障注入使用独立本地 Pilot / Executor 和可控模拟模型输出，不使用 `env.sh` 的真实模型，也不驱动 Webots；它是本轮额外检查，脚本尚未纳入本文附件。主测试和转动用例均使用真实模型。真实模型事件未发现 Unicode 替换字符，最终活动计划数为 0；本轮未覆盖 steer 并发或更复杂的导航任务。

## 历史基线与当时发现的问题（2026-10-02）

2026-10-02，`d54bae86`，`env.sh` 模型 `openai/gpt-6-sol`：11 档执行检查通过，55 个 `do` 恰好各成功一次；首个 dispatch 领先完整生成约 0.65–2.86 秒。三个协议用例通过，最终活动计划数为 0；Pilot 78 个、Executor 52 个单元测试通过。

该次测试还发现以下问题，不能被上述通过结果掩盖：

- 中文出现“顺��”。`system/pilot/src/vlm.rs` 按网络分块执行 `String::from_utf8_lossy`，存在跨 UTF-8 字符边界损坏的风险；当时未捕获上游原始字节，根因仍需针对性复现。
- 请求不可用底盘接口时，模型有时把“报告无法执行”加入成功条件并输出 `done`，Pilot 接受该状态。可用明确不可用的 capability 请求单独复测，检查零执行、拒绝回复与任务状态的一致性。
- 底盘 `move` 实际为不向模型暴露的 gRPC 接口，`CAPABILITY.md` 仍描述旧调用方式；因此没有用底盘动作凑节点数。
- Scene 曾因过期生成代码缺少 `DeleteObject_Request` 启动失败。用当前 `rbnx codegen -p system/scene --mcp` 重新生成后类型恢复，但当次没有重启验证 Scene；本套计数用例不依赖它。
- 部分连续 lidar 调用返回相同时间戳，这是必须单独记录的传感器更新限制。

源码和环境变化后以新一轮实测为准，不把这些历史问题当成已修复或仍然存在的结论。

后续处理：UTF-8 跨分块损坏已通过针对性测试复现并修复（`08e68457`）；底盘说明已改为通过导航调用，Scene 在重新生成代码并重启后已参与转动测试。`done` 表示本轮回复结束，拒绝执行后结束回复可以使用该状态；它本身不表示机器人动作成功，因此主测试始终另行核对节点实际结果。

## 2026-10-05：RTDL 生成失败定位

本轮重新读取 `env.sh`，模型为 `openai/gpt-6-luna`，使用当前源码编译的 Pilot / Executor 和真实 Webots，导航测试速度为 100%。输出目录为 `/tmp/robonix-rtdl-diagnose-20261005/`。在 Pilot 与当前模型接口之间加入只转发、不修改请求正文和响应正文的记录代理，保存请求正文、上游 SSE 字节、接收分块及其时间，不保存认证头。Pilot 提前断开后，代理仍收完上游回复以保留错误尾部。

### 已复现：串行导航生成非法字段 `:{`

- **输入：** 自然语言要求一个 sequence、恰好两个 do：导航到实时地图空闲目标 `(1.0253042958276828, 1.0253053701513688)`，然后拍照。输入见 `live-serial/navigation-cancel-input.json`。速度读取和设置均通过真实模型，实际速度为 100%。
- **上游输出：** `capture/1791206094981129994.content.txt` 是 SSE `delta.content` 原样拼接结果。完整回复是语法合法的 JSON，但第一个导航 do 出现非法字段 `":{": ""`；只有一个 do，没有相机节点；`task_update` 被放进 `rtdl` 对象，顶层缺少它。接口正常返回 `finish_reason=stop` 和 `[DONE]`，不是输出被截断。
- **Pilot 输出：** ``unexpected RTDL field `:{` ``，拒绝节点，未提交任何执行节点。串行脚本没有及时传播后台收集任务的 RPC 异常，最终显示 `Pilot did not publish a plan` 超时；真正错误须看 Pilot 日志，不能把脚本超时当作根因。
- **独立重放：** 将捕获的内容按原始 content delta、整个字符串、逐 Unicode 字符三种方式送入当前生产 `RtdlIncrementalParser`。三次均在累计 619 个 UTF-8 字节时，报同一个非法字段错误，提交节点数均为 0；完整回复为 1522 字节。结果见 `capture/replay-results.json`，临时测试源码见输出目录的 `replay_test.rs`。临时 Rust 诊断测试编译后已移除，生产 Rust 源码未修改。
- **结论：** 本次非法字段已经存在于模型接口返回的原始内容中，Pilot 在 `rtdl_stream.rs` 的节点字段白名单检查中正确拒绝；不是这次输入的分块、UTF-8 解码或流式拼接导致。请求使用 `response_format={"type":"json_object"}`，未传 RTDL JSON Schema；请求中的结构要求来自提示词，合法 JSON 仍可能不符合 RTDL。证据只定位到上游输出侧，不能进一步区分模型解码与上游网关内部处理，也不据此认定历史另一次 `{` 报错已复现。

`diagnosis.json` 汇总错误路径、请求格式和三种重放结果。原始 SSE、请求正文、分块和完整内容均在 `capture/`，没有用手写计划替代此次失败的真实模型输出。

### 第二种生成失败：空 sequence 漏掉 children

- **输入：** `live-parallel/parallel_cancel-aa075c35-input.json` 中的自然语言并行任务：导航→相机、等待 45 秒命令→激光，共 4 个 do；在真实运动中发送只取消计划 18 的自然语言指令。
- **输出：** 初始计划正确生成 7 总节点 / 4 do。后续上游回复 `capture/1791206336147301435.content.txt` 声称不再派发动作，却输出 `{"op":"sequence","op_id":0,"description":"无待执行动作"}`，没有必需的 `children: []`。Pilot 返回 `operator children must be an array`，控制用例未通过。
- **重放：** 原始分块、整段、逐字符三次都在第 388 个 UTF-8 字节处得到相同错误，回复共 704 字节；完整回复是合法 JSON，正常以 `stop` / `[DONE]` 结束。这个错误同样已存在于上游输出中。
- **执行范围：** 这轮导航、拍照、等待命令、激光均自然完成，不能声称自然语言取消正确阻止了后续动作。本节定位的是其后续回复的 RTDL 结构错误，未据此推断取消未及时生效的完整原因。

本轮三个真实模型 / Webots 主用例：串行自然语言取消在生成阶段失败；并行硬中断通过（100% 速度，4 个 do，真实停车，后续相机和激光零 dispatch）；并行自然语言取消未通过，后续回复缺少 children。此前历史 `{` 非法字段此次未复现，不能直接替它下结论。最终活动计划为 0，速度恢复为 100%。记录代理也捕获了两次 Scene 的非流式空间关系请求，已从 Pilot RTDL 重放中排除，不能把它们当成空模型回复或新增生成失败。
