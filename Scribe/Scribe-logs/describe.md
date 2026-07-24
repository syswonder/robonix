
Github Issue：https://github\.com/syswonder/robonix/issues/63

## 1\. 功能描述

Scribe 是全系统 **统一日志入口**：各组件不再各自 `print` 或 `logging`。一条日志由时间、级别、来源（tag）、内容构成；同时输出到 console（人读）与文件（按组件分文件）。`tag` 直接取组件名或 `provider_id`，便于按来源过滤。设计借鉴 Android 日志系统的统一入口加 tag 加级别（参考 logcat / logd 思路），不照搬其实现。

## 2\. 维护的数据、对应的解释和描述

```Rust
enum Level { Debug, Info, Warn, Error }

struct LogRecord {
    ts:    u64,        // 纳秒时间戳，后续接 chronos now() 统一时钟
    level: Level,
    tag:   String,     // 组件名 / provider_id
    msg:   String,     // 日志正文
}
```

|字段|说明|
|---|---|
|`ts`|产生时刻；过渡期用 `time.time()`，后续切 chronos `now()`|
|`level`|Debug / Info / Warn / Error|
|`tag`|来源标识，初步规定填写 `provider_id`，作过滤维度|
|`msg`|日志正文|

## 3\. 功能接口（输入输出、给谁来调用）

### 对外接口

Scribe 不是 capability，无 `contract_id` 与 ROS IDL：`log()` 是进程内 library（Rust trait，Python 侧 robonix\-api 暴露同语义 logger），`rbnx logs` 是 CLI；接口由 trait 签名与 CLI 定义。若日后需跨进程远程查询日志，可另设计 contract `robonix/system/scribe/query`（gRPC，`scribe/srv/Query.srv`）。

```Rust
trait Scribe {
    fn log(&self, level: Level, tag: String, msg: &str);
    // 另提供便捷封装 debug / info / warn / error(tag, msg)
}
```

|接口|输入|输出|transport|谁调用|
|---|---|---|---|---|
|`log(level, tag, msg)`|级别、tag、内容|—|进程内 library|所有组件（Rust 与 Python）|
|`rbnx logs`|过滤条件（tag 或级别）|日志流|CLI|开发者 / 运维|

### 依赖的能力接口

无能力依赖。时间戳取 chronos `now()`（进程内 library）。

## 4\. 执行图

```
flowchart TB
    subgraph scribe["scribe"]
        direction TB
        LogRecord["LogRecord"]
        console["console"]
        files["files (per tag)"]
        
        LogRecord --> console
        LogRecord --> files
    end

    components["components"] -->|"log(level, tag, msg)"| LogRecord
    developer["developer"] -->|"rbnx logs"| files
```

各组件经 `log(level, tag, msg)` 调用 scribe，scribe 组装 `LogRecord` 后同时格式化输出到 console 并按 tag 写入 `logs/<tag>.log`。开发者经 `rbnx logs` 按 tag 与级别读文件过滤查看。

## 5\. 源代码组织

`system/scribe/`（rust crate）加 Python 侧 logger（组件含 Python primitive 与 service）。

```Bash
system/scribe/
  Cargo.toml
  src/
    lib.rs      # trait Scribe，Level / LogRecord，便捷封装 debug / info / warn / error
    sink.rs     # 输出后端：console formatter 与 per-tag 文件写入，轮转可选
    format.rs   # 单行格式：时间 级别 [tag] msg
# Python primitive / service 侧：robonix-api 暴露同语义 logger
# CLI：rbnx logs，位于 tools/rbnx，按 tag 与级别读文件过滤
```

## 6\.  日志格式

磁盘格式（每行一个 JSON 对象）：

```JSON
{"ts":1765432100123456789,"lvl":"I","tag":"scene_svc","msg":"object registered","sid":"sess-42","pid":"plan-7","cid":"plan-7:3","kv":{"object_id":"scene.object.cup_003"}}
```

console 端\(以及 rbnx logs 的默认渲染\)是 logcat 风格单行文本:

```JSON
06-11 10:23:45.123  I scene_svc   object registered  object_id=scene.object.cup_003 plan=plan-7
```