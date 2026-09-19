# scene webui — 未决问题（2026-09-19 收工记录）

## 1. 右侧 dock 仍在刷新，加上过渡后一闪一闪
现象：面板内容每个 tick 仍有重绘，`transition` 让重绘变成可见的闪烁。
已做：对象表改为按 id 增量更新（`syncObjects`），relations / robot 两块改为
      「渲染结果与现值不同才写 innerHTML」。
仍不够：
  - `drawChips` 之类每 tick 重建 innerHTML 的地方还没排查完；
  - 过渡作用在每 tick 都被重新赋值的属性上时会反复触发，
    应当只对「状态变化」加过渡，而不是对「被重写」加过渡。
下一步：逐个 pane 审一遍写入点，凡是每 tick 必写的改成 diff；
      dock 内的 transition 只保留在 hover / active / 选中这类真状态上。

## 2. 按钮风格仍未统一
已做：`shell.css` 增加共享 `.btn / .field / .tag`；maps 页与 dock 的动作按钮
      已改用它们。
仍不统一：
  - `map_page.html`（regions 编辑页）还是自己那套 `.primary` / `button` 样式，
    它是 iframe 页，拿不到 shell.css；
  - `logs.html` 的工具条按钮也还是局部定义。
下一步：要么把共享控件抽成独立 `controls.css` 并注入到 iframe 页，
      要么把 regions 页也并入 shell 渲染（它本来就该重构）。

## 3. rerun viewer 与启动死锁（二选一，尚未两全）
- 死锁已诊断清楚：`RecordingStream.serve_grpc` 在 asyncio + rclpy + CUDA 争用下
  不返回且持有 GIL，`worker.join(20s)` 永远等不到调度（栈已抓到）。
- 「服务器移出进程 + connect_grpc」能消除死锁，但 viewer 收不到数据：
  三条路径验证过（scene 自身数据、`rr.log(recording=)`、`rec.log()`），
  经 scene 代理与子进程自带 viewer 均为空白。
- 当前取舍：数据面回到进程内 `serve_grpc`（viewer 可用），死锁靠重启绕过。
下一步：需要一个既不在进程内托管服务器、又能把数据送达的方案。

## 4. 其它
- 中文地图名：scene 侧已允许，但 `service-map-rbnx` 的 `_sanitize_map_id`
  仍是 ASCII 白名单，`客厅` 落盘成 `__`。跨仓库，待定。
- 10 条带变量插值的状态文案仍是英文，需要带占位符的翻译机制。
- 卡片详情里 `artifact path` / `preview path` 为空：`list_maps` 不返回这两个字段。
- 卡片详情两行标签都叫 `artifact`（健康状态与体积），需要区分。
