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

### 今晚抓到的证据（2026-09-19 深夜）
一次真实的 CMD_ACTIVATE 超时被完整抓下来了，不再是推测：

- Python 栈：`scene-rerun-up` 线程停在 `bindings.serve_grpc`
  （`rerun_sdk/rerun/sinks.py:407`），主线程停在 `rerun_sink.py:492`
  的 `worker.join()`。py-spy 连续采样，帧完全不动。
- 该线程被标为 `active+gil`，但容器 CPU 只有 1.3%、所有线程 `state=S`。
  所以不是自旋，是**原生调用里睡着了却没有释放 GIL**。
- 容器内没有任何端口被监听：卡在 bind 之前。

### 隔离测试：单独跑它一切正常
同一个镜像里：

| 场景 | 结果 |
| --- | --- |
| `serve_grpc` 在主线程 | 0.001 s 返回 |
| `serve_grpc` 在子线程 | 0.001 s 返回 |
| 先 `import torch` / `pymilvus` / `grpc` | 0.001 s 返回 |
| 先 init rclpy 并持续 spin | 0.002 s 返回 |
| 直接跑仓库自己的 `RerunSink().start()` | 两个 feed + web viewer 全部起来 |

也就是说：触发条件不是 import，不是 rclpy，也不是 sink 自己的逻辑，
而是**激活时刻进程已经处于的运行态**（milvus-lite、嵌入模型/CUDA、
两个 grpc server、uvicorn、事件循环同时活着）。还没定位到具体是哪一个。

### 一个必须承认的结论
`worker.join(_START_TIMEOUT_S)` 这条超时**在原理上就不可能生效**：
卡住的线程持有 GIL，主线程要超时返回就得先拿到 GIL。同理，任何用
Python 写的看门狗都救不了——它自己也要 GIL。**进程内不存在补救手段。**

这把「把服务器移出进程」从一种偏好变成了唯一出路，也就意味着之前
那个「viewer 空白」的问题不能再绕开，必须正面解决。

- 已验证过的失败路径：服务器移出进程 + `connect_grpc` 可消除死锁，但
  viewer 收不到数据；三条路径（scene 自身数据、`rr.log(recording=)`、
  `rec.log()`）经 scene 代理与子进程自带 viewer 均为空白。
- 当前取舍仍是：数据面留在进程内（viewer 可用），死锁靠重启绕过。

### 又缩小了一圈（同晚稍后）
- 再抓到一次，这次卡在**第二个** feed 的 `serve_grpc`，而且 9876/9877
  **都已经 LISTEN**：端口绑定成功之后才卡住。所以和端口占用无关。
- 「一个进程里连着调两次 serve_grpc」本身没问题：在空容器里连跑 12 次，
  12/12 都是 1ms 返回，0 次 hang。
- 所以触发条件还是「进程里同时有很多活跃线程」。结合「睡着但持有 GIL」
  这一点，最像的机制是 **Rust 侧在持有某个自己的锁时回调进 Python 取
  GIL**，而另一个线程正拿着那个锁等 GIL——空进程里没有竞争者，所以永远
  不会发生。这个还没证实。

### 可以先做、成本很低的一招
把 viewer 的 bring-up 挪到进程还安静的时候（rclpy / CUDA / milvus 起来
之前），而不是放在激活阶段。空进程里是 0/12，越安静越不容易撞上。这不
解决根因，但能把概率压下去，而且本身就说得通：数据汇应该先于数据存在。

下一步（真正的解法）：查清子进程模式下数据为何送不达——优先怀疑
recording id / `application_id` 与 viewer `url` 查询参数不匹配，而不是
传输本身。

## 4. scene 自己的日志曾经是被销毁的（已修）

排查上面那些问题时最大的阻力：**scene service 的 logging 输出一条也没有**
——不在容器 stdout，不在 scribe，logs 页面里除了 scene 自己什么包都有。
`SCENE_LOG_LEVEL` 一路透传进 docker，其实毫无作用。

用探针抓到了确凿证据：

| 时刻 | root.handlers | 一条 INFO 是否出现 |
| --- | --- | --- |
| import 时 | `[StreamHandler]` | 出现 |
| 激活时 | `[_StdlibBridgeHandler]` | 消失 |

即 scribe 的 bridge 在 bootstrap 时装到 **root logger** 上，且
`replace_existing_handlers=True` 把 `basicConfig` 刚装的 console handler
删掉；而它自己在容器里写不出去（`SCRIBE_LOG_DIR` 是 `:ro` 挂载）。于是从
bootstrap 起，scene 的日志既不落盘也不进控制台，是被**销毁**而不是被过滤。

后果不只是不方便：`[scene-rerun] publish failed` 这种「后台任务自报死亡」
的日志，报给了空气。

修法：把 handler 装在 scene 自己的 logger 上（bridge 只动 root），且只在
容器里装——容器里 rbnx 本来就把我们的输出接进 scribe，所以控制台**就是**
通往 scribe 的路。原生部署维持 bridge 原样，不会重复两份。
现在 `scene.log` 里能看到 17 条 `[scene-service]` 启动日志了。

## 4. 镜像 tag 被两个 worktree 抢（今晚踩到）

`system/scene/scripts/start.sh` 里 `IMG="${ROBONIX_SCENE_IMAGE:-robonix-scene}"`，
两个 worktree 默认都用 `robonix-scene` 这一个可变 tag。今晚该 tag 指到了一个
**没有 rerun** 的镜像（`.Created` 是 09-13），于是：

- `RerunSink.start()` 报 `rerun-sdk is not installed`，viewer 退回内置画布；
- shell 因此不渲染右侧 dock，4 条 dock 测试连续两轮失败；
- 页面看上去几乎正常，没有任何地方说明后端被换掉了。

排查花了很久，直到状态条加上「可视化后端」那一格，一眼就看到 `built-in`
和 hover 里的原因。带 rerun 0.37.1 的镜像还在，只是 tag 被移走成了 dangling，
已重新打成 **`robonix-scene-248:latest`**，248 的启动脚本设
`ROBONIX_SCENE_IMAGE=robonix-scene-248`，不再和 dualmap 抢同一个 tag。

待办：`start.sh` 的默认值不能单方面改（会影响 #243），是否给两个 PR 各自
固定 tag 需要和另一个 session 对齐。

## 4. 其它
- 中文地图名：scene 侧已允许，但 `service-map-rbnx` 的 `_sanitize_map_id`
  仍是 ASCII 白名单，`客厅` 落盘成 `__`。跨仓库，待定。
- 10 条带变量插值的状态文案仍是英文，需要带占位符的翻译机制。
- 卡片详情里 `artifact path` / `preview path` 为空：`list_maps` 不返回这两个字段。
- 卡片详情两行标签都叫 `artifact`（健康状态与体积），需要区分。
