# scene webui — 进展与未决问题（2026-09-20 凌晨）

## 已解决（2026-09-19 夜 ~ 20 日凌晨）

**dock 闪烁**。真因不是过渡太慢，是过渡**反复重启**：`renderDetail()` 每
tick 重写整个详情的 `innerHTML`，`.detail` 上的 `dock-in` 淡入于是每秒重放
两次。改成「选中变了才重建结构，否则只写变化的值」。验证不靠肉眼：页面上
挂 `MutationObserver` 跑 8 秒，结构重建 0 次、重复文本写入 0 次，剩下的
childList 变动全部是真实值在动（observation 427→430、位姿末位抖动）；再监听
`animationstart` / `transitionstart` 10 秒，0 次触发。

**按钮风格**。抽出 `controls.css`，每个变量带兜底值，所以 iframe 页也能用；
`__CONTROLS__` 占位注入 `map_page.html` 和 `combined.html`（后者是裸返回的，
原本根本拿不到）。审计发现还有 **13 个按钮**从来没戴上 `.btn`——dock 的
Rename / Delete 直接渲染成浏览器原生灰按钮。共享文件补了 `.btn.small` 和
`.btn.icon` 两个尺寸修饰符（这正是当初漏掉它们的原因：没有能放它们的地方），
三个页面交出各自的局部定义。现在 `grep` 不到任何没有 `.btn` 的按钮。

**插值文案**。表里现在存「带 `{name}` 洞的整句」，`tv()` 填洞；填进去的值
存在元素上（`data-i18n-vars`），所以切语言是重新填一遍而不是冻结在上一句。
中文能把 id 放在和英文不同的位置——这正是当初拼接做不到、只能留英文的原因。

**卡片详情**。英文里 `maps.health` 和 `maps.size` 都叫 "artifact"（中文本来
是分开的），改为 "artifact health" / "artifact size"。两条路径不再是永远的
`—`：provider 确实不给路径，但 scene 自己每次出预览图都在开这些文件，所以
改由 `/contents` 汇报，且**文件不存在就不出这一行**。

**regions 页**。它的本职是「圈住一组东西」，而那些东西原本画成 3.5px、60%
透明、颜色和机器人几乎一样的点——你看得见有东西，看不出是什么，也分不出
哪个是机器人。2D 图早就把这件事做对了（类别配色 + 常驻标签 + 退火排版），
于是把那套抽成 `web_assets/labels.js`，两张图共用，而不是在 regions 页再写
一份更差的。顺带修了配色表：原本 14 个类之外全是同一个灰，而感知实际产出的
`potted_plant` / `picture_frame` 都不在表里。

**侧边栏概念**。`semantic map` / `2D map` / `regions` 不是 `maps` 的同级——
前三个是**同一张地图**的三个视图，第四个是另一种东西的库。平铺列出等于说
机器人有四张地图。现在分组：「当前地图」标题下挂 3D / 2D / 区域，库和仪表
各自独立。

## 1. rerun viewer 与启动死锁（二选一，尚未两全）

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

### 进程外方案：五个实验，断点已定位（2026-09-20）

"移出进程" 这条路以前只留下一句 "viewer 空白" 就停了。这次查到底了。

**先说结论**：进程外**确实消除了死锁**（连续 5 次启动 5/5，对照：改动前
约 2/3、把拉起提前后 4/6）。但 **viewer 收不到数据**，断点在
`rerun --serve-grpc` 的 server 与 0.37.1 web viewer 之间，不在我们的管道里。
所以**已回退**——拿死锁换一个死掉的 viewer 不是修复，是换一种坏法。

| # | 实验 | 结果 |
| --- | --- | --- |
| 1 | 子进程 `rerun --serve-web`，SDK `connect_grpc` 连上去 | `connect_grpc` **0.14s 返回**（对照 `serve_grpc` 会无限期持 GIL） |
| 2 | 浏览器开子进程自带的 web viewer | 空白 |
| 3 | 浏览器用 **scene 自己那份能用的 viewer 资源**，经 scene `/proxy` 转发到子进程 | 空白 |
| 4 | 同上但**绕开 scene 的转发**，直连子进程端口 | 空白 → **不是 scene 转发的问题** |
| 5 | 看子进程 RSS | 起初**完全不动**（24s 内恒为 90812 kB） |

实验 5 挖出一个**独立的真 bug**：`rerun_sink` **从来不 flush**
（`grep -c flush` = 0）。服务器在本进程内时这是免费的——log 直接交给它；
一旦变成网络发送，SDK 会攒批。补上每 tick 一次 flush 后，子进程 RSS
开始增长（75588 → 77636 kB / 14s），**数据确实进了子进程**。

但 viewer 仍然空白。所以断点被夹到了最后一段：**子进程 → viewer**。

**关键证据是浏览器控制台里两条不同的代码路径**，同一个 URL、同一份 viewer：

```
进程内（能用）: re_grpc_client::read: Loading via gRPC… → Streaming messages
进程外（空白）: re_viewer_context::open_url: RedapProxy(...) → Web app started.
              re_auth::credentials::oauth: ... → 然后没有了
```

viewer 把 CLI 托管的 server 当成 **redap 端点**（走认证流程），而不是当成
它认识的那种 live proxy。SDK 内嵌的 server 和 CLI 的 server 不是一回事。

| 6 | 把 **SDK 自己的 `serve_grpc`** 放进子进程（参数和进程内那次完全一致），scene 端 `connect_grpc` | **也是空白** |

实验 6 否定了「CLI 和 SDK 的 server 实现不同」这个假设。所有失败情形的
共同点变成了一件事：**recording 在一个进程、server 在另一个进程**。能用的
那次两者在同一进程里。字节确实到达了 server（RSS 增长可证），但 viewer
取不到——所以问题在「经 `connect_grpc` 进来的数据如何对 viewer 可见」，
而不在传输本身。

### 更正（同日稍后）：空白与「进程外」无关

回退之后又测了一次，结论必须推翻前面那段：

**回退到进程内之后，viewer 一样空白。** 连续重载 4 次，iframe 里的 canvas
始终是 `300x150`（HTML 默认值，即从未开始绘制），控制台走的也是同一条
`RedapProxy → Web app started → 无后续` 的路径。而这份代码今晚早些时候是
渲染成功过的（截图里有点云和 rerun 顶栏）。

所以：

- **「空白」是一个独立的、与进程内外无关的故障**，我先前把它归因于移出
  进程是错的，**据此做出的回退也是基于错误判断**。
- **「进程外消除死锁」这个测量仍然成立**（5/5），它和空白问题互不相干。

已排除的原因（都实测过，不要再试）：

| 怀疑 | 怎么排除的 |
| --- | --- |
| scene 的 `/proxy` 转发 | 绕开直连，一样空白 |
| CLI server ≠ SDK server | SDK 的 `serve_grpc` 放子进程，一样空白 |
| 浏览器残留状态 | 清掉 localStorage/IndexedDB（界面语言确实被重置了），一样空白 |
| 发布循环没跑 | 日志有 `tick 300: 5 objects, 4 with points` |
| 数据没到 server | 进程外时子进程 RSS 持续增长 |

**所以现在真正未知的是：同一份代码，今晚早些时候渲染成功、之后一直空白，
中间变了什么。** 这个先查清楚，再谈进程内外。

**下一步该从哪开始**（不要再重复上面六个实验）：
- 查 0.37.1 里 `re_grpc_client::read` 与 `RedapProxy` 两条路径的分叉条件——
  大概率取决于 server 在首次调用时返回什么。
- ~~在子进程里跑 Python 的 `serve_grpc`~~ —— 已试（实验 6），同样空白。
- 真正该问的问题变成了：**一个通过 `connect_grpc` 送进 proxy 的 recording，
  viewer 要怎样才能订阅到它？** 怀疑方向是 recording id / store id 的可见性
  ——viewer 可能只订阅了 server 自己那条 recording，而不是转发进来的那条。
  可以用 `rerun rrd print <url>` 或第二个 SDK 客户端去读，确认到底存在几条
  recording、各自的 id 是什么。
- 需要读 0.37.1 的 `re_grpc_server` 源码，这不是黑盒能试出来的。

## 2. scene 自己的日志曾经是被销毁的（已修，留档）

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

## 3. 镜像 tag 被两个 worktree 抢

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
- 端口探测曾经比它守护的服务器还严（不带 SO_REUSEADDR 的 bind），
  一个开着的浏览器标签页就能让 viewer 整场不可用。已改为「问服务器会问的
  问题」，并对真正还没关完的 listener 给一个共享的短等待。
