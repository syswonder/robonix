# SceneVerifier

PR #242 (`a18cec68a6db871ba01874adbe08fd2eaf5123e0`) 的后续实现参考。
提供 `robonix/service/verifier/verify`，通过指定 Scene 的
`robonix/system/scene/get_robot_context` 验证平面导航结果。

## 文件职责

- `scene_verifier/config.py`：配置解析和有限数校验。
- `scene_verifier/core.py`：请求解析、目标解析、Scene 响应校验、空间判断。
- `scene_verifier/scene_client.py`：Atlas 精确连接、MCP 观察及响应解码。
- `scene_verifier/service.py`：生命周期、请求编排和启动入口。
- `tests/test_verifier.py`：核心逻辑、响应解析、请求编排测试。
- `scripts/build.sh`：依赖安装和生成 Verify 类型。
- `scripts/start.sh`：启动已构建的环境。

## 放置与构建

将整个目录放到 Robonix 的 `services/verifiers/scene_verifier/`。
在根 `pyproject.toml` 已有的 `[tool.uv.workspace].members` 数组增加
`"services/verifiers/*"`，保留原有成员。不替换整个根配置。
在仓库根运行：

```bash
uv lock
rbnx validate services/verifiers/scene_verifier
rbnx build -p services/verifiers/scene_verifier
```

`verifier_mcp` 是 `rbnx codegen --mcp` 的生成物，不手写、不单独下载。
Scene 返回值采用严格 JSON 解析，不需要导入 `semantic_map_mcp`。
测试不需要 Atlas、ROS 2 或 codegen：

```bash
cd services/verifiers/scene_verifier
python3 -m unittest discover -s tests -v
```

## Webots 部署

将下面内容合并到 `examples/webots/robonix_manifest.yaml` 已有的对应节点，
不要重复创建顶层 `system` 或 `service`：

```yaml
system:
  executor:
    verification:
      - target_contract_id: robonix/service/navigation/navigate
        target_provider_id: nav2
        verifier_provider_id: scene_verifier
        verifier_args:
          scene_provider_id: scene
          check_yaw: true

service:
  - name: scene_verifier
    path: ../../services/verifiers/scene_verifier
    config:
      distance_tolerance_m: 0.5
      yaw_tolerance_rad: 0.35
      observation_timeout_s: 5.0
```

通过现有 Webots build/boot 流程启动部署；上面的 package build 只构建验证器。
配置 `check_yaw: true` 要求合法的显式目标朝向，单位四元数表示 yaw=0，仍会验证。
配置 `false` 时完全跳过朝向检查。这是规则级约定，不能推断自然语言意图。
这些私有参数在本实现中定义，Executor 仅原样传递。

可选 `verifier_args.expected_map_id` 是非空字符串，用于固定地图部署。
不配置时，仅要求 Scene 地图身份非空，不声称检查了目标地图来源。

## 行为与边界

- 只支持 `goal.header.frame_id == "map"`、目标 z=0 的平面导航。
- 位置缺失、非有限数、无效四元数、非支持坐标系是请求错误。
- pose 未知、stale、地图不匹配、距离或角度超限返回 `passed=false`。
- 采用严格小于容差，等于容差也拒绝。
- MCP/Atlas 失败、响应损坏、参数损坏抛异常，由 Executor 归为 unavailable。
- 不根据 target_output 的 SUCCEEDED 或 accepted 推断成功。
- `map_id` 是地图身份，`frame_id` 是坐标系，两者不能直接比较。
- Scene 当前响应没有机器人 pose 的独立 frame_id，本实现依赖其 map-frame
  快照约定；不能检测 Scene 内部错误坐标系，也不能证明目标地图代次未改变。
- `stale` 使用 Scene 自己的判定，不拿宿主时钟与仿真时钟自行比较。
- `observation_timeout_s` 仅约束异步 MCP 观察；当前同步 Atlas SDK 调用没有
  在这里增加 deadline。Executor 自身仍有 60 秒验证期限，不能据此声称
  Verifier 内部 Atlas 调用也会在 5 秒内结束。
- 读取单次快照，不重试移动、不等待导航收敛、不订阅 ROS、不新增公共 contract。

## 验证状态

交付前已运行标准库 unittest 测试和 Python/Shell 语法检查。
没有在本环境运行 rbnx codegen/build、真实 Atlas/MCP 生命周期或 Webots 端到端；
部署测试仍需在具备 Robonix 工具链的开发环境完成。