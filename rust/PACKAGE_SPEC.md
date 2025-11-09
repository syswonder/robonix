# Robonix Package Specification

## 概述

本文档定义了 robonix provider package 的规范。一个 package 可以提供多个 capabilities (cap) 和 skills (skl)。

**重要**: Robonix 不关心开发者如何实现 capabilities 和 skills。开发者可以使用任何技术栈（Python、C++、Rust、ROS2等）。Robonix 只要求 package 提供标准的 manifest 文件和必要的控制脚本，并提供 ROS2 通信接口。

## Package 目录结构

一个标准的 robonix provider package 应包含以下结构：

```
provider_package/
├── rbnx_manifest.yaml    # Robonix package manifest (必需)
├── rbnx/                 # Robonix 配置目录 (必需)
│   ├── start             # 启动脚本 (必需)
│   ├── stop              # 停止脚本 (必需)
│   ├── entry             # 入口脚本 (可选)
│   └── config.yaml       # 配置文件 (可选)
└── [source code]         # 源代码 (实现相关，任意技术栈)
```

## 必需配置文件

### 1. rbnx_manifest.yaml

这是 robonix package 的核心配置文件，定义了 package 提供的所有 capabilities 和 skills。

#### 格式规范

```yaml
package:
  name: string              # Package 名称
  version: string           # 版本号 (语义化版本)
  description: string       # 描述
  maintainer: string        # 维护者
  maintainer_email: string  # 维护者邮箱
  license: string           # 许可证

capabilities:
  - name: string            # 标准能力名称 (如 cap::grasp.move)
    inputs:                 # 输入参数通道映射 (字典格式: {参数名: topic通道})
      parameter_name: topic_channel
    outputs:                # 输出参数通道映射 (字典格式: {参数名: topic通道})
      parameter_name: topic_channel
    configs: {}             # 配置服务映射 (字典格式，通常为空)

skills:
  - name: string            # 标准技能名称 (如 skl::pick)
    inputs:                 # 输入参数通道映射 (字典格式: {参数名: topic通道})
      parameter_name: topic_channel
    outputs:                # 输出参数通道映射 (字典格式: {参数名: topic通道})
      parameter_name: topic_channel
    configs: {}             # 配置服务映射 (字典格式，通常为空)
```

**重要说明：**
- `name` 和 `type` 由标准规范 (spec) 定义，manifest 中无需重复指定
- `code_path`、`package_id`、`description` 等字段由 CLI 在注册时自动填充
- 只需提供参数名到通道的映射关系，格式为字典：`{参数名: topic通道}`
- 参数名必须与 spec 中定义的标准参数名完全一致

#### 字段说明

**Package 字段：**
- `name`: Package 名称
- `version`: 语义化版本号 (如 1.0.0)
- `description`: Package 功能描述
- `maintainer`: 维护者名称
- `maintainer_email`: 维护者邮箱
- `license`: 许可证类型 (如 Apache-2.0)

**Capability 字段：**
- `name`: 标准能力名称，必须以 `cap::` 开头，格式为 `cap::category.action`。名称和参数定义由标准规范 (spec) 定义
- `inputs`: 输入参数的通道映射，字典格式 `{参数名: topic通道}`。参数名必须与 spec 中定义的标准参数名一致
- `outputs`: 输出参数的通道映射，字典格式 `{参数名: topic通道}`。参数名必须与 spec 中定义的标准参数名一致
- `configs`: 配置服务的映射，字典格式（通常为空）

**Skill 字段：**
- `name`: 标准技能名称，必须以 `skl::` 开头。名称和参数定义由标准规范 (spec) 定义
- `inputs`: 输入参数的通道映射，字典格式 `{参数名: topic通道}`。参数名必须与 spec 中定义的标准参数名一致
- `outputs`: 输出参数的通道映射，字典格式 `{参数名: topic通道}`。参数名必须与 spec 中定义的标准参数名一致
- `configs`: 配置服务的映射，字典格式（通常为空）

**自动填充字段（无需在 manifest 中指定）：**
- `package_id`: 由 CLI 根据 package 名称自动生成
- `code_path`: 由 CLI 自动设置为 package 的安装路径
- `description`: 从 spec 中自动获取
- `type`: 从 spec 中自动获取参数类型

**示例：**

```yaml
capabilities:
  - name: cap::vision.capture_rgb
    # Spec 定义: OUTPUT: ["image" => "sensor_msgs/msg/Image"]
    outputs:
      image: /demo_rgb/image

  - name: cap::grasp.move
    # Spec 定义:
    #   INPUT: ["target_pose" => "geometry_msgs/msg/PoseStamped"]
    #   OUTPUT: ["status" => "boolean"]
    inputs:
      target_pose: /demo_grasp/pose_goal
    outputs:
      status: /demo_grasp/pose_status

skills:
  - name: skl::pick
    # Spec 定义:
    #   INPUT: ["target_label" => "string"]
    #   OUTPUT: ["status" => "boolean"]
    inputs:
      target_label: /demo_pick/target_label
    outputs:
      status: /demo_pick/status
```

### 2. rbnx/ 目录

`rbnx/` 目录包含所有 robonix 特定的配置和脚本。

#### rbnx/start (必需)

启动脚本，用于启动 package。Robonix 会在需要时调用此脚本。

脚本要求：
- 必须是可执行文件 (`chmod +x`)
- 应该启动 package 提供的所有 capabilities/skills
- 应该将进程 PID 保存到文件，供 stop 脚本使用
- 应该处理错误情况并返回适当的退出码

#### rbnx/stop (必需)

停止脚本，用于停止 package。Robonix 会在需要时调用此脚本。

脚本要求：
- 必须是可执行文件 (`chmod +x`)
- 应该停止所有由 start 脚本启动的进程
- 应该清理临时文件和资源

#### rbnx/entry (可选)

入口脚本，用于初始化 package。可以在启动前执行一些初始化任务。

#### rbnx/config.yaml (可选)

Package 特定的配置文件，可以包含：
- 日志配置
- 资源限制
- 环境变量
- 其他 package 特定的设置

## 实现独立性

Robonix **不要求**以下内容：

- ❌ ROS2 package.xml
- ❌ 特定的构建系统 (CMake, setuptools, Cargo 等)
- ❌ 特定的编程语言
- ❌ ROS2 依赖（除非 capability/skill 本身需要 ROS2）

开发者可以：
- ✅ 使用任何编程语言实现 capabilities 和 skills
- ✅ 使用任何构建系统或打包方式
- ✅ 使用 ROS2 或非 ROS2 的通信机制（只要符合 channel 规范）
- ✅ 自由组织源代码目录结构

## 标准规范验证

所有 capabilities 和 skills 必须符合 `robonix-core` 中定义的标准规范。系统会在注册时进行验证：

1. 能力/技能名称必须匹配标准规范
2. 输入/输出参数名称必须与规范完全一致
3. 参数类型由 spec 自动提供，无需在 manifest 中指定
4. 配置服务必须与规范完全一致

CLI 在注册时会自动：
- 从 spec 中获取参数的类型信息
- 验证 manifest 中的参数名是否与 spec 定义一致
- 自动填充 `code_path`、`package_id`、`description` 等字段

## 注册流程

Package 在安装后，需要通过 ROS2 service `/rbnx/srv/register` 注册其提供的 capabilities 和 skills。每个 capability 和 skill 需要单独注册。

注册请求格式参见 `robonix-core/srv/Register.srv`。

## 示例

完整示例请参考 `demo_provider/` 目录