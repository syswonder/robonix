# Skill

本目录定义具身智能的具体行为技能，通常由多个 Capability 按事件流或状态机方式组合而成。

## 示例技能：

- `hello_spin_greet`: 原地旋转 -> 视觉检测人形 -> 播报 "Hello World"
- `pick_cup`: 定位杯子 -> 导航接近 -> 抓取动作

每个 Skill 是一个任务模板，可被 Brain 调度执行。
