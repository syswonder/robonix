# examples

wheatfox <wheatfox17@icloud.com>

## 如何运行 genesis flow demo

1. 先运行 `python simulator/genesis/robot1.py` 启动模拟器，等待出现渲染窗口
2. 运行 `python manager/eaios_decorators.py --config config/include_sim.yaml`，导出 `skill/__init__.py`
3. 运行 `python simulator/examples/demo1/simple_demo.py --mode auto` 或 `--mode manual`，其中 auto 是用yolo辅助生成实体图