# examples

wheatfox <wheatfox17@icloud.com>

## 如何运行 genesis flow demo

1. 先运行 `python simulator/genesis/robot1.py` 启动模拟器，等待出现渲染窗口

2. 运行 `python manager/eaios_decorators.py --config config/include_sim.yaml`，导出 `skill/__init__.py`

3. 下载并保存 yoloe 模型文件以供 `sim_vision` 相关 skill 使用：

```
cd skill/sim_vision/models
wget https://huggingface.co/jameslahm/yoloe/resolve/main/yoloe-11l-seg-pf.pt
```

4. 运行 `python simulator/examples/demo1/simple_demo.py --mode auto` 或 `--mode manual`，其中 auto 是用yolo辅助生成实体图

auto 模式下小车目前会自动识别物体，并自动绑定 flow 参数，flow 会让小车移动到识别到的物体位置。