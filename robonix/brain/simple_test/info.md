# 任务

请帮我用下面提供的命令写一个bash脚本，来控制机器人完成任务。
提示：任务最后你总是要回到办公室

## 你可以使用以下命令

python api/setgoal.py x y rotation

python api/voice_tts.py.py text

## 位置信息如下

303办公室: 3.2, -1.3, 0
电梯口：0.3, -10.9, 0
餐厅: -8.5, -13.6, 0
办公桌 3.2 1.6 0
沙发 2.5 -1.9 0
301门口 -6.3 -4.0 0
梅老师办公室 3.2 1.6 0
303门口 4.0 -4.0 0
自动售货机 -11.7 -6.8 0
厕所 8.9 -4.8 0
休息区 17.0 -6.7 0
315办公室门口 29.0 -5.1 0
315曹老师办公室 28.3 0.1 0

## 示例1

### 任务要求

先回到办公室，然后去电梯口，最后回到办公室

### 输出示例

```bash
python api/voice_tts.py 开始执行任务
python api/setgoal.py 3.2 -1.3 0
python api/voice_tts.py 已到达办公室
python api/voice_tts.py 准备前往电梯
python api/setgoal.py 0.3 -10.9 0
python api/voice_tts.py 已到达电梯口
python api/voice_tts.py 准备返回办公室
python api/setgoal.py 3.2 -1.3 0
python api/voice_tts.py 任务完成
```

## 示例2

### 任务要求

先回到办公室，然后去餐厅, 最后回到办公室

### 输出示例

```bash
python api/voice_tts.py 开始执行任务
python api/setgoal.py 3.2 -1.3 0
python api/voice_tts.py 已到达办公室
python api/voice_tts.py 准备前往餐厅
python api/setgoal.py -8.5 -13.6 0
python api/voice_tts.py 已到达餐厅
python api/voice_tts.py 准备返回办公室
python api/setgoal.py 3.2 -1.3 0
python api/voice_tts.py 任务完成
```