# LLM任务规划Demo

## 简介

`llm_demo.py` 是一个基于DeepSeek LLM的机器人任务规划演示程序。它能够自动检测场景中的物体，理解用户的自然语言需求，生成相应的执行计划并自动执行。

## 主要功能

程序会自动使用YOLO检测场景中的物体并构建entity graph，然后接收用户的自然语言输入（如"让机器人走到椅子那边"），调用DeepSeek LLM生成相应的Python flow代码，最后自动执行这个flow完成任务。

## 安装依赖

```bash
pip install openai python-dotenv
```

## 使用方法

### 获取API Key
访问 [DeepSeek官网](https://platform.deepseek.com/) 获取API Key。

### 设置环境变量
推荐使用设置脚本：
```bash
cd simulator/examples/demo1
python setup_env.py
```

或者手动创建`.env`文件：
```bash
echo "DEEPSEEK_API_KEY=sk-your-api-key-here" > .env
```

### 运行程序

请在 deepembody 根目录下运行：

```bash
python simulator/examples/demo1/llm_demo.py
```

## 工作流程

程序启动后会自动检测场景并构建entity graph，然后等待用户输入需求。用户输入后，程序会调用LLM生成执行计划，显示任务描述和参数，询问是否执行。确认后程序会自动运行生成的flow代码。程序会在`llm_demo_debug/`目录下保存调试信息，包括LLM的完整响应和生成的flow代码。程序会将当前场景的entity graph结构、每个entity绑定的skill信息、flow语法示例等结构化信息传递给LLM。LLM基于这些信息分析用户需求，生成包含@flow装饰器的完整Python函数代码，并指定具体的参数值。生成的flow代码会被保存为临时文件，然后通过runtime加载执行。执行完成后自动清理临时文件。