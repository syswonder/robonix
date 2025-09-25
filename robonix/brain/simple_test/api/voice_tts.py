import asyncio
import edge_tts
from playsound import playsound
import os
import sys

async def play_chinese_tts(text, voice="zh-CN-XiaoyiNeural", output_filename="output.mp3"):
    """
    使用 edge-tts 生成中文语音并播放
    :param text: 要转换为语音的中文文本
    :param voice: 使用的语音模型 (例如: zh-CN-XiaoyiNeural, zh-CN-XiaohanNeural 等)
    :param output_filename: 生成的音频文件名
    """
    communicate = edge_tts.Communicate(text, voice)
    await communicate.save(output_filename)

    print(f"生成音频文件: {output_filename}")

    try:
        playsound(output_filename)
        print("播放完成")
    except Exception as e:
        print(f"播放音频时发生错误: {e}")
    finally:
        # 可以选择删除生成的音频文件
        # os.remove(output_filename)
        pass

def play_tts(txt:str):
    chinese_text = "你好，这是一个中文语音合成的测试。Hello, this is a chinese tts"
    # 你可以在这里尝试不同的中文语音模型
    voice_model = "zh-CN-XiaoyiNeural" # 女声
    # voice_model = "zh-CN-XiaohanNeural" # 男声
    # voice_model = "zh-CN-YunxiNeural"  # 男声
    # voice_model = "zh-CN-YunzeNeural"  # 男声

    # 创建保存音频的目录 (如果不存在)
    output_dir = "temp"
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "test_chinese.mp3")

    asyncio.run(play_chinese_tts(txt, voice=voice_model, output_filename=output_file))
    
if __name__ == "__main__":
    play_tts(str(sys.argv[1]))
