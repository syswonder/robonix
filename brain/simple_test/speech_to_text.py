import whisper
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wavfile
import tempfile
import os
import time
import subprocess
import asyncio
import edge_tts
from playsound import playsound
import os
import sys
from api.voice_tts import play_tts

# 2. 配置录音参数
fs = 16000  # 采样率 (Whisper 模型通常使用 16kHz)
duration = 15 # 录音时长 (秒)
channels = 1 # 声道数 (单声道即可)

# 1. 加载 Whisper 模型
# 确保 download_root 目录存在，并且有足够的空间下载模型
model_name = "large-v2"
download_root = '/mnt/DataDisk/MODELS'
filename = "temp/temp.wav"

# print(f"Loading Whisper model '{model_name}' from '{download_root}'...")
try:
    model = whisper.load_model(model_name, download_root=download_root)
    # print("Model loaded successfully.")
except Exception as e:
    print(f"Error loading model: {e}")
    print("Please check the model name and download_root path, and ensure you have internet access for the first download.")
    exit()

# 3. 录制音频
try:
    # 使用 sounddevice 录制音频
    # cmd = ["arecord", "-D", "plughw:2,0","-r", str(fs),"-d", str(duration), "-f", "cd", filename]
    # subprocess.run(cmd)
    play_tts("开始录音")
    audio_data = sd.rec(int(duration * fs), samplerate=fs, channels=channels, dtype='float32')
    sd.wait()  # 等待录音完成
    play_tts("录音完成")
    wavfile.write(filename, fs,audio_data)
    # print("Recording finished.")
except Exception as e:
    # print(f"Error during recording: {e}")
    # print("Please check your microphone setup and permissions.")
    exit()

try:
    start_time = time.time()
    # Whisper's transcribe method can directly accept a numpy array of the audio
    result = model.transcribe(filename, language="zh")
    end_time = time.time()
    transcribed_text = result["text"]
    # print("Transcription:")
    print(transcribed_text)
    # print(f"Transcription time: {end_time - start_time:.2f} seconds")
except Exception as e:
    print(f"Error during transcription: {e}")

