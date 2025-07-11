import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
from piper_msgs.srv import PlayText
import torch
import sounddevice as sd
import scipy.io.wavfile as wav


import whisper
import torch
# -*- encoding:utf-8 -*-
import hashlib
import hmac
import base64
from socket import *
import json, time, threading
from websocket import create_connection
import websocket
from urllib.parse import quote
import logging


def process_data_str(data : str) -> list:
    # 使用正则匹配:seg_id的内容  {"seg_id":0, ...}
    seg_id_str = data.split("seg_id\":")[1].split(",")[0]
    # 数字 字符串->int字符
    seg_id = int(seg_id_str)
    # print(seg_id,type(seg_id))

    type_id_str = data.split("type\":\"")[1].split("\",")[0]
    # 数字 字符串->int字符
    type_id = int(type_id_str)
    # print(type_id)
    
    # 遍历多个 "w\":\"的内容，并拼接
    str_get = ""
    for i in range(1, len(data.split("\"w\":\""))):
        str_get += data.split("\"w\":\"")[i].split("\"")[0]

    # print(str_get)

    return {
        "seg_id" : seg_id,
        "words" : str_get,
        "type": type_id
    }


# reload(sys)
# sys.setdefaultencoding("utf8")
class Client():
    def __init__(self, app_id: str = "cb173188",
                   api_key: str = "e1e345135a66fb8142f2685d5f929ca9"):
        base_url = "ws://rtasr.xfyun.cn/v1/ws"
        ts = str(int(time.time()))
        tt = (app_id + ts).encode('utf-8')
        md5 = hashlib.md5()
        md5.update(tt)
        baseString = md5.hexdigest()
        baseString = bytes(baseString, encoding='utf-8')

        apiKey = api_key.encode('utf-8')
        signa = hmac.new(apiKey, baseString, hashlib.sha1).digest()
        signa = base64.b64encode(signa)
        signa = str(signa, 'utf-8')
        self.end_tag = "{\"end\": true}"

        self.ws = create_connection(base_url + "?appid=" + app_id + "&ts=" + ts + "&signa=" + quote(signa))
        self.trecv = threading.Thread(target=self.recv)
        self.trecv.start()
        self.result_list = []

    def send(self, file_path):
        file_object = open(file_path, 'rb')
        try:
            index = 1
            while True:
                chunk = file_object.read(1280)
                if not chunk:
                    break
                self.ws.send(chunk)

                index += 1
                time.sleep(0.04)
        finally:
            file_object.close()

        self.ws.send(bytes(self.end_tag.encode('utf-8')))
        print("send end tag success")

    def recv(self):
        try:
            while self.ws.connected:
                result = str(self.ws.recv())
                if len(result) == 0:
                    print("receive result end")
                    break
                result_dict = json.loads(result)
                # 解析结果
                if result_dict["action"] == "started":
                    print("handshake success, result: " + result)

                if result_dict["action"] == "result":
                    result_1 = result_dict
                    # result_2 = json.loads(result_1["cn"])
                    # result_3 = json.loads(result_2["st"])
                    # result_4 = json.loads(result_3["rt"])
                    # print("rtasr result: " + result_1["data"])
                    self.result_list.append(result_1["data"])
                    

                if result_dict["action"] == "error":
                    print("rtasr error: " + result)
                    self.ws.close()
                    return
        except websocket.WebSocketConnectionClosedException:
            print("receive result end")

    def close(self):
        self.ws.close()
        print("connection closed")



def asr_transcribe(file_path: str,
                   app_id: str = "cb173188",
                   api_key: str = "e1e345135a66fb8142f2685d5f929ca9") -> str:
    """
    使用指定的 Client 对音频文件进行识别，并返回最终识别的字符串。

    :param file_path: 音频文件路径
    :param app_id: 应用 ID（默认值为示例）
    :param api_key: API 密钥（默认值为示例）
    :return: 识别结果的字符串
    """
    logging.basicConfig()

    client = Client(app_id=app_id, api_key=api_key)
    client.send(file_path)

    fin_str = ""
    for line in client.result_list:
        result = process_data_str(line)
        if result["type"] == 0:
            fin_str += result["words"]

    return fin_str


KEYWORDS = ["你好", "开始", "激活"]
TEMP_AUDIO_FILE = "./temp_listen.wav"


# @TODO 需要后期加入声纹识别模块
class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.model = whisper.load_model("small", download_root='/mnt/DataDisk/MODELS')  # 可选 base / medium / large

        # ✅ 加载完之后，转到 GPU
        if torch.cuda.is_available():
            self.model = self.model.to('cuda')
            self.get_logger().info("✅ Whisper 模型已移动到 GPU 运行")
        else:
            self.get_logger().warn("⚠️ CUDA 不可用，Whisper 将在 CPU 上运行")
        self.get_logger().info("✅ Whisper 模型加载完成，准备监听语音指令")

        self.publisher = self.create_publisher(String, 'voice_command', 10)
        # self.tts_pub = self.create_publisher(String, '·', 10)
        self.client = self.create_client(PlayText, 'play_tts')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 TTS 服务...')
        # 启动循环监听
        self.request = PlayText.Request()
        self.loop()
        
    def listen(self):
        self.get_logger().info("🎙️ 正在录音 3 秒...")
        self.record_audio(TEMP_AUDIO_FILE, duration=3)
        text = self.transcribe_audio(TEMP_AUDIO_FILE).strip()
        return text
        
    def loop(self):
        while rclpy.ok():
            self.get_logger().info("🎙️ 正在录音 3 秒...")
            self.record_audio(TEMP_AUDIO_FILE, duration=3)
            text = self.transcribe_audio(TEMP_AUDIO_FILE).strip()
            if text:
            # if True:
                for keyword in KEYWORDS:
                    # if True:
                    if keyword in text:
                        print(f"🚀 关键词 '{keyword}' 触发！发布消息到topic")
                        text_command = ''
                        while not text_command:
                            start = time.time()

                            self.publlish_is_sync("你好，请您在我说完后发布指令，您有十秒时间", sync=True)
                            self.record_audio(TEMP_AUDIO_FILE, duration=10)
                            # 修改使用讯飞的asr api来做语音命令转换，这样可以加速很多，否则原来的whisper模型实在是太慢了。
                            text_command = asr_transcribe(TEMP_AUDIO_FILE).strip()
                            if text_command:
                                msg = String(data=text + '。' + text_command)
                                self.publisher.publish(msg)
                                self.get_logger().info(f"📤 发布语音指令: {text + '。' + text_command}")
                                self.publlish_is_sync("收到，正在思考中", sync=False)
                                print('经过了', time.time() - start, ' 秒')
                                break
                            # edge_free_tts(['收到', '正在思考中'], 1, 'zh-CN-XiaoxiaoNeural', './tishi.wav')
                    else:
                        print(f"\r未识别到关键词^_^", end='')
                        continue
            else:
                self.get_logger().warn("🈳 无识别结果")
            time.sleep(1)


    def publlish_is_sync(self, text='说话啊', sync=False):
        self.request.text = text
        self.request.sync = sync
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)



    # def publlish_is_sync(self, text='说话啊', sync=False):
    #     if sync:
    #         self.tts_pub.publish(String(data="[SYNC]" + text))
    #     else:
    #         self.tts_pub.publish(String(data=text))


    def record_audio(self, filename, duration=3):
        sample_rate=44100
        recording = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype='float64')
        sd.wait()
        wav.write(filename, sample_rate, recording)
        cmd = ["arecord", "-D", "plughw:2,0", "-d", str(duration), "-f", "cd", filename]
        subprocess.run(cmd)

    def transcribe_audio(self, filename):
        result = self.model.transcribe(filename, language="zh")
        return result["text"]


def main(args=None):    
    rclpy.init(args=args)
    try:
        rclpy.spin(WhisperNode())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()