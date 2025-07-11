import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import shutil
import edge_tts
import asyncio
import subprocess
from pydub import AudioSegment
from piper_msgs.srv import PlayText  # ✅ 引入自定义服务


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_output')
        # self.sub = self.create_subscription(String, 'tts_request', self.cb, 10)
        self.voice_name = "zh-CN-XiaoxiaoNeural"
        self.srv = self.create_service(PlayText, 'play_tts', self.handle_play_tts)
        self.get_logger().info("✅ TTS 服务节点已启动（支持同步/异步播报）")

    # def cb(self, msg: String):
    #     text = str(msg.data.strip())
    #     if text.find('[SYNC]') != -1:
    #         text = text[6:]
    #         is_sync = True
    #         self.get_logger().info(f"🔈 收到同步播报指令: {text}")
    #     else:
    #         is_sync = False
    #         self.get_logger().info(f"🔈 收到异步播报指令: {text}")

    #     self.edge_free_tts(text.split('，'), speed=1.0, voice_name=self.voice_name, save_path="./tishi.wav", sync=is_sync)

    def handle_play_tts(self, request, response):
        text = request.text.strip()
        sync = request.sync
        self.get_logger().info(f"🧾 来自服务的 {'同步' if sync else '异步'}播报请求: {text}")
        self.edge_free_tts(text.split('，'), speed=1.0, voice_name=self.voice_name, save_path="./tishi.wav", sync=sync)
        response.success = True
        return response


    async def amain(self, TEXT, VOICE, OUTPUT_FILE):
        communicate = edge_tts.Communicate(TEXT, VOICE)
        await communicate.save(OUTPUT_FILE)

    def edge_free_tts(self, chunks_list, speed, voice_name, save_path, sync=False):
        if len(chunks_list) > 1:
            chunk_audio_list = []
            if os.path.exists("./edge_tts_voice"):
                shutil.rmtree("./edge_tts_voice")
            os.mkdir("./edge_tts_voice")

            for k, i in enumerate(chunks_list, 1):
                OUTPUT_FILE = f"./edge_tts_voice/{k}.wav"
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self.amain(i, voice_name, OUTPUT_FILE))
                chunk_audio_list.append(OUTPUT_FILE)
                # play_audio(OUTPUT_FILE)  # 边生成边播放

            self.merge_audio_files(chunk_audio_list, save_path)
        else:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.amain(chunks_list[0], voice_name, save_path))

        # ✅ 判断是否同步播放
        if sync:
            # 播放完成再返回
            print("🔈 正在同步播放 TTS...")
            subprocess.run(["aplay", save_path])
            print("🔈 播放完毕！...")
        else:
            # 异步播放
            print("🔈 正在异步播放 TTS...")
            subprocess.Popen(["aplay", save_path])
            print("🔈 播放完毕！...")



    def merge_audio_files(self, audio_paths, output_path):
        merged_audio = AudioSegment.silent(duration=0)
        for audio_path in audio_paths:
            audio = AudioSegment.from_file(audio_path)
            merged_audio += audio
        merged_audio.export(output_path, format="wav")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TTSNode())
    rclpy.shutdown()
