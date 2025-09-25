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



if __name__ == '__main__':
    logging.basicConfig()

    app_id = "cb173188"
    api_key = "e1e345135a66fb8142f2685d5f929ca9"
    file_path = r"jiabin.wav"

    client = Client()
    client.send(file_path)

    result_list_proc = []
    for line in client.result_list:
        result_list_proc.append(process_data_str(line))
    fin_str = ""

    for result_proc in result_list_proc :
        if result_proc["type"] == 0:
            fin_str += result_proc["words"]

    print('aaaaa', fin_str)