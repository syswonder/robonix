import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty          # 触发器消息类型，可替换为你需要的类型
from std_srvs.srv import Empty as srcEmpty
import json
import cv2
from cv_bridge import CvBridge
import os
from datetime import datetime
from piper_vision import s3img
# 通过 pip install volcengine-python-sdk[ark] 安装方舟SDK
from volcenginesdkarkruntime import Ark
from typing import Dict, List
from piper_msgs.msg import AllObjectPos
import traceback 


# 替换 <Model> 为模型的Model ID
vlmmodel="doubao-1.5-vision-pro-32k-250115"


#  @TODO 设计一个服务，如果收到vlm识别的请求，就读取摄像头数据，和当前的摄像头给的目标点的坐标，然后返回当前的一些内容，并且将他们的坐标锚定到目标点坐标附近（or直接给目标点坐标

class VLMMapperNode(Node):
    def __init__(self):
        super().__init__('vlm_mapper')

        # ---------- 基础组件 ----------
        self.bridge = CvBridge()
        self.latest_img_path: str | None = None     # 存最近一帧图像完整路径
        self.latest_img_stamp: str | None = None    # 存图像时间戳字符串
        self.latest_data: Dict[str, Any] | None = None  # 存 parse_object_points 结果

        # ---------- 订阅 ----------
        self._image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self._points_sub = self.create_subscription(
            AllObjectPos, '/piper_vision/all_object_points',
            self.parse_object_points, 10
        )
        self._trigger_sub = self.create_subscription(
            Empty, '/detection_trigger', self.on_trigger, 10
        )

        self._map_capture_trigger = self.create_service(
            srcEmpty, '/piper_vision/map_capture', self.map_capture_trigger
        )

        # ---------- 其他 ----------
        if not os.getenv('ARK_API_KEY'):
            raise RuntimeError("请设置环境变量 ARK_API_KEY")
        self.vlmclient = Ark(api_key=os.getenv('ARK_API_KEY'))
        self.get_logger().info("📸 VLM 图像识别与坐标记录节点启动")

    def map_capture_trigger(self, request, response):
        self.get_logger().info("📸 触发构建语义地图，开始处理...")
        if not (self.latest_img_path and self.latest_data):
            self.get_logger().warn("⚠️ 触发时缺少最新图像或目标数据，忽略")
            return response
        vlm_result = self.call_doubao_rm_dup(self.latest_img_path)
        if vlm_result is None:
            self.get_logger().error("❌ VLM 识别失败")
            return response
        # 根据vlm_result，过滤objects中的东西
        self.get_logger().debug("过滤前： %s" % str(self.latest_data["objects"]))
        filtered_objects = [obj for obj in self.latest_data["objects"] if obj["name"] in vlm_result]
        self.get_logger().debug("过滤后： %s" % str(self.latest_data["objects"]))

        existing_data = {}

        os.makedirs("map", exist_ok=True)
        file_path = "map/map.json"
        if os.path.exists(file_path) and os.path.getsize(file_path) > 0:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    existing_data = json.load(f)
            except json.JSONDecodeError:
                print(f"Warning: '{file_path}' is not a valid JSON file or is empty. Starting with an empty map.")
                existing_data = {}

        for obj in filtered_objects:
            name = obj["name"]
            position = obj["position"]
            existing_data[name] = [position["x"], position["y"]]

        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(existing_data, f, indent=4, ensure_ascii=False)
            print(f"Successfully appended data to '{file_path}'.")
        except IOError as e:
            print(f"Error writing to file '{file_path}': {e}")
        return response

    # ------------------------------------------------------------------
    # ① 图像缓存：每到一帧就立即保存，但只保存最新一张
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        self.get_logger().debug("📸 收到新图像消息，开始处理...")
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        # img_path = f"images/{now}.jpg"
        os.makedirs("images", exist_ok=True)
        img_path = f"images/images_now.jpg"
        cv2.imwrite(img_path, cv_image)

        # 更新缓存
        self.latest_img_path = img_path
        self.latest_img_stamp = now
        self.get_logger().debug(f"✅ 新图像缓存于 {img_path}")

    # ------------------------------------------------------------------
    # ② 目标检测结果缓存（与之前相同，略有删减）
    # ------------------------------------------------------------------
    def parse_object_points(self, msg: AllObjectPos):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        objs = [
            {
                "name": name,
                "position": {"x": pt.x, "y": pt.y, "z": pt.z},
                "size": {"width": msg.widths[i], "height": msg.heights[i]},
            }
            for i, (name, pt) in enumerate(zip(msg.names, msg.points))
        ]
        self.latest_data = {
            "time": datetime.fromtimestamp(t).isoformat(timespec="milliseconds"),
            "frame": msg.header.frame_id,
            "objects": objs,
        }

    # ------------------------------------------------------------------
    # ③ 触发：拿最近一帧做 VLM + 解析目标检测 + 存储
    # ------------------------------------------------------------------
    def on_trigger(self, _):
        if not (self.latest_img_path and self.latest_data):
            self.get_logger().warn("⚠️ 触发时缺少最新图像或目标数据，忽略")
            return

        # 1) 调用 VLM 识别
        vlm_result = self.call_doubao_rm_dup(self.latest_img_path)
        if vlm_result is None:
            self.get_logger().error("❌ VLM 识别失败")
            return
        # 根据vlm_result，过滤objects中的东西
        self.get_logger().debug("过滤前： %s" % str(self.latest_data["objects"]))
        filtered_objects = [obj for obj in self.latest_data["objects"] if obj["name"] in vlm_result]
        self.latest_data["objects"] = filtered_objects
        self.get_logger().debug("过滤后： %s" % str(self.latest_data["objects"]))

        # 2) 合并两路结果
        record = {
            "img_path": self.latest_img_path,
            "img_stamp": self.latest_img_stamp,
            "vlm_result": vlm_result,
            "objects": self.latest_data,
        }

        # 3) 写文件（JSON 追加）
        os.makedirs("records", exist_ok=True)
        out_path = f"records/{self.latest_img_stamp}.json"
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(record, f, ensure_ascii=False, indent=2)

        self.get_logger().info(f"📝 记录写入 {out_path}")

    # ------------------------------------------------------------------
    # ④ 封装上传 / 调用大模型
    # ------------------------------------------------------------------
    def call_doubao(self, img_path: str) -> list | None:
        try:
            # 假设 s3img.upload_file(img_path) 返回公网 URL
            img_url = s3img.upload_file(img_path)
            resp = self.vlmclient.chat.completions.create(
                model=vlmmodel,
                messages=[
                    {"role": "user", "content": [
                        {"type": "text",
                         "text": "你是一个智能楼宇测绘员，请提炼这张照片中的物品、门牌号、以及所有有价值值得存到语义地图里的信息，以 JSON list 返回"},
                        {"type": "image_url", "image_url": {"url": img_url}},
                    ]}
                ],
            )
            return json.loads(resp.choices[0].message.content)
        except Exception as e:
            self.get_logger().error(f"doubao 调用失败: {e}")
            return None
        
    def call_doubao_rm_dup(self, img_path: str) -> list | None:
        list_objects = [obj["name"] for obj in self.latest_data["objects"]]
        self.get_logger().info("prompt: %s" % list_objects)
        # 去重
        try:
            # 假设 s3img.upload_file(img_path) 返回公网 URL
            img_url = s3img.upload_file(file_name = img_path)
            resp = self.vlmclient.chat.completions.create(
                model=vlmmodel,
                messages=[
                    {"role": "user", "content": [
                        {"type": "text",
                         "text": "请你查看这张图片中是否包含以下物品，包含的物品请按原名称直接以 JSON list 返回, 不要有其他文字说明: " + str([obj["name"] for obj in self.latest_data["objects"]])},
                        {"type": "image_url", "image_url": {"url": img_url}},
                    ]}
                ],
            )
            self.get_logger().info("resp: %s " % resp.choices[0].message.content)
            return json.loads(resp.choices[0].message.content)
        except Exception as e:
            error_msg = traceback.format_exc()  # This will include file name, line number, and call stack
            self.get_logger().error(f"doubao 调用失败: {e}")
            return None



def main(args=None):
    rclpy.init(args=args)
    node = VLMMapperNode()
    rclpy.spin(node) # 回调函数默认顺序执行
    rclpy.shutdown()
