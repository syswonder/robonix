import os
import s3img
# 通过 pip install volcengine-python-sdk[ark] 安装方舟SDK
from volcenginesdkarkruntime import Ark

# 替换 <Model> 为模型的Model ID
vlmmodel="doubao-1.5-vision-pro-32k-250115"

# 初始化Ark客户端，从环境变量中读取您的API Key
vlmclient = Ark(
    api_key=os.getenv('ARK_API_KEY'),
    )

# # 创建一个对话请求
# response = client.chat.completions.create(
#     # 指定您部署了视觉理解大模型的推理接入点ID
#     model = model,
#     messages = [
#         {
#             # 指定消息的角色为用户
#             "role": "user",
#             "content": [
#                 # 文本消息，希望模型根据图片信息回答的问题
#                 {"type": "text", "text": "你是一个室内地图测绘员，请提炼出这张照片中的物品、门牌号、以及所有有价值的信息。以list的形式返回给我"},
#                 # 图片信息，希望模型理解的图片
#                 {"type": "image_url", "image_url": {"url":  f"{s3img.upload_file()}"}
#                 },
#             ],
#         }
#     ],
# )
#
# print(response.choices[0].message.content)