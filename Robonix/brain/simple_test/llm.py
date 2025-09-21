import re
import os # 导入 os 模块用于文件操作
import urllib.request
import urllib.parse
import json

API_KEY = "0e987a80-e2fa-4539-83fd-3d6d2d863138"

BOT_MODEL_URL = "https://ark.cn-beijing.volces.com/api/v3/bots"
BOT_MODEL_ID = "bot-20250506223735-hv64s"

THINK_MODEL_URL = "https://ark.cn-beijing.volces.com/api/v3"
THINK_MODEL_ID = "doubao-1-5-thinking-pro-250415"

def extract_bash_code(text):
  """
  从文本中提取用 ```bash ``` 包裹的代码块。

  Args:
    text: 包含代码块的字符串。

  Returns:
    包含提取到的代码块的列表。如果没有找到，返回空列表。
  """
  pattern = r"```bash\n(.*?)```"
  matches = re.findall(pattern, text, re.DOTALL)
  return [matches[0]] if len(matches)>0 else ["echo hello"]

def save_to_sh_file(codes, filename="temp/tmp.sh"):
  """
  将提取的 bash 代码写入到指定的 .sh 文件中。

  Args:
    codes: 包含 bash 代码字符串的列表。
    filename: 要保存的文件名（默认为 tmp.sh）。
  """
  if not codes:
    print("没有提取到 bash 代码，不创建文件。")
    return

  try:
    # 使用 'w' 模式打开文件，如果文件不存在则创建，如果存在则清空内容
    with open(filename, "w") as f:
      # 添加 shebang 行，指定使用 bash 执行
      f.write("#!/bin/bash\n\n")
      print(f"正在将提取的 bash 代码写入到 {filename}...")
      for i, code in enumerate(codes):
        f.write(f"# --- 代码块 {i+1} ---\n")
        f.write(code)
        f.write("\n\n") # 在代码块之间添加空行

    # 赋予文件执行权限
    os.chmod(filename, 0o755) # 0o755 表示所有者读写执行，组和其他人读执行

    print(f"代码已成功写入到 {filename} 并已赋予执行权限。")

  except IOError as e:
    print(f"写入文件时发生错误: {e}")

def get_llm(prompt: str, message: str) -> str:
    base_url = THINK_MODEL_URL
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {API_KEY}"
    }
    data = {
        "model": THINK_MODEL_ID,
        "messages": [
            {"role": "system", "content": prompt},
            {"role": "user", "content": message}
        ],
        "temperature": 0
    }
    data = json.dumps(data).encode('utf-8')
    req = urllib.request.Request(base_url + "/chat/completions", data=data, headers=headers, method='POST')
    with urllib.request.urlopen(req) as response:
        result = json.loads(response.read().decode('utf-8'))
    return result["choices"][0]["message"]["content"]

if __name__=="__main__":
    with open("info.md", "r", encoding="utf-8") as f:
        info = f.read()
        with open("temp/cmd.txt") as cmd:
          llm_output = get_llm(info, cmd.read())
          print(llm_output)
          code = extract_bash_code(llm_output)
          print(code)
          save_to_sh_file(code)
        