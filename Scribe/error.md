
  # 2. 写入正确的 apt 源（通用 .deb 路径，不是 ubuntu24.04）
  curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

  # 3. 安装
  sudo apt-get update
  sudo apt-get install -y nvidia-container-toolkit

  # 4. 重启 Docker
  sudo systemctl restart docker

  # 5. 验证
  docker info 2>/dev/null | grep nvidia


-----------------------

    sudo apt-get install -y docker-buildx

  装完后 DOCKER_BUILDKIT=1 rbnx build -p system/scene 就能正常跑了。

-----------------------

 主机 rbnx codegen 生成的 _pb2.py 要求 protobuf 7.35.0
  - sim 容器只有 protobuf 6.33.6（太老，在 Dockerfile 里 pin 了 <7）

  chassis 能过是因为它不 import asr_pb2（语音相关的共享 proto）；camera 和 lidar 的 robonix_contracts_pb2_grpc 拖进了所有 contract proto，碰到 asr_pb2 就炸了。

  修复——升级容器内的 protobuf：

  docker exec robonix_tiago_sim pip install --upgrade "protobuf>=7.35.0"

  然后重新 rbnx boot。

--------------------------

  ▎ 根因：主机 rbnx codegen 生成的 proto 文件要求 protobuf ≥ 
  ▎ 7.35，但所有容器/venv 都 pin 了旧版 6.x。需要逐个升级。

  for d in /home/hyl/robonix/services/memory
  /home/hyl/robonix/services/memsearch /home/hyl/robonix/services/speech
  /home/hyl/robonix/services/voiceprint; do
      venv="$d/rbnx-build/venv/bin/pip"
      [ -f "$venv" ] && echo "=== $d ===" && "$venv" install --upgrade
  "protobuf>=7.35" 2>&1 | tail -1

