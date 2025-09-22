# Copyright 2024 the LlamaFactory team.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import random
import subprocess
from copy import deepcopy

from spatiallm.tuner import trainer
from spatiallm.tuner.framework import logging
from spatiallm.tuner.framework.utils import get_device_count, is_env_enabled
from spatiallm.tuner.trainer import run_exp  # use absolute import

logger = logging.get_logger(__name__)


def main():
    force_torchrun = os.getenv("FORCE_TORCHRUN", "0").lower() in ["true", "1"]
    if force_torchrun or get_device_count() > 1:
        master_addr = os.getenv("MASTER_ADDR", "127.0.0.1")
        master_port = os.getenv("MASTER_PORT", str(random.randint(20001, 29999)))
        logger.info_rank0(
            f"Initializing distributed tasks at: {master_addr}:{master_port}"
        )
        standalone = os.getenv("STANDALONE", "0").lower() in ["true", "1"]

        env = deepcopy(os.environ)
        if is_env_enabled("OPTIM_TORCH", "1"):
            # optimize DDP, see https://zhuanlan.zhihu.com/p/671834539
            env["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"
            env["TORCH_NCCL_AVOID_RECORD_STREAMS"] = "1"

        command = (
            "torchrun {standalone} --nnodes {nnodes} --node_rank {node_rank} --nproc_per_node {nproc_per_node} "
            "--master_addr {master_addr} --master_port {master_port} {file_name} {args}"
        ).format(
            standalone="--standalone" if standalone else "",
            nnodes=os.getenv("NNODES", "1"),
            node_rank=os.getenv("NODE_RANK", "0"),
            nproc_per_node=os.getenv("NPROC_PER_NODE", str(get_device_count())),
            master_addr=master_addr,
            master_port=master_port,
            file_name=trainer.__file__,
            args=" ".join(sys.argv[1:]),
            env=env,
            check=True,
        )
        logger.info_rank0(f"Running command: {command}")
        process = subprocess.run(command.split())
        sys.exit(process.returncode)
    else:
        run_exp()


if __name__ == "__main__":
    main()
