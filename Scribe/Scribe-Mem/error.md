(base) hyl@lenovo-ThinkStation-PX:~/robonix/examples/webots$ rbnx build
[Building] packages declared in /home/hyl/robonix/examples/webots/robonix_manifest.yaml

[   0.000] :: checking remote providers for updates (skip: --no-update-check) ::
[   1.910] [ OK ]  mapping             up to date
[   3.225] [ OK ]  nav2                up to date
[   4.823] [ OK ]  explore             up to date
    3 remote provider(s) up to date — nothing to update
  -> primitive tiago_chassis
[Building] com.robonix.example.tiago_chassis via manifest.build
[tiago_chassis/build] rbnx codegen --mcp --ros2
[codegen] package: /home/hyl/robonix/examples/webots/primitives/tiago_chassis
[codegen] robonix source: /home/hyl/robonix
[codegen] robonix-codegen --lang proto ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] proto: 37 packages, 177 msgs, 82 srv -> /home/hyl/robonix/examples/webots/primitives/tiago_chassis/rbnx-build/proto-staging
[robonix-codegen] contracts: robonix_contracts.proto + contract_proto_modules.rs (under /home/hyl/robonix/examples/webots/primitives/tiago_chassis/rbnx-build/proto-staging)
[codegen] robonix-codegen --lang mcp ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] mcp: 37 packages, 177 msgs -> /home/hyl/robonix/examples/webots/primitives/tiago_chassis/rbnx-build/codegen/robonix_mcp_types
[codegen] grpc_tools.protoc → /home/hyl/robonix/examples/webots/primitives/tiago_chassis/rbnx-build/codegen/proto_gen
[codegen] robonix-codegen --lang ros2 ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] ros2: 37 packages, 177 msg, 82 srv -> /home/hyl/robonix/examples/webots/primitives/tiago_chassis/rbnx-build/codegen/ros2_idl/src
[codegen] done — proto+mcp+stubs, mcp_types,  setup.bash
[tiago_chassis/build] colcon build ros2_idl in sim container
Starting >>> builtin_interfaces
Starting >>> audio
Starting >>> pilot
Starting >>> lifecycle
Starting >>> lifecycle_msgs
Starting >>> map
Starting >>> semantic_map
Starting >>> std_srvs
Starting >>> voiceprint
Finished <<< lifecycle [1.38s]
Finished <<< builtin_interfaces [1.42s]
Finished <<< voiceprint [1.40s]
Finished <<< std_srvs [1.42s]
Starting >>> std_msgs
Starting >>> rcl_interfaces
Starting >>> rosgraph_msgs
Starting >>> statistics_msgs
Starting >>> test_msgs
Finished <<< lifecycle_msgs [1.46s]
Finished <<< semantic_map [1.50s]
Finished <<< audio [1.55s]
Finished <<< pilot [1.56s]
Starting >>> asr
Starting >>> executor
Starting >>> liaison
Starting >>> speech
Starting >>> tts
Finished <<< map [1.59s]
Finished <<< rosgraph_msgs [1.46s]
Finished <<< test_msgs [1.48s]
Finished <<< statistics_msgs [1.51s]
Finished <<< liaison [1.50s]
Finished <<< asr [1.51s]
Finished <<< rcl_interfaces [1.69s]
Finished <<< tts [1.56s]
Finished <<< executor [1.59s]
Starting >>> composition_interfaces
Finished <<< speech [1.61s]
Finished <<< std_msgs [1.76s]
Starting >>> geometry_msgs
Starting >>> actionlib_msgs
Starting >>> chassis
Starting >>> diagnostic_msgs
Starting >>> memgraph
Starting >>> memory
Finished <<< composition_interfaces [1.52s]
Finished <<< actionlib_msgs [1.51s]
Finished <<< chassis [1.53s]
Finished <<< memgraph [1.59s]
Finished <<< memory [1.61s]
Finished <<< diagnostic_msgs [1.63s]
Finished <<< geometry_msgs [1.75s]
Starting >>> sensor_msgs
Starting >>> nav_msgs
Starting >>> navigation
Starting >>> shape_msgs
Starting >>> soma
Starting >>> trajectory_msgs
Finished <<< trajectory_msgs [1.63s]
Finished <<< shape_msgs [1.67s]
Finished <<< navigation [1.70s]
Finished <<< soma [1.71s]
Finished <<< nav_msgs [1.74s]
Finished <<< sensor_msgs [1.91s]
Starting >>> camera
Starting >>> lidar
Starting >>> perception
Starting >>> stereo_msgs
Starting >>> visualization_msgs
Finished <<< stereo_msgs [1.76s]
Finished <<< perception [1.79s]
Finished <<< lidar [1.81s]
Finished <<< camera [1.83s]
Finished <<< visualization_msgs [1.86s]

Summary: 37 packages finished [8.96s]
[tiago_chassis/build] done.
✓ Package 'com.robonix.example.tiago_chassis' build finished
  -> primitive tiago_camera
[Building] com.robonix.example.tiago_camera via manifest.build
[tiago_camera/build] rbnx codegen --mcp --ros2
[codegen] package: /home/hyl/robonix/examples/webots/primitives/tiago_camera
[codegen] robonix source: /home/hyl/robonix
[codegen] robonix-codegen --lang proto ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] proto: 37 packages, 177 msgs, 82 srv -> /home/hyl/robonix/examples/webots/primitives/tiago_camera/rbnx-build/proto-staging
[robonix-codegen] contracts: robonix_contracts.proto + contract_proto_modules.rs (under /home/hyl/robonix/examples/webots/primitives/tiago_camera/rbnx-build/proto-staging)
[codegen] robonix-codegen --lang mcp ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] mcp: 37 packages, 177 msgs -> /home/hyl/robonix/examples/webots/primitives/tiago_camera/rbnx-build/codegen/robonix_mcp_types
[codegen] grpc_tools.protoc → /home/hyl/robonix/examples/webots/primitives/tiago_camera/rbnx-build/codegen/proto_gen
[codegen] robonix-codegen --lang ros2 ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] ros2: 37 packages, 177 msg, 82 srv -> /home/hyl/robonix/examples/webots/primitives/tiago_camera/rbnx-build/codegen/ros2_idl/src
[codegen] done — proto+mcp+stubs, mcp_types,  setup.bash
Starting >>> builtin_interfaces
Starting >>> audio
Starting >>> pilot
Starting >>> lifecycle
Starting >>> lifecycle_msgs
Starting >>> map
Starting >>> semantic_map
Starting >>> std_srvs
Starting >>> voiceprint
Finished <<< builtin_interfaces [1.39s]
Starting >>> std_msgs
Starting >>> rcl_interfaces
Starting >>> rosgraph_msgs
Starting >>> statistics_msgs
Starting >>> test_msgs
Finished <<< lifecycle [1.44s]
Finished <<< std_srvs [1.44s]
Finished <<< audio [1.48s]
Starting >>> asr
Starting >>> speech
Starting >>> tts
Finished <<< semantic_map [1.49s]
Finished <<< voiceprint [1.50s]
Finished <<< lifecycle_msgs [1.52s]
Finished <<< map [1.58s]
Finished <<< pilot [1.61s]
Starting >>> executor
Starting >>> liaison
Finished <<< rosgraph_msgs [1.56s]
Finished <<< test_msgs [1.57s]
Finished <<< statistics_msgs [1.59s]
Finished <<< asr [1.58s]
Finished <<< tts [1.60s]
Finished <<< rcl_interfaces [1.73s]
Finished <<< liaison [1.52s]
Starting >>> composition_interfaces
Finished <<< executor [1.57s]
Finished <<< speech [1.71s]
Finished <<< std_msgs [1.83s]
Starting >>> geometry_msgs
Starting >>> actionlib_msgs
Starting >>> chassis
Starting >>> diagnostic_msgs
Starting >>> memgraph
Starting >>> memory
Finished <<< composition_interfaces [1.51s]
Finished <<< memory [1.48s]
Finished <<< chassis [1.52s]
Finished <<< memgraph [1.53s]
Finished <<< actionlib_msgs [1.56s]
Finished <<< diagnostic_msgs [1.62s]
Finished <<< geometry_msgs [1.74s]
Starting >>> sensor_msgs
Starting >>> nav_msgs
Starting >>> navigation
Starting >>> shape_msgs
Starting >>> soma
Starting >>> trajectory_msgs
Finished <<< shape_msgs [1.64s]
Finished <<< soma [1.68s]
Finished <<< nav_msgs [1.71s]
Finished <<< navigation [1.72s]
Finished <<< trajectory_msgs [1.73s]
Finished <<< sensor_msgs [1.84s]
Starting >>> camera
Starting >>> lidar
Starting >>> perception
Starting >>> stereo_msgs
Starting >>> visualization_msgs
Finished <<< lidar [1.73s]
Finished <<< perception [1.74s]
Finished <<< stereo_msgs [1.75s]
Finished <<< camera [1.81s]
Finished <<< visualization_msgs [1.83s]

Summary: 37 packages finished [8.87s]
[tiago_camera/build] done.
✓ Package 'com.robonix.example.tiago_camera' build finished
  -> primitive tiago_lidar
[Building] com.robonix.example.tiago_lidar via manifest.build
[tiago_lidar/build] rbnx codegen --mcp --ros2
[codegen] package: /home/hyl/robonix/examples/webots/primitives/tiago_lidar
[codegen] robonix source: /home/hyl/robonix
[codegen] robonix-codegen --lang proto ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] proto: 37 packages, 177 msgs, 82 srv -> /home/hyl/robonix/examples/webots/primitives/tiago_lidar/rbnx-build/proto-staging
[robonix-codegen] contracts: robonix_contracts.proto + contract_proto_modules.rs (under /home/hyl/robonix/examples/webots/primitives/tiago_lidar/rbnx-build/proto-staging)
[codegen] robonix-codegen --lang mcp ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] mcp: 37 packages, 177 msgs -> /home/hyl/robonix/examples/webots/primitives/tiago_lidar/rbnx-build/codegen/robonix_mcp_types
[codegen] grpc_tools.protoc → /home/hyl/robonix/examples/webots/primitives/tiago_lidar/rbnx-build/codegen/proto_gen
[codegen] robonix-codegen --lang ros2 ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] ros2: 37 packages, 177 msg, 82 srv -> /home/hyl/robonix/examples/webots/primitives/tiago_lidar/rbnx-build/codegen/ros2_idl/src
[codegen] done — proto+mcp+stubs, mcp_types,  setup.bash
Starting >>> builtin_interfaces
Starting >>> audio
Starting >>> pilot
Starting >>> lifecycle
Starting >>> lifecycle_msgs
Starting >>> map
Starting >>> semantic_map
Starting >>> std_srvs
Starting >>> voiceprint
Finished <<< builtin_interfaces [1.43s]
Finished <<< std_srvs [1.42s]
Starting >>> std_msgs
Starting >>> rcl_interfaces
Starting >>> rosgraph_msgs
Starting >>> statistics_msgs
Starting >>> test_msgs
Finished <<< voiceprint [1.47s]
Finished <<< lifecycle [1.50s]
Finished <<< map [1.51s]
Finished <<< semantic_map [1.53s]
Finished <<< pilot [1.60s]
Starting >>> executor
Starting >>> liaison
Finished <<< audio [1.64s]
Finished <<< lifecycle_msgs [1.64s]
Starting >>> asr
Starting >>> speech
Starting >>> tts
Finished <<< rosgraph_msgs [1.50s]
Finished <<< test_msgs [1.53s]
Finished <<< statistics_msgs [1.55s]
Finished <<< rcl_interfaces [1.65s]
Starting >>> composition_interfaces
Finished <<< asr [1.52s]
Finished <<< tts [1.58s]
Finished <<< liaison [1.64s]
Finished <<< speech [1.61s]
Finished <<< executor [1.67s]
Finished <<< std_msgs [1.86s]
Starting >>> geometry_msgs
Starting >>> actionlib_msgs
Starting >>> chassis
Starting >>> diagnostic_msgs
Starting >>> memgraph
Starting >>> memory
Finished <<< composition_interfaces [1.49s]
Finished <<< chassis [1.51s]
Finished <<< memgraph [1.52s]
Finished <<< actionlib_msgs [1.55s]
Finished <<< memory [1.55s]
Finished <<< diagnostic_msgs [1.61s]
Finished <<< geometry_msgs [1.69s]
Starting >>> sensor_msgs
Starting >>> nav_msgs
Starting >>> navigation
Starting >>> shape_msgs
Starting >>> soma
Starting >>> trajectory_msgs
Finished <<< navigation [1.62s]
Finished <<< shape_msgs [1.64s]
Finished <<< trajectory_msgs [1.66s]
Finished <<< soma [1.71s]
Finished <<< nav_msgs [1.75s]
Finished <<< sensor_msgs [1.79s]
Starting >>> camera
Starting >>> lidar
Starting >>> perception
Starting >>> stereo_msgs
Starting >>> visualization_msgs
Finished <<< stereo_msgs [1.77s]
Finished <<< lidar [1.81s]
Finished <<< perception [1.89s]
Finished <<< visualization_msgs [1.92s]
Finished <<< camera [1.94s]

Summary: 37 packages finished [8.95s]
[tiago_lidar/build] done.
✓ Package 'com.robonix.example.tiago_lidar' build finished
  -> primitive audio_driver
[Building] com.robonix.example.audio_driver via manifest.build
[codegen] package: /home/hyl/robonix/examples/webots/primitives/audio_driver
[codegen] robonix source: /home/hyl/robonix
[codegen] robonix-codegen --lang proto ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] proto: 37 packages, 177 msgs, 82 srv -> /home/hyl/robonix/examples/webots/primitives/audio_driver/rbnx-build/proto-staging
[robonix-codegen] contracts: robonix_contracts.proto + contract_proto_modules.rs (under /home/hyl/robonix/examples/webots/primitives/audio_driver/rbnx-build/proto-staging)
[codegen] grpc_tools.protoc → /home/hyl/robonix/examples/webots/primitives/audio_driver/rbnx-build/codegen/proto_gen
[codegen] done — proto+stubs,  setup.bash
[build] done.
✓ Package 'com.robonix.example.audio_driver' build finished
  -> service memgraph
[Building] com.robonix.example.memgraph_service via manifest.build
[build] uv → /home/hyl/miniconda3/envs/env_robonix/bin/uv (uv 0.11.21 (x86_64-unknown-linux-gnu))
[build] uv sync (pyproject.toml → rbnx-build/venv)
Resolved 233 packages in 1ms
Checked 76 packages in 0.54ms
[build] rbnx codegen --mcp
[codegen] package: /home/hyl/robonix/services/memory
[codegen] robonix source: /home/hyl/robonix
[codegen] robonix-codegen --lang proto ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] proto: 37 packages, 177 msgs, 82 srv -> /home/hyl/robonix/services/memory/rbnx-build/proto-staging
[robonix-codegen] contracts: robonix_contracts.proto + contract_proto_modules.rs (under /home/hyl/robonix/services/memory/rbnx-build/proto-staging)
[codegen] robonix-codegen --lang mcp ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] mcp: 37 packages, 177 msgs -> /home/hyl/robonix/services/memory/rbnx-build/codegen/robonix_mcp_types
[codegen] grpc_tools.protoc → /home/hyl/robonix/services/memory/rbnx-build/codegen/proto_gen
[codegen] done — proto+mcp+stubs, mcp_types,  setup.bash
[build] done.
✓ Package 'com.robonix.example.memgraph_service' build finished
  -> service memory
[Building] com.robonix.example.memsearch_service via manifest.build
[build] error: 'uv' not found on PATH. Install: https://docs.astral.sh/uv/
  -> service speech
[Building] com.robonix.example.speech_service via manifest.build
[build] error: 'uv' not found on PATH. Install: https://docs.astral.sh/uv/
  -> service voiceprint
[Building] com.robonix.example.voiceprint_service via manifest.build
[build] error: 'uv' not found on PATH. Install: https://docs.astral.sh/uv/
  -> service mapping
[Building] com.robonix.service.mapping via manifest.build
[build] rbnx codegen --mcp
[codegen] package: /home/hyl/robonix/examples/webots/rbnx-boot/cache/mapping_rbnx
[codegen] robonix source: /home/hyl/robonix
[codegen] robonix-codegen --lang proto ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] proto: 37 packages, 177 msgs, 82 srv -> /home/hyl/robonix/examples/webots/rbnx-boot/cache/mapping_rbnx/rbnx-build/proto-staging
[robonix-codegen] contracts: robonix_contracts.proto + contract_proto_modules.rs (under /home/hyl/robonix/examples/webots/rbnx-boot/cache/mapping_rbnx/rbnx-build/proto-staging)
[codegen] robonix-codegen --lang mcp ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] mcp: 37 packages, 177 msgs -> /home/hyl/robonix/examples/webots/rbnx-boot/cache/mapping_rbnx/rbnx-build/codegen/robonix_mcp_types
[codegen] grpc_tools.protoc → /home/hyl/robonix/examples/webots/rbnx-boot/cache/mapping_rbnx/rbnx-build/codegen/proto_gen
[codegen] done — proto+mcp+stubs, mcp_types,  setup.bash
[build] target=x86-docker
[build] image robonix-mapping present; rebuilding incrementally
[build] docker build -f docker/Dockerfile -t robonix-mapping
[+] Building 2.6s (13/13) FINISHED                                                                                                                                                                                                                                       docker:default
 => [internal] load build definition from Dockerfile                                                                                                                                                                                                                               0.0s
 => => transferring dockerfile: 3.64kB                                                                                                                                                                                                                                             0.0s
 => [internal] load metadata for docker.io/library/ros:humble-ros-base                                                                                                                                                                                                             2.5s
 => [internal] load .dockerignore                                                                                                                                                                                                                                                  0.0s
 => => transferring context: 2B                                                                                                                                                                                                                                                    0.0s
 => [1/8] FROM docker.io/library/ros:humble-ros-base@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16                                                                                                                                                       0.0s
 => => resolve docker.io/library/ros:humble-ros-base@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16                                                                                                                                                       0.0s
 => [internal] load build context                                                                                                                                                                                                                                                  0.0s
 => => transferring context: 5.08kB                                                                                                                                                                                                                                                0.0s
 => CACHED [2/8] RUN set -eux;     find /etc/apt/sources.list.d/ -maxdepth 1 ( -name '*.list' -o -name '*.sources' ) -print0       | xargs -0 -r sed -i --follow-symlinks           -e 's|http://packages\.ros\.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|  0.0s
 => CACHED [3/8] RUN apt-get update  && apt-get install -y --no-install-recommends         python3-pip         python3-numpy         python3-yaml         ros-humble-rtabmap-ros         ros-humble-tf2-ros         ros-humble-tf-transformations         ros-humble-pointcloud-t  0.0s
 => CACHED [4/8] RUN pip install --no-cache-dir         grpcio>=1.60.0         protobuf>=4.25.0         pyyaml>=6.0         numpy         pillow         mcp>=1.28.1                                                                                                               0.0s
 => CACHED [5/8] WORKDIR /mapping                                                                                                                                                                                                                                                  0.0s
 => CACHED [6/8] COPY entrypoint.sh /entrypoint.sh                                                                                                                                                                                                                                 0.0s
 => CACHED [7/8] COPY no_shm_profile.xml /etc/fastrtps_no_shm.xml                                                                                                                                                                                                                  0.0s
 => CACHED [8/8] RUN chmod +x /entrypoint.sh                                                                                                                                                                                                                                       0.0s
 => exporting to image                                                                                                                                                                                                                                                             0.0s
 => => exporting layers                                                                                                                                                                                                                                                            0.0s
 => => exporting manifest sha256:7ab1a74ffd3371cd63f03c31789731256e051b9d3b221f5ee010d0a7faa04f1e                                                                                                                                                                                  0.0s
 => => exporting config sha256:a93bd59283fe44e4d58f8c1f89bbda5b83552fa1337bb491558ef268926b08ae                                                                                                                                                                                    0.0s
 => => exporting attestation manifest sha256:25cb1a964af5ed0b153ad5c3218d84c54259358c39ab0ec3625bf1a2dce18f81                                                                                                                                                                      0.0s
 => => exporting manifest list sha256:9fcbb4d6dbb2f7e7aac34fc4b34e36322ccec7e10f667cbe897285595204e533                                                                                                                                                                             0.0s
 => => naming to docker.io/library/robonix-mapping:latest                                                                                                                                                                                                                          0.0s
 => => unpacking to docker.io/library/robonix-mapping:latest                                                                                                                                                                                                                       0.0s
[build] done (target=x86-docker).
✓ Package 'com.robonix.service.mapping' build finished
  -> service nav2
[Building] com.robonix.service.nav2 via manifest.build
[nav2/build] rbnx codegen --mcp 
[codegen] package: /home/hyl/robonix/examples/webots/rbnx-boot/cache/nav2_wrapper_rbnx
[codegen] robonix source: /home/hyl/robonix
[codegen] robonix-codegen --lang proto ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] proto: 37 packages, 177 msgs, 82 srv -> /home/hyl/robonix/examples/webots/rbnx-boot/cache/nav2_wrapper_rbnx/rbnx-build/proto-staging
[robonix-codegen] contracts: robonix_contracts.proto + contract_proto_modules.rs (under /home/hyl/robonix/examples/webots/rbnx-boot/cache/nav2_wrapper_rbnx/rbnx-build/proto-staging)
[codegen] robonix-codegen --lang mcp ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] mcp: 37 packages, 177 msgs -> /home/hyl/robonix/examples/webots/rbnx-boot/cache/nav2_wrapper_rbnx/rbnx-build/codegen/robonix_mcp_types
[codegen] grpc_tools.protoc → /home/hyl/robonix/examples/webots/rbnx-boot/cache/nav2_wrapper_rbnx/rbnx-build/codegen/proto_gen
[codegen] done — proto+mcp+stubs, mcp_types,  setup.bash
[nav2/build] target=x86-docker
[nav2/build] docker build -f docker/Dockerfile -t robonix-nav2
[+] Building 0.2s (13/13) FINISHED                                                                                                                                                                                                                                       docker:default
 => [internal] load build definition from Dockerfile                                                                                                                                                                                                                               0.0s
 => => transferring dockerfile: 2.76kB                                                                                                                                                                                                                                             0.0s
 => [internal] load metadata for docker.io/library/ros:humble-ros-base                                                                                                                                                                                                             0.1s
 => [internal] load .dockerignore                                                                                                                                                                                                                                                  0.0s
 => => transferring context: 2B                                                                                                                                                                                                                                                    0.0s
 => [1/8] FROM docker.io/library/ros:humble-ros-base@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16                                                                                                                                                       0.0s
 => => resolve docker.io/library/ros:humble-ros-base@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16                                                                                                                                                       0.0s
 => [internal] load build context                                                                                                                                                                                                                                                  0.0s
 => => transferring context: 4.26kB                                                                                                                                                                                                                                                0.0s
 => CACHED [2/8] RUN set -eux;     find /etc/apt/sources.list.d/ -maxdepth 1 ( -name '*.list' -o -name '*.sources' ) -print0       | xargs -0 -r sed -i --follow-symlinks           -e 's|http://packages\.ros\.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|  0.0s
 => CACHED [3/8] RUN apt-get update  && apt-get install -y --no-install-recommends         python3-pip         python3-yaml         ros-humble-navigation2         ros-humble-nav2-bringup         ros-humble-tf2-ros         ros-humble-pointcloud-to-laserscan         ros-humb  0.0s
 => CACHED [4/8] RUN pip install --no-cache-dir         grpcio>=1.60.0         mcp>=1.0         fastmcp>=3         protobuf>=4.25.0         pyyaml>=6.0                                                                                                                            0.0s
 => CACHED [5/8] WORKDIR /nav2                                                                                                                                                                                                                                                     0.0s
 => CACHED [6/8] COPY entrypoint.sh /entrypoint.sh                                                                                                                                                                                                                                 0.0s
 => CACHED [7/8] COPY no_shm_profile.xml /etc/fastrtps_no_shm.xml                                                                                                                                                                                                                  0.0s
 => CACHED [8/8] RUN chmod +x /entrypoint.sh                                                                                                                                                                                                                                       0.0s
 => exporting to image                                                                                                                                                                                                                                                             0.0s
 => => exporting layers                                                                                                                                                                                                                                                            0.0s
 => => exporting manifest sha256:33fa43b3730931aad5aa60203b7094fefb7cdc15e882cd513dc4b06d2e9c5d09                                                                                                                                                                                  0.0s
 => => exporting config sha256:45f953d6f0e2b1b6dd49d9a8b6b5ea7e99d31110ac1202baef8aa932c77b9dba                                                                                                                                                                                    0.0s
 => => exporting attestation manifest sha256:85ec1263d25a395d147c03e975cf4c62b7779949fa728c57627699b82829531c                                                                                                                                                                      0.0s
 => => exporting manifest list sha256:13f8369682d388ed1124f6dfded6064d5029aaefd17794668199b02dce76be36                                                                                                                                                                             0.0s
 => => naming to docker.io/library/robonix-nav2:latest                                                                                                                                                                                                                             0.0s
 => => unpacking to docker.io/library/robonix-nav2:latest                                                                                                                                                                                                                          0.0s
[nav2/build] done (target=x86-docker).
✓ Package 'com.robonix.service.nav2' build finished
  -> skill explore
[Building] com.robonix.skill.explore via manifest.build
[build] rbnx codegen --mcp 
[codegen] package: /home/hyl/robonix/examples/webots/rbnx-boot/cache/explore_rbnx
[codegen] robonix source: /home/hyl/robonix
[codegen] robonix-codegen --lang proto ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] proto: 38 packages, 177 msgs, 85 srv -> /home/hyl/robonix/examples/webots/rbnx-boot/cache/explore_rbnx/rbnx-build/proto-staging
[robonix-codegen] contracts: robonix_contracts.proto + contract_proto_modules.rs (under /home/hyl/robonix/examples/webots/rbnx-boot/cache/explore_rbnx/rbnx-build/proto-staging)
[codegen] robonix-codegen --lang mcp ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] mcp: 38 packages, 177 msgs -> /home/hyl/robonix/examples/webots/rbnx-boot/cache/explore_rbnx/rbnx-build/codegen/robonix_mcp_types
[codegen] grpc_tools.protoc → /home/hyl/robonix/examples/webots/rbnx-boot/cache/explore_rbnx/rbnx-build/codegen/proto_gen
[codegen] done — proto+mcp+stubs, mcp_types,  setup.bash
[build] image robonix-explore present; rebuilding incrementally
[build] docker build -f docker/Dockerfile -t robonix-explore
[+] Building 0.2s (13/13) FINISHED                                                                                                                                                                                                                                       docker:default
 => [internal] load build definition from Dockerfile                                                                                                                                                                                                                               0.0s
 => => transferring dockerfile: 3.03kB                                                                                                                                                                                                                                             0.0s
 => [internal] load metadata for docker.io/library/ros:humble-ros-base                                                                                                                                                                                                             0.1s
 => [internal] load .dockerignore                                                                                                                                                                                                                                                  0.0s
 => => transferring context: 2B                                                                                                                                                                                                                                                    0.0s
 => [1/8] FROM docker.io/library/ros:humble-ros-base@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16                                                                                                                                                       0.0s
 => => resolve docker.io/library/ros:humble-ros-base@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16                                                                                                                                                       0.0s
 => [internal] load build context                                                                                                                                                                                                                                                  0.0s
 => => transferring context: 3.58kB                                                                                                                                                                                                                                                0.0s
 => CACHED [2/8] RUN set -eux;     find /etc/apt/sources.list.d/ -maxdepth 1 ( -name '*.list' -o -name '*.sources' ) -print0       | xargs -0 -r sed -i --follow-symlinks           -e 's|http://packages\.ros\.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|  0.0s
 => CACHED [3/8] RUN apt-get update  && apt-get install -y --no-install-recommends         python3-pip         python3-numpy         python3-scipy         python3-yaml         ros-humble-tf2-ros         ros-humble-tf-transformations         ros-humble-rmw-zenoh-cpp  && rm   0.0s
 => CACHED [4/8] RUN pip install --no-cache-dir         grpcio>=1.60.0         protobuf>=4.25.0         pyyaml>=6.0         numpy         fastmcp         uvicorn                                                                                                                  0.0s
 => CACHED [5/8] WORKDIR /explore                                                                                                                                                                                                                                                  0.0s
 => CACHED [6/8] COPY entrypoint.sh /entrypoint.sh                                                                                                                                                                                                                                 0.0s
 => CACHED [7/8] COPY no_shm_profile.xml /etc/fastrtps_no_shm.xml                                                                                                                                                                                                                  0.0s
 => CACHED [8/8] RUN chmod +x /entrypoint.sh                                                                                                                                                                                                                                       0.0s
 => exporting to image                                                                                                                                                                                                                                                             0.0s
 => => exporting layers                                                                                                                                                                                                                                                            0.0s
 => => exporting manifest sha256:00eaaffa33c0925e1165cf1026f135c6813a63597c441e4d5a5a4a12b77cc935                                                                                                                                                                                  0.0s
 => => exporting config sha256:16b2cf1fc6b0f7b3a3bc3ef99c33fc25c272829c60bc504adb5254e81be957e9                                                                                                                                                                                    0.0s
 => => exporting attestation manifest sha256:aa056b698ad1ffc052e8aa493fa7f253b1dd77a2bc8b5f594e3252c5b0082618                                                                                                                                                                      0.0s
 => => exporting manifest list sha256:00cdf7a98009f2b915e2b8ae8463ba345efc6e5c1a9ebce033a77e28e2c495fa                                                                                                                                                                             0.0s
 => => naming to docker.io/library/robonix-explore:latest                                                                                                                                                                                                                          0.0s
 => => unpacking to docker.io/library/robonix-explore:latest                                                                                                                                                                                                                       0.0s
[build] done.
✓ Package 'com.robonix.skill.explore' build finished
  -> system scene
[Building] com.robonix.system.scene via manifest.build
[build] rbnx codegen --mcp
[codegen] package: /home/hyl/robonix/system/scene
[codegen] robonix source: /home/hyl/robonix
[codegen] robonix-codegen --lang proto ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] proto: 37 packages, 177 msgs, 82 srv -> /home/hyl/robonix/system/scene/rbnx-build/proto-staging
[robonix-codegen] contracts: robonix_contracts.proto + contract_proto_modules.rs (under /home/hyl/robonix/system/scene/rbnx-build/proto-staging)
[codegen] robonix-codegen --lang mcp ...
[robonix-codegen] 4 IDL msg/srv skipped (unresolved deps); use --verbose for details
[robonix-codegen] mcp: 37 packages, 177 msgs -> /home/hyl/robonix/system/scene/rbnx-build/codegen/robonix_mcp_types
[codegen] grpc_tools.protoc → /home/hyl/robonix/system/scene/rbnx-build/codegen/proto_gen
[codegen] done — proto+mcp+stubs, mcp_types,  setup.bash
[build] weight already present: yolov8l-world.pt
[build] weight already present: mobile_sam.pt
[build] ROS distro: humble (set ROBONIX_SCENE_ROS_DISTRO to change)
[build] proxy disabled by RBNX_BUILD_PROXY=0
[build] docker build -f docker/Dockerfile -t robonix-scene docker/  (target=x86-docker)
[+] Building 1.3s (29/29) FINISHED                                                                                                                                                                                                                                       docker:default
 => [internal] load build definition from Dockerfile                                                                                                                                                                                                                               0.0s
 => => transferring dockerfile: 11.20kB                                                                                                                                                                                                                                            0.0s
 => resolve image config for docker-image://docker.io/docker/dockerfile:1.7                                                                                                                                                                                                        0.4s
 => CACHED docker-image://docker.io/docker/dockerfile:1.7@sha256:a57df69d0ea827fb7266491f2813635de6f17269be881f696fbfdf2d83dda33e                                                                                                                                                  0.0s
 => => resolve docker.io/docker/dockerfile:1.7@sha256:a57df69d0ea827fb7266491f2813635de6f17269be881f696fbfdf2d83dda33e                                                                                                                                                             0.0s
 => [internal] load metadata for docker.io/library/ros:humble-ros-base                                                                                                                                                                                                             0.1s
 => [internal] load .dockerignore                                                                                                                                                                                                                                                  0.0s
 => => transferring context: 2B                                                                                                                                                                                                                                                    0.0s
 => [stage-0  1/22] FROM docker.io/library/ros:humble-ros-base@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16                                                                                                                                             0.0s
 => => resolve docker.io/library/ros:humble-ros-base@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16                                                                                                                                                       0.0s
 => [internal] load build context                                                                                                                                                                                                                                                  0.5s
 => => transferring context: 136.43MB                                                                                                                                                                                                                                              0.5s
 => CACHED [stage-0  2/22] RUN cat > /usr/local/bin/with-build-proxy <<'EOF'  && chmod +x /usr/local/bin/with-build-proxy  && cat > /usr/local/bin/without-proxy <<'EOF2'  && chmod +x /usr/local/bin/without-proxy                                                                0.0s
 => CACHED [stage-0  3/22] RUN set -eux;     find /etc/apt/sources.list.d/ -maxdepth 1 ( -name '*.list' -o -name '*.sources' ) -print0       | xargs -0 -r sed -i --follow-symlinks           -e 's|http://packages\.ros\.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ro  0.0s
 => CACHED [stage-0  4/22] RUN --mount=type=cache,target=/var/cache/apt,sharing=locked     without-proxy sh -c '         apt-get update &&         apt-get install -y --no-install-recommends             python3-pip             python3-cv-bridge             python3-numpy      0.0s
 => CACHED [stage-0  5/22] RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked     without-proxy python3 -m pip install         --progress-bar on         --timeout 60         --retries 3         --upgrade pip                                                         0.0s
 => CACHED [stage-0  6/22] RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked     without-proxy sh -c 'set -eu;         base="https://mirrors.aliyun.com/pytorch-wheels/cu128";         curl -fSL --retry 5 --retry-delay 2 -C - -o /tmp/torch.whl              "$base  0.0s
 => CACHED [stage-0  7/22] COPY requirements/scene-base.txt /tmp/requirements/scene-base.txt                                                                                                                                                                                       0.0s
 => CACHED [stage-0  8/22] RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked     without-proxy pip install -r /tmp/requirements/scene-base.txt         --progress-bar on         --timeout 60         --retries 3     && rm /tmp/requirements/scene-base.txt           0.0s
 => CACHED [stage-0  9/22] COPY requirements/scene-perception-core.txt /tmp/requirements/scene-perception-core.txt                                                                                                                                                                 0.0s
 => CACHED [stage-0 10/22] RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked     without-proxy pip install -r /tmp/requirements/scene-perception-core.txt         --progress-bar on         --timeout 60         --retries 3     && rm /tmp/requirements/scene-percep  0.0s
 => CACHED [stage-0 11/22] COPY requirements/scene-perception-heavy.txt /tmp/requirements/scene-perception-heavy.txt                                                                                                                                                               0.0s
 => CACHED [stage-0 12/22] RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked     without-proxy pip install -r /tmp/requirements/scene-perception-heavy.txt         --progress-bar on         --timeout 60         --retries 3     && rm /tmp/requirements/scene-perce  0.0s
 => CACHED [stage-0 13/22] RUN set -eux;     rm -rf /opt/concept-graphs;     with-build-proxy git clone --depth 1 --branch ali-dev         https://ghfast.top/https://github.com/concept-graphs/concept-graphs.git         /opt/concept-graphs     || (         rm -rf /opt/conce  0.0s
 => CACHED [stage-0 14/22] RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked     without-proxy pip install --no-deps -e /opt/concept-graphs                                                                                                                            0.0s
 => CACHED [stage-0 15/22] RUN mkdir -p /opt/models /opt/models/hf /root/.cache/clip                                                                                                                                                                                               0.0s
 => CACHED [stage-0 16/22] COPY _weights/yolov8l-world.pt /opt/models/yolov8l-world.pt                                                                                                                                                                                             0.0s
 => CACHED [stage-0 17/22] COPY _weights/mobile_sam.pt    /opt/models/mobile_sam.pt                                                                                                                                                                                                0.0s
 => CACHED [stage-0 18/22] RUN with-build-proxy sh <<'EOF'                                                                                                                                                                                                                         0.0s
 => CACHED [stage-0 19/22] WORKDIR /scene                                                                                                                                                                                                                                          0.0s
 => CACHED [stage-0 20/22] COPY entrypoint.sh /entrypoint.sh                                                                                                                                                                                                                       0.0s
 => CACHED [stage-0 21/22] COPY no_shm_profile.xml /etc/fastrtps_no_shm.xml                                                                                                                                                                                                        0.0s
 => CACHED [stage-0 22/22] RUN chmod +x /entrypoint.sh                                                                                                                                                                                                                             0.0s
 => exporting to image                                                                                                                                                                                                                                                             0.1s
 => => exporting layers                                                                                                                                                                                                                                                            0.0s
 => => exporting manifest sha256:f564126c98618b651b85811844775600e99f784cbe843c261bad2c2c3f7f23e2                                                                                                                                                                                  0.0s
 => => exporting config sha256:44fd454c2368883add903b8d7f206e9b6028cb1d8096e2b37edb5050f241ab45                                                                                                                                                                                    0.0s
 => => exporting attestation manifest sha256:69ec115afd8e0e28db2c6b4f164cb4afd0219132a7648d19506646a388ed0590                                                                                                                                                                      0.0s
 => => exporting manifest list sha256:136c220abd97411cec90474c3c3de39791dd7b1057ee9980583d5ae36071dd06                                                                                                                                                                             0.0s
 => => naming to docker.io/library/robonix-scene:latest                                                                                                                                                                                                                            0.0s
 => => unpacking to docker.io/library/robonix-scene:latest                                                                                                                                                                                                                         0.0s
[build] done.
✓ Package 'com.robonix.system.scene' build finished

════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════ Build summary ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
  Manifest: /home/hyl/robonix/examples/webots/robonix_manifest.yaml
  Built: 9   Fetched: 0   Skipped: 0   Failed: 3   Total: 12

     section    name           package.name                          version  location                                                           
  ─  ─────────  ─────────────  ────────────────────────────────────  ───────  ───────────────────────────────────────────────────────────────────
  ✓  primitive  tiago_chassis  com.robonix.example.tiago_chassis     0.1.0    /home/hyl/robonix/examples/webots/primitives/tiago_chassis
  ✓  primitive  tiago_camera   com.robonix.example.tiago_camera      0.1.0    /home/hyl/robonix/examples/webots/primitives/tiago_camera
  ✓  primitive  tiago_lidar    com.robonix.example.tiago_lidar       0.1.0    /home/hyl/robonix/examples/webots/primitives/tiago_lidar
  ✓  primitive  audio_driver   com.robonix.example.audio_driver      0.1.0    /home/hyl/robonix/examples/webots/primitives/audio_driver
  ✓  service    memgraph       com.robonix.example.memgraph_service  0.1.0    /home/hyl/robonix/services/memory
  ✓  service    mapping        com.robonix.service.mapping           0.4.0    /home/hyl/robonix/examples/webots/rbnx-boot/cache/mapping_rbnx
                                                                              ↳ https://github.com/enkerewpo/mapping_rbnx (branch=main)
  ✓  service    nav2           com.robonix.service.nav2              0.1.0    /home/hyl/robonix/examples/webots/rbnx-boot/cache/nav2_wrapper_rbnx
                                                                              ↳ https://github.com/enkerewpo/nav2_wrapper_rbnx (branch=main)
  ✓  skill      explore        com.robonix.skill.explore             0.1.0    /home/hyl/robonix/examples/webots/rbnx-boot/cache/explore_rbnx
                                                                              ↳ https://github.com/enkerewpo/explore_rbnx (branch=main)
  ✓  system     scene          com.robonix.system.scene              0.1.0    /home/hyl/robonix/system/scene

  ✗ service/memory: Build exited with status Some(1)
  ✗ service/speech: Build exited with status Some(1)
  ✗ service/voiceprint: Build exited with status Some(1)
════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
Error: 3 package(s) failed to build from robonix_manifest.yaml
(base) hyl@lenovo-ThinkStation-PX:~/robonix/examples/webots$ 


