# Robonix Quick Start

wheatfox

## prepare

first you need a ROS2 environment with rust installed.

```bash
cd rust # at robonix src root folder
cd robonix-cli
cargo build
# in robonix-cli, run:
mkdir -p ~/.robonix; rm -rf ~/.robonix/packages; ln -s "$(realpath ../provider/)" ~/.robonix/packages;
export FASTRTPS_DEFAULT_PROFILES_FILE=
```

## start robonix-core

before everything, we need to start robonix-core (in a separate terminal).
```bash
cd robonix-core
cargo run --
```

then you can start another terminal to use `robonix-cli`.

## how to install a package

```bash
cd robonix-cli
# in robonix-cli, run:
cargo run -- config -s # print current robonix system config (normally under ~/.robonix/config.yaml)
cargo run -- config --set-msg-path ../robonix-msg # update the robonix-msg path to the local robonix-msg folder
cargo run -- config -s

# install a package from github
cargo run -- package install --github https://github.com/enkerewpo/demo-package-01-robonix
cargo run -- package list
cargo run -- package info demo_package_01_github
```

## how to register a model (for planning)

```bash
cd robonix-cli
export ROBONIX_MODEL_API_KEY=sk-xxxx # or add --api-key sk-xxxx to the command
# in robonix-cli, run:
cargo run -- model register \
  --model-id deepseek-chat \
  --model-name "DeepSeek Chat" \
  --model-type llm \
  --provider deepseek \
  --api-endpoint https://api.deepseek.com/v1/chat \
  --description "DeepSeek Chat model via DeepSeek API" \
  --capabilities "planning,reasoning,general"
```

## how to register a package to robonix core

using a recipe file:
```bash
cd robonix-cli
# in robonix-cli, run:
cargo run -- deploy register demo_recipe.yaml
cargo run -- deploy start
cargo run -- deploy status
cargo run -- deploy restart
# cargo run -- deploy stop
# cargo run -- deploy unregister demo_recipe.yaml
```

Note: robonix-msg setup is automatically sourced by start scripts. The CLI will:
1. First check config file (set via `rbnx config --set-msg-path`)
2. Then check ROBONIX_MSG_PATH environment variable

## how to issue a natural language task and let the system run

```bash
cargo run -- task create "Pick up the red box"
cargo run -- task list
```