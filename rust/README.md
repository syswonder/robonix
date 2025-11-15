# Robonix Quick Start

wheatfox

## NOTES

```bash
mkdir -p ~/.robonix/packages; rm -rf ~/.robonix/packages; ln -s "$(realpath ../provider/)" ~/.robonix/packages;
export FASTRTPS_DEFAULT_PROFILES_FILE=
```

## HOW TO INSTALL A PACKAGE

```bash
# in robonix-cli, run:
cargo run -- package install --github https://github.com/enkerewpo/demo-package-01-robonix
cargo run -- package list
cargo run -- package info demo_package_01_github
```

## HOW TO REGISTER A MODEL (FOR PLANNING)

```bash
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

## HOW TO REGISTER A PACKAGE TO ROBONIX CORE

using a recipe file:
```bash
# in robonix-cli, run:
cargo run -- config --set-msg-path ../robonix-msg

cd robonix-cli
cargo run -- deploy register demo_recipe.yaml
cargo run -- deploy start
cargo run -- deploy status
# cargo run -- deploy stop
# cargo run -- deploy unregister
```

Note: robonix-msg setup is automatically sourced by start scripts. The CLI will:
1. First check config file (set via `rbnx config --set-msg-path`)
2. Then check ROBONIX_MSG_PATH environment variable

## HOW TO ISSUE A NATURAL LANGUAGE TASK AND LET THE SYSTEM RUN

```bash
cargo run -- task create "Pick up the red box"
cargo run -- task list
```