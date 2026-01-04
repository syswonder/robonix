#! /bin/bash

make build-sdk
eval $(make source-sdk)

rbnx package build
rbnx deploy register demo_recipe.yaml
rbnx deploy restart

# rbnx task create "Pick up the red box on the table"
# rbnx task get task_0

# ros2 service call /demo_service/task_plan/plan robonix_sdk/srv/PlanTask "{description: 'test', params: {keys: ['object_graph'], values: ['[]']}}"