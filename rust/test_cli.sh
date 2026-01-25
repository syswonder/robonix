#! /bin/bash

# make build-sdk
eval $(make source-sdk)
rbnx deploy build
rbnx deploy register demo_recipe.yaml
rm -rf ./provider/logs
mkdir -p ./provider/logs
rbnx deploy restart

# rbnx task create "Pick up the red box on the table"
# rbnx task create "Just wandering around and let semantic map service to build the map"
# rbnx task create "go to the plant near sofa"

# ros2 service call /demo_service/task_plan/plan robonix_sdk/srv/PlanTask "{description: 'test', params: {keys: ['object_graph'], values: ['[]']}}"
# ros2 service call /demo_service/semantic_map/query robonix_sdk/srv/QuerySemanticMap "{types: ['']}"