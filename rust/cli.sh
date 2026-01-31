rbnx daemon stop 2>/dev/null || true

# make build-sdk
eval $(make source-sdk)
rbnx deploy build
rbnx deploy register demo_recipe.yaml
rm -rf ./provider/logs
mkdir -p ./provider/logs
rbnx deploy restart