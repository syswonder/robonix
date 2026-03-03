#!/bin/env bash
# only use this script when you want to test packages under provider directory
# and only use this script in a fresh docker container booted from docker/run.sh

set -e

make build-sdk
make install
make setup-dev
eval $(make source-sdk)
rbnx config --set-sdk-path $(realpath ./robonix-sdk)
rbnx config --show
rbnx package list
rbnx package build
