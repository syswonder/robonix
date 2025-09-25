#!/bin/bash
# Generate gRPC Python code from robot_control.proto
python3 -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. robot_control.proto