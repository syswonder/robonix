#!/bin/bash
ARG=$1
python3 -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. $ARG