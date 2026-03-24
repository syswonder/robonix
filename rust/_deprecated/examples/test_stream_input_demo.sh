#!/usr/bin/env bash
# Test stream_demo input: client sends to server (input stream)
exec bash "$(dirname "$0")/test_demo.sh" stream_demo input_demo_server input_demo_client
