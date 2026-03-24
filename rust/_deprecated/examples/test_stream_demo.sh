#!/usr/bin/env bash
# Test stream_demo: pose publisher + subscriber
exec bash "$(dirname "$0")/test_demo.sh" stream_demo stream_server stream_client
