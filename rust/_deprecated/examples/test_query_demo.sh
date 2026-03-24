#!/usr/bin/env bash
# Test query_demo: semantic_query server + client
exec bash "$(dirname "$0")/test_demo.sh" query_demo semantic_server semantic_client
