#!/usr/bin/env bash
# Test skill_demo: greet command server + client
exec bash "$(dirname "$0")/test_demo.sh" skill_demo skill_server skill_client
