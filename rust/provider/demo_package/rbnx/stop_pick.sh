#!/bin/bash
# Stop script for skl::pick

# This script is called to stop the pick skill process
# CLI will also manage the process by PID, but this script can be used
# for additional cleanup if needed

# Kill by process name as fallback
pkill -f "pick_skill" || true

# Clean up any temporary files
rm -f /tmp/demo_pick_skill.log

exit 0

