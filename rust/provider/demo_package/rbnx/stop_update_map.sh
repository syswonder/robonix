#!/bin/bash
# Stop script for skl::update_map

# This script is called to stop the update_map skill process
# CLI will also manage the process by PID, but this script can be used
# for additional cleanup if needed

# Kill by process name as fallback
pkill -f "update_map_skill" || true

# Clean up any temporary files
rm -f /tmp/demo_update_map_skill.log

exit 0

