#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Stop Base Navigate Primitive Script
#
# Stop script for primitive::base.navigate (navigate to target pose)
# Note: Nav2 is typically managed by launch files, so this is a no-op.

set -e

echo "Base navigate primitive stop requested (no-op: Nav2 is managed by launch files)"
exit 0
