#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Stop Base Pose AMCL Primitive Script
#
# Stop script for prm::base.pose.amcl
# No converter process to stop - just verify topic is available

# Nothing to stop for this primitive (AMCL is managed externally)
echo "prm::base.pose.amcl primitive stopped (AMCL topic /amcl_pose should remain available)"
