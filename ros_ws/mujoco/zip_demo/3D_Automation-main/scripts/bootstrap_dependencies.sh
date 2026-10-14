#!/usr/bin/env bash
set -euo pipefail

cd /workspace/3D_Automation
mkdir -p src/vendor

vcs import src/vendor < deps/turtlebot3.repos

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
