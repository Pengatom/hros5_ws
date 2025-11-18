#!/bin/bash
# ~/hros5_ws/scripts/generate_compile_commands.sh

# Go to workspace root
cd "$(dirname "$0")/.." || exit 1

# Build with compile commands
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Symlink the compile_commands.json to workspace root
ln -sf build/hros5_head_tracker_cpp/compile_commands.json compile_commands.json

echo "compile_commands.json updated in workspace root"
