#!/bin/bash

set -e

# Define paths to the packages
DYNAMIXEL_DIR=~/hros5_ws/src/dynamixel
PACKAGES=(
  DynamixelSDK
  dynamixel_interfaces
  dynamixel_hardware_interface
)

echo "🔓 Unignoring packages..."
for pkg in "${PACKAGES[@]}"; do
  rm -f "$DYNAMIXEL_DIR/$pkg/COLCON_IGNORE"
done

echo "🔄 Pulling latest from GitHub (jazzy branch)..."
for pkg in "${PACKAGES[@]}"; do
  cd "$DYNAMIXEL_DIR/$pkg"
  git checkout jazzy && git pull
done

echo "🔨 Rebuilding Dynamixel packages..."
cd ~/hros5_ws
colcon build --packages-select dynamixel_sdk dynamixel_interfaces dynamixel_hardware_interface

echo "🔒 Reapplying COLCON_IGNORE to freeze packages..."
for pkg in "${PACKAGES[@]}"; do
  touch "$DYNAMIXEL_DIR/$pkg/COLCON_IGNORE"
done

echo "✅ Dynamixel packages updated, built, and ignored again."
source install/setup.bash

