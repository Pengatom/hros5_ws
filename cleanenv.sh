#!/bin/bash

echo "🧼 Cleaning environment..."

# Unset known catkin/ROS 1 vars
unset CATKIN_INSTALL_INTO_PREFIX_ROOT
unset CMAKE_PREFIX_PATH
unset AMENT_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH

# Source only ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Optional: Source your ROS 2 workspace if built
if [ -f ~/hros5_ws/install/setup.bash ]; then
  echo "🔗 Sourcing ROS 2 workspace"
  source ~/hros5_ws/install/setup.bash
fi

echo "✅ ROS 2 Jazzy environment is now clean."

