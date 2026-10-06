#!/bin/bash
# Start the SIH forest simulation on the NVIDIA GPU (Terminal 1).
# Usage:  ~/SIH/launch/start_sim.sh            -> with the Gazebo window (for recording)
#         ~/SIH/launch/start_sim.sh headless   -> no Gazebo window (faster; watch in RViz)

HEADLESS=False
if [ "$1" == "headless" ]; then
  HEADLESS=True
  echo ">>> Starting Gazebo WITHOUT its window (headless). Watch the robot in RViz."
fi

source /opt/ros/jazzy/setup.bash
source ~/SIH/bot_basics2/install/setup.bash

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/SIH/forest_world/Forest3D/models
export ROBOT_NAMESPACE=

# Render on the RTX 3050 instead of the Intel chip
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

ros2 launch rosbot_gazebo simulation.yaml \
  robot_model:=rosbot \
  configuration:=basic \
  gz_world:=$HOME/SIH/forest_world/Forest3D/forest.world \
  gz_headless_mode:=$HEADLESS \
  x:=9.0 \
  y:=-45.0 \
  z:=39.5 \
  rviz:=False