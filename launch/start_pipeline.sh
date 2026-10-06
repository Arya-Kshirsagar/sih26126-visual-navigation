#!/bin/bash
# Start the SIH pipeline (bridge, stereo, visual odometry, SLAM, Nav2, RViz).
# Run after the sim is up (start_sim.sh).
# Usage:  ~/SIH/launch/start_pipeline.sh                 (everything)
#         ~/SIH/launch/start_pipeline.sh nav:=false      (no Nav2)
#         ~/SIH/launch/start_pipeline.sh slam:=false nav:=false

source /opt/ros/jazzy/setup.bash
source ~/SIH/bot_basics2/install/setup.bash   # robot meshes and packages for RViz

ros2 launch ~/SIH/launch/pipeline.launch.py slam:=true nav:=true "$@"