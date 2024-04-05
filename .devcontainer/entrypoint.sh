#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS ${ROS_DISTRO}"

if [ -f /underlay_ws/devel/setup.bash ]
then
  source /underlay_ws/devel/setup.bash
  echo "Sourced underlay_ws workspace"
fi

if [ -f /overlay_ws/devel/setup.bash ]
then
  source /overlay_ws/devel/setup.bash
  echo "Sourced overlay_ws workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"