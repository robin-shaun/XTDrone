#!/bin/sh

# the function relocates all the ROS remappings in the command at the end of the
# string this allows some punky uses of rosrun, for more information see:
# https://github.com/ros-simulation/gazebo_ros_pkgs/issues/387
relocate_remappings()
{
  command_line=${1}

  for w in $command_line; do
    if $(echo "$w" | grep -q ':='); then
      ros_remaps="$ros_remaps $w"
    else
      gazebo_args="$gazebo_args $w"
    fi
  done

  echo "$gazebo_args$ros_remaps" | cut -c 1-
}
