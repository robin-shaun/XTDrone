#!/bin/bash

# spawn_wamv.bash: A bash script to spawn a wamv model using gz.
#                  Used to avoid using gazebo_ros spawn_model, as
#                  it relies on /gazebo/model_states, which is not
#                  published when enable_ros_network:=false for competition
#
# E.g.: ./spawn_wamv.bash -x <x> -y <y> -z <z> -R <R> -P <P> -Y <Y> --urdf <urdf> --model <model>

set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

is_gzserver_running()
{
  if pgrep gzserver >/dev/null; then
    true
  else
    false
  fi
}

wait_until_gzserver_is_up()
{
  until is_gzserver_running
  do
    sleep 1s
  done

  # ToDo: Figure out a better way to check Gazebo is ready.
  sleep 4s
}

# Define usage function.
usage()
{
  echo "Usage: $0 -x <x> -y <y> -z <z> -R <R> -P <P> -Y <Y> --urdf <urdf> --model <model>"
  exit 1
}

# Call usage() function if arguments not supplied.
[[ $# -le 15 ]] && usage

# Parse arguments
POSITIONAL=()
while [[ $# -gt 0 ]]
do
  key="$1"
  
  case $key in
      -x)
      x="$2"
      shift # past argument
      shift # past value
      ;;
      -y)
      y="$2"
      shift # past argument
      shift # past value
      ;;
      -z)
      z="$2"
      shift # past argument
      shift # past value
      ;;
      -R)
      R="$2"
      shift # past argument
      shift # past value
      ;;
      -P)
      P="$2"
      shift # past argument
      shift # past value
      ;;
      -Y)
      Y="$2"
      shift # past argument
      shift # past value
      ;;
      --urdf)
      urdf="$2"
      shift # past argument
      shift # past value
      ;;
      --model)
      model="$2"
      shift # past argument
      shift # past value
      ;;
      *)    # unknown option
      POSITIONAL+=("$1") # save it in an array for later
      shift # past argument
      ;;
  esac
  done
set -- "${POSITIONAL[@]}" # restore positional parameters

# Ensure that gzserver has been started
wait_until_gzserver_is_up

# If input urdf is a xacro file, convert xacro to a urdf file
final_urdf=/tmp/my_urdf.urdf
rosrun xacro xacro --inorder -o $final_urdf $urdf

# Remove <package>/models
sed -e "s/vrx_gazebo\/models\///g" -i $final_urdf
sed -e "s/wamv_gazebo\/models\///g" -i $final_urdf
sed -e "s/wamv_description\/models\///g" -i $final_urdf

# Spawn model
gz model --model-name=$model --spawn-file=$final_urdf -x $x -y $y -z $z -R $R -P $P -Y $Y
