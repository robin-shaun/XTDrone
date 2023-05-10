FROM jenniferbuehler/general-message-pkgs 

MAINTAINER Jennifer Buehler

# Install required ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-indigo-gazebo-ros \
    ros-indigo-eigen-conversions \
    ros-indigo-roslint \
    && rm -rf /var/lib/apt/lists/

COPY gazebo_grasp_plugin /catkin_ws/src/gazebo_grasp_plugin
COPY gazebo_state_plugins /catkin_ws/src/gazebo_state_plugins
COPY gazebo_test_tools /catkin_ws/src/gazebo_test_tools
COPY gazebo_world_plugin_loader /catkin_ws/src/gazebo_world_plugin_loader

# Build
RUN bin/bash -c "source /.bashrc \
    && cd /catkin_ws \
    && catkin_make \
    && catkin_make install"

RUN bin/bash -c "source .bashrc"

CMD ["bash","-l"]
