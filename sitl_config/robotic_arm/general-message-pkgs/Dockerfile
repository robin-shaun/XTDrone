FROM jenniferbuehler/ros-indigo-full-catkin 

MAINTAINER Jennifer Buehler

# Install system essentials
RUN apt-get update && apt-get install -y \
    cmake \
    sudo \
    vim \
    && rm -rf /var/lib/apt/lists/*

# need g++ for compiling with cmake even if gcc
# is already installed
RUN apt-get update && apt-get install -y g++ \
    && rm -rf /var/lib/apt/lists/*

# Install required ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-indigo-object-recognition-msgs \
    && rm -rf /var/lib/apt/lists/

COPY object_msgs /catkin_ws/src/object_msgs
COPY object_msgs_tools /catkin_ws/src/object_msgs_tools
COPY path_navigation_msgs /catkin_ws/src/path_navigation_msgs

# Build
RUN bin/bash -c "source /.bashrc \
    && cd /catkin_ws \
    && catkin_make \
    && catkin_make install"

RUN bin/bash -c "source .bashrc"

CMD ["bash","-l"]
