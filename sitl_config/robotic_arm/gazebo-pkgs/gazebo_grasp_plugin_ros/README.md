gazebo_grasp_plugin_ros
-----------------------

ROS-specific components for `gazebo_grasp_plugin`.

This package contains a re-publisher for the grasp event message published by
the gazebo grasp plugin.

To run the node, after starting up your robot with the gazebo grasp plugin loaded:

rosrun gazebo_grasp_plugin_ros grasp_event_republisher

The topic will be published on "<node-name>/grasp_event".
