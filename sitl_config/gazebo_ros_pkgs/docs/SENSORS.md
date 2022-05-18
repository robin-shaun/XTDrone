# Plugins supported in Gazebo ROS packages

## Initial note

As detailed in the CONTRIBUITON.md guide, Gazebo ROS packages are a wrapper that
connects upstream Gazebo simulations with the ROS framework. Ideally all the
plugins in this repository should implement the ROS wrapper over a gazebo plugin
(plugin code in the [upstream gazebo repository](https://bitbucket.org/osrf/gazebo/)).

## Sensors

### Template

   - ***description:*** short description of the wrapper.
   - ***status:***
       - **maintained**: no big issues, should work fine
       - **autotest**: the wrapper has automatic tests
       - **doxygen**: the wrapper implements doxygen comments
       - **needs-cleanup**: the code of the wrapper should be improved
       - **not-just-a-wrapper**: the code in this repo is not just a wrapper and
         contains simulation code. The simulation code should be migrated to
         the Gazebo repository.
   - ***gazebo plugin:*** gazebo upstream plugin used by the wrapper
   - ***example:*** example files that use the wrapper

### ROS wrappers for Gazebo upstream sensors

 * ***gazebo_ros_block_laser***
   - ***description:*** implements ray based sensors (lasers). Publishes
     sensors_msgs::PointCloud.
   - ***status:*** maintained
   - ***gazebo plugin:*** RayPlugin
   - ***example:*** gazebo_plugins/test/test_worlds/gazebo_ros_block_laser.world
              gazebo_plugins/test/test_worlds/gazebo_ros_trimesh_collision.world

 * ***gazebo_ros_bumper***
   - ***description:***  implements a contact sensor. Publishes
     gazebo_msgs::ContactsState.
   - ***status:*** maintained, needs-cleanup
   - ***gazebo plugin:***
   - ***example:*** gazebo_plugins/test/bumper_test/gazebo_ros_bumper.world:
   - test: gazebo_plugins/test/test_worlds/bumper_test.world:

 * ***gazebo_ros_camera***
   gazebo_ros_camera_utils
   - ***description:*** implements a camera. Publishes: sensor_msgs::Image, sensor_msgs::CameraInfo
   - ***status:*** maintained, dynamic-reconfigure, autotest
   - ***gazebo plugin:*** CameraPlugin, GazeboRosCameraUtils
   - ***example:*** gazebo_plugins/test/test_worlds/gazebo_ros_block_laser.world:
              gazebo_plugins/test/test_worlds/gazebo_ros_depth_camera.world:
              gazebo_plugins/test/test_worlds/gazebo_ros_camera.world:
              gazebo_plugins/test/test_worlds/gazebo_ros_trimesh_collision.world:
              gazebo_plugins/test/multi_robot_scenario/xacro/camera/camera.xacro:

 * ***gazebo_ros_f3d***
   - ***description:*** controller that fake (empty publisher) a 6 dof force sensor
     Publishes geometry_msgs::WrenchStamped
   - ***status:*** stub
   - ***gazebo plugin:*** ModelPlugin (generic)
   - ***example:*** --

 * ***gazebo_ros_force***
   - ***description:*** collects data from a ROS topic and applies wrench to a body accordingly.
   - ***status:*** maintained, doxygen
   - ***gazebo plugin:*** ModelPlugin (generic)
   - ***example:*** --

 * ***gazebo_ros_ft_sensor***
   - ***description:*** implements Force/Torque sensor.
     Publishes: geometry_msgs/WrenchStamped messages
   - ***status:*** maintained, doxygen
   - ***gazebo plugin:*** ModelPlugin (generic)
   - ***example:*** --

 * ***gazebo_ros_gpu_laser***
   - ***description:***  implements GPU laser based sensors.
     Publishes: sensor_msgs::LaserScan
   - ***status:*** maintained
   - ***gazebo plugin:*** GpuRayPlugin
   - ***example:*** gazebo_plugins/test/test_worlds/gazebo_ros_gpu_laser.world
              gazebo_plugins/test/multi_robot_scenario/xacro/laser/hokuyo_gpu.xacro

 * ***gazebo_ros_laser***
   - ***description:*** ROS wrapper for ray based sensors (lasers).
     Publishes sensor_msgs::LaserScan.
   - ***status:*** maintained
   - ***gazebo plugin:*** RayPlugin
   - ***example:*** test/test_worlds/gazebo_ros_laser.world
              test/test_worlds/test_lasers.world
              test/multi_robot_scenario/xacro/laser/hokuyo.xacro

 * ***gazebo_ros_multicamera***
   MultiCameraPlugin
   gazebo_ros_camera_utils
   - ***description:*** ROS wrapper for one or more synchronized cameras.
     Publishes: sensor_msgs::Image, sensor_msgs::CameraInfo
   - ***status:*** maintained, autotest
   - ***gazebo plugin:*** MultiCameraPlugin
   - ***example:*** --

 * ***gazebo_ros_openni_kinect***
   - ***description:*** ROS wrapper for depth camera sensors (Kinect like)
     Publishes: sensor_msgs::PointCloud2, sensor_msgs::Image
   - ***status:*** maintained
   - ***gazebo plugin:*** DepthCameraPlugin
   - ***example:*** --

 * ***gazebo_ros_prosilica***
   - ***description:*** plugin for simulating prosilica cameras in gazebo
     Publishes: sensor_msgs::Image, sensor_msgs::CameraInfo
   - ***status:*** maintained
   - ***gazebo plugin:*** > CameraPlugin > ModelPlugin (generic)
   - ***example:*** --

 * ***gazebo_ros_range***
   - ***description:*** simulate range sensors like infrared or ultrasounds
     Publishes: sensor_msgs/Range
   - ***status:*** maintained
   - ***gazebo plugin:*** RayPlugin
   - ***example:*** gazebo_plugins/test/test_worlds/gazebo_ros_range.world
              gazebo_ros/launch/range_world.launch



### ROS implementations for sensors (not recommended)

Although there is some code in this repo that implements simulations, this
should be an exception. Please checkout the CONTRIBUTION.md guide for more
details about which code should be submitted to this repository.

 * ***gazebo_ros_depth_camera***
   - ***description:*** implements depth camera based sensors. Publishes: sensor_msgs::Image,
     sensor_msgs::CameraInfo, sensors_msgs::PointCloud2
   - ***status:*** maintained, not-just-a-wrapper, autotest, needs-cleanup
   - ***gazebo plugin:*** DepthCameraPlugin
   - ***example:*** gazebo_plugins/test/test_worlds/gazebo_ros_depth_camera.world
              gazebo_plugins/test/test_worlds/gazebo_ros_trimesh_collision.world

 * ***gazebo_ros_diff_drive***
   - ***description:*** implements a diff drive base. Publishes: sensor_msgs::JointState,
     nav_msgs::Odometry
   - ***status:*** maintained, not-just-a-wrapper, rosparams, needs-cleanup
   - ***gazebo plugin:*** ModelPlugin (generic)
   - ***example:***  gazebo_plugins/test/multi_robot_scenario/xacro/p3dx/pioneer3dx_plugins.xacro
               gazebo_plugins/test/multi_robot_scenario/launch/pioneer3dx.gazebo.launch


## Other plugins (not related to sensors)

 * ***gazebo_ros_elevator***
   - ***description:*** implements an elevator
   - ***status:*** maintained
   - ***gazebo plugin:*** ElevatorPlugin
   - ***example:*** gazebo_plugins/test/test_worlds/elevator.world

 * ***gazebo_ros_hand_of_god***
   - ***description:*** Drives floating object around based on the location of a TF frame
   - ***status:*** maintained, doxygen
   - ***gazebo plugin:*** ModelPlugin (generic)
   - ***example:*** --

 * ***gazebo_ros_joint_pose_trajectory***
   - ***description:*** get a pose trajectory (trajectory_msgs::JointTrajectory) and execute it
   - ***status:*** maintained
   - ***gazebo_plugin:*** --
   - ***example:*** gazebo_plugins/test/pub_joint_trajectory_test.cpp

 * ***gazebo_ros_joint_state_publisher***
   - ***description:*** ROS plugin that publishes the state of a given set of joints at a given rate
     Publishes: sensors_msgs::JointState
   - ***status:*** maintained
   - ***gazebo_plugin:*** --
   - ***example:*** multi_robot_scenario/xacro/p3dx/pioneer3dx_plugins.xacro

 * ***gazebo_ros_p3d***
   - ***description:*** publish 3D position interface for ground truth
     Publishes: nav_msgs::Odometry
   - ***status:*** maintained
   - ***gazebo plugin:*** ModelPlugin (generic)
   - ***example:*** test/p3d_test/worlds/single_pendulum.world
              test/p3d_test/worlds/3_double_pendulums.world
              test2/large_models/smaller_large_model.urdf.xacro
              test2/large_models/large_model.urdf.xacro

 * ***gazebo_ros_planar_move***
   - ***description:*** simple model controller that uses a twist message to move a robot on the xy plane
     Publishes: nav_msgs::Odometry, tf
   - ***status:*** maintained
   - ***gazebo plugin:*** ModelPlugin (generic)
   - ***example:***

 * ***gazebo_ros_projector***
   - ***description:*** controller that controls texture projection into the world from a given body.
     Publish: msgs::Projector
   - ***status:*** maintained
   - ***gazebo plugin:*** ModelPlugin (generic)
   - ***example:*** -- (some information in the header file)

 * ***gazebo_ros_skid_steer_drive***
   - ***description:*** A skid steering drive plugin
     Publish: nav_msgs::Odometry
   - ***status:*** maintained
   - ***gazebo plugin:*** ModelPlugin (generic)
   - ***example:*** --

 * ***gazebo_ros_tricycle_drive***
   - ***description:*** a tricycle drive plugin for gazebo
     Publish: nav_msgs::Odometry, sensor_msgs::JointState
   - ***status:*** maintained, rosparams
   - ***gazebo plugin:*** ModelPlugin (generic)
   - ***example:*** gazebo_plugins/test/tricycle_drive/
              xacro, launch, world, rviz, etc.

 * ***gazebo_ros_triggered_camera / gazebo_ros_triggered_multicamera***
   - ***description:*** These camera sensors do not publish unless triggered. They have an
     additional topic (default name `image_trigger`) that subscribes to std_msgs/Empty messages
     and will publish a single update after being triggered. Its maximum update rate is currently
     set by the <update_rate> sdf tag in the <sensor> block.
     Publishes: sensor_msgs::Image, sensor_msgs::CameraInfo
   - ***status:*** maintained
   - ***example:*** gazebo_plugins/test/camera/triggered_camera.*

 * ***gazebo_ros_video***
   - ***description:*** Video plugin for displaying ROS image topics on Ogre textures
   - ***status:*** maintained
   - ***gazebo plugin:*** VisualPlugin
   - ***example:*** --


## List of deprecated plugins (to be removed)

 * ***camera_synchronizer***
 * ***vision_reconfigure***
   - ***description:***
   - ***status:*** needs-work, dead?
   - ***gazebo plugin:*** CameraSynchronizerConfig
   - ***example:***

 * ***gazebo_ros_imu***
   - ***description:*** implements an IMU sensor. Publishes: sensor_msgs::Imu
   - ***status:*** maintained, not-just-a-wrapper
   - ***gazebo plugin:*** ModelPlugin (generic)
   - ***example:*** --

