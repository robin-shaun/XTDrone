^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wamv_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2019-12-26)
------------------
* Switch to 0 0 1 in axis.
* Create links where the GPS and IMU sensors are attached.
* Contributors: Carlos Aguero

1.2.6 (2019-10-04)
------------------

1.2.5 (2019-09-19)
------------------

1.2.4 (2019-09-12)
------------------

1.2.3 (2019-09-12)
------------------

1.2.2 (2019-09-06)
------------------

1.2.1 (2019-09-05)
------------------
* minor cleanup + env flag + disable z
* Adding namespaces to the other thruster configurations.
* Namespace tweaks.
* Fix typo.
* Style changes.
* Merge
* Added commented code to wamv_xacro
* removed from wamv
* force vectors are correct; scaling added
* merged with master
* Contributors: Carlos Aguero, Rumman Waqar <rumman.waqar05@gmail.com>, Tyler Lum <tylergwlum@gmail.com>

1.2.0 (2019-08-19)
------------------
* Merge from topic_namespace_generation
* Update rviz configuration with new lidar topic
* lidar->lidars in topic names
* Merged default into topic_namespace_generation
* Alter launch file to fix tf frame error in navsat_transform_node
* Update rviz configuration to match namespaces
* Update localisation example launch file to support namespace.  The addition of a robot namespace also results in the creation of tf namespaces.  This patch upadtes the launch file to support this
* Modify xacro files to add further topic subnamespaces to sensor models
* Merged default into topic_namespace_generation
* fix catkin make install issues with meshes
* Merged in Fix-RVIZ-Mesh-Issue (pull request #158)
  Fix RVIZ Mesh Issue
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Fully functional solution, with urdf file modified when calling spawn_wamv.bash and giving proper model dirs to wamv_gazebo and wamv_description
* Fully functional solution, with model.config errors only when non_competition_mode:=false
* Change all to package://vrx_gazebo
* Rm fake vrx_gazebo/models and make it work, needs testing
* Change model names to No Model
* Move cameras into sensor namespace
  Add senosr namespace to p3d position feedback
* Remove extra slashes from topic names
* Merged default into topic_namespace_generation
* Fix namepsaces to work with robotNamespace parameter
* Merge changes with default branch
* Add sensor namepsace to planar lidar
  Remove gloabl namespace
* Add support for namespaces This includes thruster, camera and sensor namespaces, as well as a global namespace allowing multiple vehicles to be created without causing interference.
* Add v3d plugin - this publishes a vecotr based on the world frame velocity in Gazebo
  Update gps configuration to add gazebo gps and v3d plugins to standard configuration
* Add plugin for ROS interface to gazebo GPS sensor.
* Clean up model.config files
* Add fake models in vrx_gazebo, wamv_description, wamv_gazebo to avoid GAZEBO_MODEL_PATH errors
* fixed wamv_camera
* remove tab
* fix issue
* removed old debug msg
* added force visualization to wamv
* Contributors: Carlos Aguero, Carlos Agüero <cen.aguero@gmail.com>, Jonathan Wheare <jonathan.wheare@flinders.edu.au>, MarshallRawson, Rumman Waqar <rumman.waqar05@gmail.com>, Tyler Lum <tylergwlum@gmail.com>

1.1.2 (2019-07-10)
------------------
* lidar.xacro edited online with Bitbucket
* fixed issue
* cpu cases now included when configured by yaml
* Contributors: Marshall Rawson <marshallrawson@osrfoundation.org>, MarshallRawson

1.1.1 (2019-07-03)
------------------

1.1.0 (2019-07-01)
------------------
* wamv_gps.xacro edited online with Bitbucket
* Update camera resolution
* Using 16 beam lasers
* default urdf is now 32 beam lidar
* completed lidar specs
* ready for detailed lidar spec input
* added rviz config
* Merged default into issue#94-buoyancy
* merging with default - need to check wind
* Added allowences for post_Y and moved wamv_imu, wamv_gps default locations to be within compliance
* temporary branch for comparing with wave_visualization
* Merged in Issue#100-wind-plugin (pull request #106)
  Issue#100 wind plugin moved to world plugin
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Put required parameters together and make it obvious which are required
* Add <enableAngle> bool parameter that controls if angle is adjustable or not
* filled out the SensorCompliance. It is formatted by the sensors_compliance files
* Implement turnable thruster joint
* Basic implementation of angle adjustable thrusters, still need to test, add joints, and change visuals
* merged. expanded xacro capabilities
* changing the interface from timePeriod to frequency
* adding ROS API to probe for wind speed
* enabling the user to input only the angle for wind direction
* Make thruster config with yaml work without affecting use of sensor yaml config, still need to clean up
* Move engine.xacro to thrusters directory to allow for different types of thrusters
* Merged default into Issue#97-yaml-thruster-configuration
* Clarify link relative position calculation
* Implement varying length lidar pole
* Change post angle for right camera
* Vary post mass as length changes
* Fix camera seeing itself by increasing clip distance
* incremental(basic testing passed)
* Fix post color issue by removing <visual> tag name
* Add gazebo tag for color, still not working
* Update wamv_gazebo.urdf.xacro file to use thruster yaml file if given
* Initial testing of random seed with print statements
* Change from visual mesh to cylinder, but color not working. Stil showing white
* Define positioning variables for improved clarity
* Scale post length to better match camera height
* Simplify all transformations: base->post->arm->camera
* Tweaked comments.
* Simplify all transformations: base->post->arm->lidar
* Implement post_Y parameter that allows the post to be rotated in the yaw direction
* Redo sensor post to lidar joints to cleaner (x,y,z,r,p,y) coordinates
* Temporary test setting xyz of lidar, next need to change frames to simplify all of this
* updated readme, changed operation procedure, still not installed
* Add second adjustment link to perfectly match sensor and base frames
* Add adjustment link and joint to make the lidar frame better match base frame
* Add adjustment link and joint to make the camera frame match base frame
* Integrate sensor post to camera urdf, with height parameter
* Fix issue with lidar seeing itself and set default lidar angle downwards towards water
* Add mono_camera mesh to urdf file and onto WAM-V
* Merged default into Issue#86-add-3d-lidar-mesh
* Add sensor_post_arm.dae
* Add sensor post to 3d lidar on WAM-V, including height parameter
* Fix default 3d lidar pose
* Add CPU cases only in VRX configuration + remove redundant pose info
* Move boxes forward to prevent collision with gps
* Add 3D Lidar mesh and put it on WAM-V
* Add CPU case model to WAM-V
* Added script to interpret a yaml and auto generate appropriate xacro macro file while checking for compliance
* Added Batteries to vrx_gazebo/models(sdf format) and macro(urdf format) to place on wamv
* Lower mast.
* turning wind off to better test - tweaking waypoints in wayfinding task example
* Tweak names.
* Adding gps mesh, collisions and inertia.
* Tweaking positions and adding post and navigation course.
* Restoring cameras and laser visuals and creating demo.launch
* Sandisland texture, sensor meshes and extra objects.
* Implemented changed after PR is reviewed - V1
  Remove Ros dependency (regarding time)
  fixed typoes
  fixed wrong comments
  Exposed seed value to user
  Updated purpose of SDF params in the header file
  lines are now shorted than 80 chars
  added comments around explaining the calculations done
* made wind speed randomized
* Modify velodyne configuration to set intensity filtering
  Alter ocean laser retro to be filtered by the lidar sensor
* Setting wave parameters by hand in source for testing
* setting default wind to zero
* Issue #23: Coordinate the physics and visualization of the wave field
  1. Use the asv_wave_sim_gazebo_plugins package for wave field visualisation and depth calculation.
  2. Update the buoyancy and dynamics plugins for buoyancy calculations.
  3. Update sdf and xacro for models that require buoyancy.
  4. Replace the ocean model with ocean_waves in the sandisland world.
* Red placards and rearrange a bit the sensors.
* Port to VRX code using Gazebo9.
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Aguero <caguero@osrfoundation.org>, Carlos Agüero <cen.aguero@gmail.com>, Jonathan Wheare <jonathan.wheare@flinders.edu.au>, MarshallRawson, Rhys Mainwaring <rhys.mainwaring@me.com>, Rumman Waqar <rumman.waqar05@gmail.com>, Tyler Lum <tylergwlum@gmail.com>, Youssef Khaky <youssefkhaky@hotmail.com>, YoussefKhaky <youssefkhaky@hotmail.com.com>

1.0.1 (2019-03-01)
------------------
* changed rviz camera topic
* Contributors: Brian Bingham<briansbingham@gmail.com>

1.0.0 (2019-02-28)
------------------
* Merge from default.
* Merge from symbols_dock_part2
* Merge from default.
* Merged in vrx (pull request #68)
  Rename vmrc to vrx
  Approved-by: Brian Bingham <briansbingham@gmail.com>
* Custom tweaks
* More leftovers.
* Rename vmrc to vrx.
* assembling pieces for stationkeeping
* Merged in urdf_easy (pull request #62)
  Simplify urdf
  Approved-by: Brian Bingham <briansbingham@gmail.com>
* Simplify urdf files.
* Locking the WAM-V conditionally.
* Playing with locking and releasing.
* Changed from buoyancy calculation method
* Decrease sensor noise to more clearly allow debugging of the simulation.
* Add the pinger plugin to the wamv_gazebo package.
  The wamv_gazebo_sensors.urdf file has been modified to add support for the pinger plugin.
* add missing dependencies
* Create perception.launch and lock the WAM-V.
* removing static tags so vessel is freee to move
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Aguero <caguero@osrfoundation.org>, Carlos Agüero <cen.aguero@gmail.com>, Jonathan Wheare <jonathan.wheare@flinders.edu.au>, chapulina <burajiru.no.chapulina@gmail.com>

0.3.2 (2018-10-08)
------------------
* Include jrivero as maintainer of the ROS packages
* Contributors: Jose Luis Rivero <jrivero@osrfoundation.org>

0.3.1 (2018-10-05)
------------------

0.3.0 (2018-09-28)
------------------
* Tweak
* vrx metapackage and spring cleaning.
* Static model and fog.
* trying to get wamv to be static using a fixed joint
* Merge from default.
* reverting example rviz config back to original to be consistent with existing tutorial
* adding launch/config files for running the example
* adding examples to the sensors tutorial for the T and X propulsion configuration
* Create a standard sensor configuration for VRX.
* Merged in 3dlaser (pull request #41)
  Add 3D laser xacro
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Merge from default.
* Merged in holonomic-example-refactored (pull request #40)
  Holonomic example refactored
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Add 3D laser xacro
* Refactor thruster layout customization
* Enable on/off arguments for sensors xacro
* Fix multibeam laser xacro
* adding examples for T and X thruster configurations - accessible as args to sandisland.launch. Prototype - too much redundancy in the various urdf.xacro file hierarchy, but functional.
* Tabs -> spaces
* Initial style pass
* props now spinning, removed old method of thrust implementation, removed custome UsvDrive message
* working prototype - next remove old method
* increment - builds, but need to go home
* Add changelog.
* Merge from default
* Removing superfluous SDF for thrust
* More tweaks.
* Merge from default.
* Merged in sensor-examples (pull request #12)
  Add sensor macros and example
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Add multibeam to example sensor urdf
* Add simple visuals for sensors
* Move multibream -> multibeam
* Remove unneeded robot_description param from localization_example.launch
* Add optical frame for proper camera visualization
* Install config/launch files
* Merge default into sensor-examples
* Simplify wamv_gazebo macros
* Simplify xacro macros
* Refactor wind plugin.
* Split the wamv xacro file.
* More modular model with spinning propellers.
* Add example rviz config/launch
* Tweak
* Tweak
* Add sensor macros and example localization config
* Fix issues after wamv_gazebo migration
* Boostrap wamv_gazebo
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Agüero <caguero@osrfoundation.org>, Carlos Agüero <cen.aguero@gmail.com>, Kevin Allen <kallen@osrfoundation.org>
