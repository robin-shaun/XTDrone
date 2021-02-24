^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package wamv_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2019-12-26)
------------------
* Move rudder farther back
* Add sail and rudder link to wamv base, as well as turn post to wind direction arrow
* Add sail link and joint
* Contributors: Tyler Lum <tylergwlum@gmail.com>

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
* Merge
* merged with master
* Contributors: Rumman Waqar <rumman.waqar05@gmail.com>, Tyler Lum <tylergwlum@gmail.com>

1.2.0 (2019-08-19)
------------------
* Merge from topic_namespace_generation
* fix catkin make install issues with meshes
* Merged in Fix-RVIZ-Mesh-Issue (pull request #158)
  Fix RVIZ Mesh Issue
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Fully functional solution, with urdf file modified when calling spawn_wamv.bash and giving proper model dirs to wamv_gazebo and wamv_description
* Fully functional solution, with model.config errors only when non_competition_mode:=false
* Change all to package://vrx_gazebo
* Rm fake vrx_gazebo/models and make it work, needs testing
* Change model names to No Model
* Add sensor namepsace to planar lidar
  Remove gloabl namespace
* Add support for namespaces This includes thruster, camera and sensor namespaces, as well as a global namespace allowing multiple vehicles to be created without causing interference.
* Merged default into gps_plugin
* Clean up model.config files
* Add fake models in vrx_gazebo, wamv_description, wamv_gazebo to avoid GAZEBO_MODEL_PATH errors
* Contributors: Carlos Aguero, Carlos Agüero <cen.aguero@gmail.com>, Jonathan Wheare <jonathan.wheare@flinders.edu.au>, MarshallRawson, Tyler Lum <tylergwlum@gmail.com>

1.1.2 (2019-07-10)
------------------

1.1.1 (2019-07-03)
------------------

1.1.0 (2019-07-01)
------------------
* added dummy link for robot state publisher
* functional. no recording
* linting
* Merged default into issue#94-buoyancy
* merge
* merging default
* Merge from default, conflicts and style.
* Refix engine.xacro to have revolute joint
* Remove old engine.xacro file
* Merged in Issue#90_YAML_world_genreation (pull request #102)
  Issue#90 YAML world generation
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* merge for api update
* Implement turnable thruster joint
* Remove unnecessary files
* Add namespace to thruster config parameters and look in thrusters directory for valid engine types
* merge
* merge
* incremental
* incremental
* updated readme, changed operation procedure, still not installed
* fixed styling problems with flake8, updated readme
* changed directory, added launch file support
* incremental, now supports macros with no parameters
* Change cpu case collision box from 1 box to 2 boxes
* Move boxes forward to prevent collision with gps
* Tweak indentation.
* added a variance function parameter and fixed some bugs
* incremental
* incremental
* Fix formatting (tab->spaces, etc.)
* Add CPU case model to WAM-V
* incremental
* incremental
* Added script to interpret a yaml and auto generate appropriate xacro macro file while checking for compliance
* Added Batteries to vrx_gazebo/models(sdf format) and macro(urdf format) to place on wamv
* Sandisland texture, sensor meshes and extra objects.
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Agüero <cen.aguero@gmail.com>, MarshallRawson, Rumman Waqar <rumman.waqar05@gmail.com>, Tyler Lum <tylergwlum@gmail.com>

1.0.1 (2019-03-01)
------------------
* Merged in wamv_meshes_meters (pull request #75)
  Fix issue #66
  Approved-by: Brian Bingham <briansbingham@gmail.com>
* Change mesh units to meters.
* Contributors: Carlos Aguero, Carlos Agüero <cen.aguero@gmail.com>

1.0.0 (2019-02-28)
------------------
* Rename vmrc to vrx.
* removing static tags so vessel is freee to move
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero

0.3.2 (2018-10-08)
------------------
* Include jrivero as maintainer of the ROS packages
* Contributors: Jose Luis Rivero <jrivero@osrfoundation.org>

0.3.1 (2018-10-05)
------------------

0.3.0 (2018-09-28)
------------------
* vrx metapackage and spring cleaning.
* Static model and fog.
* Merge from default.
* Merged in holonomic-example-refactored (pull request #40)
  Holonomic example refactored
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Refactor thruster layout customization
* adding blank world for photo shoot of propulsion
* adding examples for T and X thruster configurations - accessible as args to sandisland.launch. Prototype - too much redundancy in the various urdf.xacro file hierarchy, but functional.
* Integrate the placards into the docks.
* Install config/launch files
* Remove references to the Gazebo 8 wind plugin.
* Do not generate anything from engine.xacro.
* Restore wind
* Split the wamv xacro file.
* More modular model with spinning propellers.
* Merged in packages (pull request #4)
  Packages
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
* Remove unused properties.urdf
* Remove unused PROPELLER.dae
* Merged in kevin-refactor (pull request #3)
  Various cleanups / refactors
  Approved-by: Carlos Agüero <cen.aguero@gmail.com>
  Approved-by: Kevin Allen <kallen@osrfoundation.org>
* Remove unused spreadsheets and thrust_curve_fit program
* Remove autogenerated files
* Build xacro files as install targets
* Remove platform specific features from wamv base
* Delete unused/broken launch files in wamv_description
* Delete unused blender param files
* Merge from default
* Simplified collisions
* Simplified collisions
* Small cleanup of old comments.
* Updated WAM-V model.
* Merge from waves/master usv_gazebo_plugins.
* Initial version of the code.
* Contributors: Brian Bingham <briansbingham@gmail.com>, Carlos Aguero, Carlos Agüero <caguero@osrfoundation.org>, Carlos Agüero <cen.aguero@gmail.com>, Kevin Allen <kallen@osrfoundation.org>
