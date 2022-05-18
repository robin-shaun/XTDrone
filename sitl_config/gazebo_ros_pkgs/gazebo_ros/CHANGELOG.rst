^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.7 (2020-05-01)
------------------
* add node required to empty world (`#1078 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1078>`_)
  Signed-off-by: Mabel Zhang <mabel@openrobotics.org>
* Contributors: Mabel Zhang

2.8.6 (2019-12-26)
------------------
* ROS API: remove unhelpful error in GetWorldProperties call (`#747 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/747>`_)
* Create reconfigure thread only if network enabled (`#919 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/919>`_)
  This thread was blocked in client.waitForExistance because the services
  it depends on are only created if `enable_ros_network` is true. This in
  turn blocked gazebo from being shut down.
  Signed-off-by: Shane Loretz <sloretz@osrfoundation.org>
* spawn_model: Fix urlparse imports for Python 3
* spawn_model: Ensure that "model_xml" is a string, required for Python 3
* catkin_find gazebo plugin from bin folder. (`#993 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/993>`_)
* [Windows][melodic-devel] more Windows build break fix (`#975 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/975>`_)
  * Fix CMake install error for Windows build.
  * conditionally include <sys/time.h>
* provide Windows implemenation for setenv. (`#879 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/879>`_)
* implement basic gazebo scripts to support launch file on Windows build. (`#880 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/880>`_)
* Contributors: Kartik Mohta, Kevin Allen, Sean Yen, Shane Loretz

2.8.5 (2019-06-04)
------------------
* Add output arg to launch files, plus some small fixes (melodic) (`#907 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/907>`_)
  * Add output arg to empty_world
  * add output arg to elevator_world
  * add output arg to range_world
  * don't set use_sim_time in range_world
  Instead parse it to empty world, where it will be set.
  * add xml prolog to all launch files
  * Remove unnecessary arg in range_world.launch
* use C++11 std sleep instead of usleep. (`#877 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/877>`_)
* fix issue `#198 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/198>`_ (`#825 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/825>`_)
* Lower minimum cmake version (`#817 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/817>`_)
* Contributors: Matthijs van der Burgh, Paul Bovbel, Sean Yen [MSFT], Steven Peters

2.8.4 (2018-07-06)
------------------
* Refactor spawn_model script
  * more robust -package_to_model implementation (issue #449)
  * add stdin as source option
  * parse arguments with argparse
  * remove deprecated/unused -gazebo and -trimesh options
* Fix physics reconfigure within namespace (issue #507)
* Contributors: Kevin Allen, Steven Peters

2.8.3 (2018-06-04)
------------------
* Use generic SIGINT parameter in kill command for gazebo script (melodic-devel) (`#724 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/724>`_)
  * Use generic SIGINT parameter in kill command for gazebo script
  * redirect to kill command to std_err
* Contributors: Jose Luis Rivero

2.8.2 (2018-05-09)
------------------
* Fix the build on Ubuntu Artful. (`#715 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/715>`_)
  Artful has some bugs in its cmake files for Simbody that
  cause it to fail the build.  If we are on artful, remove
  the problematic entries.
  Signed-off-by: Chris Lalancette <clalancette@openrobotics.org>
* Contributors: Chris Lalancette

2.8.1 (2018-05-05)
------------------
* Parameter to disable ROS network interaction from/to Gazebo (lunar-devel) (`#704 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/704>`_)
* Load the libgazebo_ros_api_plugin when starting gzclient so that the ROS event loop will turn over, which is required when you have a client-side Gazebo plugin that uses ROS. (`#676 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/676>`_)
* Pass verbose argument to gzclient (`#677 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/677>`_)
* strip comments from parsed urdf (`#698 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/698>`_)
  Remove comments from urdf before trying to find packages. Otherwise non-existant packages will produce a fatal error, even though they are not used.
* Contributors: Jose Luis Rivero

2.7.4 (2018-02-12)
------------------
* Fix last gazebo8 warnings! (lunar-devel) (`#664 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/664>`_)
* Fix for relative frame errors (lunar-devel) (`#663 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/663>`_)
* Fix gazebo8 warnings part 7: retry `#642 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/642>`_ on lunar (`#660 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/660>`_)
* Fix gazebo8 warnings part 10: ifdefs for GetModel, GetEntity, Light (lunar-devel) (`#657 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/657>`_)
* gazebo8 warnings: ifdefs for Get.*Vel() (`#655 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/655>`_)
* [gazebo_ros] don't overwrite parameter "use_sim_time" (lunar-devel) (`#607 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/607>`_)
* Fix gazebo8 warnings part 8: ifdef's for GetWorldPose (lunar-devel) (`#652 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/652>`_)
* Prevents GAZEBO_MODEL_DATABASE_URI from being overwritten (`#649 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/649>`_)
* for gazebo8+, call functions without Get (`#640 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/640>`_)
* Contributors: Jose Luis Rivero, Steven Peters

2.7.3 (2017-12-11)
------------------
* gazebo_ros_api_plugin: improve plugin xml parsing (`#627 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/627>`_)
* Fix gazebo8 warnings part 5: ignition math in gazebo_ros (lunar-devel) (`#636 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/636>`_)
* Fix gazebo8 warnings part 4: convert remaining local variables in plugins to ign-math (lunar-devel) (`#634 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/634>`_)
* gazebo_ros: fix support for python3 (`#629 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/629>`_)
* Replace Events::Disconnect* with pointer reset (`#626 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/626>`_)
* Install spawn_model using catkin_install_python (`#624 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/624>`_)
* Quote arguments to echo in libcommon.sh (`#591 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/591>`_)
* Contributors: Jose Luis Rivero

2.7.2 (2017-05-21)
------------------
* Revert gazebo8 changes in Lunar and back to use gazebo7 (`#583 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/583>`_)
* Contributors: Jose Luis Rivero

2.7.1 (2017-04-28)
------------------
* Fixes for compilation and warnings in Lunar-devel  (`#573 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/573>`_)
  Multiple fixes for compilation and warnings coming from Gazebo8 and ignition-math3
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (`#571 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/571>`_)
* Contributors: Jose Luis Rivero

2.5.12 (2017-04-25)
-------------------

2.5.11 (2017-04-18)
-------------------
* Changed the spawn model methods to spawn also lights. (`#511 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/511>`_)
* Change build system to set DEPEND on Gazebo/SDFormat (fix catkin warning)
  Added missing DEPEND clauses to catkin_package to fix gazebo catkin warning.
  Note that after the change problems could appear related to -lpthreads
  errors. This is an known issue related to catkin:
  https://github.com/ros/catkin/issues/856.
* Use correct logerr method (`#557 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/557>`_)
* Contributors: Alessandro Ambrosano, Dave Coleman, Gary Servin

2.5.10 (2017-03-03)
-------------------
* Revert catkin warnings to fix regressions (problems with catkin -lpthreads errors)
  For reference and reasons, please check:
  https://discourse.ros.org/t/need-to-sync-new-release-of-rqt-topic-indigo-jade-kinetic/1410/4
  * Revert "Fix gazebo catkin warning, cleanup CMakeLists (`#537 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/537>`_)"
  This reverts commit 5a0305fcb97864b66bc2e587fc0564435b4f2034.
  * Revert "Fix gazebo and sdformat catkin warnings"
  This reverts commit 11f95d25dcd32faccd2401d45c722f7794c7542c.
* Contributors: Jose Luis Rivero

2.5.9 (2017-02-20)
------------------
* Fix gazebo catkin warning, cleanup CMakeLists (`#537 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/537>`_)
* Namespace console output (`#543 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/543>`_)
* Removed all trailing whitespace
* Contributors: Dave Coleman

2.5.8 (2016-12-06)
------------------
* Workaround to support gazebo and ROS arguments in the command line
* Fix ROS remapping by reverting "Remove ROS remapping arguments from gazebo_ros launch scripts.
* Fixed getlinkstate service's angular velocity return
* Honor GAZEBO_MASTER_URI in gzserver and gzclient
* Contributors: Jared, Jon Binney, Jordan Liviero, Jose Luis Rivero, Martin Pecka

2.5.7 (2016-06-10)
------------------

2.5.6 (2016-04-28)
------------------
* Remove deprecated spawn_gazebo_model service
* Contributors: Steven Peters

2.5.5 (2016-04-27)
------------------
* merge indigo, jade to kinetic-devel
* Upgrade to gazebo 7 and remove deprecated driver_base dependency
  * Upgrade to gazebo 7 and remove deprecated driver_base dependency
  * disable gazebo_ros_control until dependencies are met
  * Remove stray backslash
* spawn_model: adding -b option to bond to the model and delete it on sigint
* Update maintainer for Kinetic release
* Allow respawning gazebo node.
* Contributors: Hugo Boyer, Isaac IY Saito, Jackie Kay, Jonathan Bohren, Jose Luis Rivero, Steven Peters

2.5.3 (2016-04-11)
------------------
* Include binary in runtime
* Remove ROS remapping arguments from gazebo_ros launch scripts.
* Contributors: Jose Luis Rivero, Martin Pecka

2.5.2 (2016-02-25)
------------------
* merging from indigo-devel
* Merge pull request `#302 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/302>`_ from maxbader/jade-devel-GetModelState
  Header for GetModelState service request for jade-devel
* Fix invalid signal name on OS X
  scripts/gazebo: line 30: kill: SIGINT: invalid signal specification
* Fix invalid signal name on OS X
  scripts/gazebo: line 30: kill: SIGINT: invalid signal specification
* Restart package resolving from last position, do not start all over.
* 2.4.9
* Generate changelog
* Import changes from jade-branch
* Add range world and launch file
* fix crash
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* GetModelState modification for jade
* Contributors: Bence Magyar, Boris Gromov, Guillaume Walck, Ian Chen, John Hsu, Jose Luis Rivero, Markus Bader, Steven Peters, hsu

2.5.1 (2015-08-16)
------------------
* Port of Pal Robotics range sensor plugin to Jade
* Added a comment about the need of libgazebo5-dev in runtime
* Added missing files
* Added elevator plugin
* Use c++11
* run_depend on libgazebo5-dev (`#323 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/323>`_)
  Declare the dependency.
  It can be fixed later if we don't want it.
* Contributors: Jose Luis Rivero, Nate Koenig, Steven Peters

* Port of Pal Robotics range sensor plugin to Jade
* Added a comment about the need of libgazebo5-dev in runtime
* Added missing files
* Added elevator plugin
* Use c++11
* run_depend on libgazebo5-dev
* Contributors: Jose Luis Rivero, Nate Koenig, Steven Peters

2.5.0 (2015-04-30)
------------------
* run_depend on libgazebo5-dev instead of gazebo5
* Changed the rosdep key for gazebo to gazebo5, for Jade Gazebo5 will be used.
* Contributors: Steven Peters, William Woodall

2.4.10 (2016-02-25)
-------------------
* Fix invalid signal name on OS X
  scripts/gazebo: line 30: kill: SIGINT: invalid signal specification
* Restart package resolving from last position, do not start all over.
* Contributors: Boris Gromov, Guillaume Walck

2.4.9 (2015-08-16)
------------------
* Import changes from jade-branch
* Add range world and launch file
* fix crash
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Contributors: Bence Magyar, Ian Chen, Jose Luis Rivero, Steven Peters

2.4.8 (2015-03-17)
------------------
* Specify physics engine in args to empty_world.launch
* Contributors: Steven Peters

2.4.7 (2014-12-15)
------------------
* temporary hack to **fix** the -J joint position option (issue `#93 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/93>`_), sleeping for 1 second to avoid race condition. this branch should only be used for debugging, merge only as a last resort.
* Fixing set model state method and test
* Extended the fix for `#246 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/246>`_ also to debug, gazebo, gzclient and perf scripts.
* Update Gazebo/ROS tutorial URL
* [gazebo_ros] Fix for `#246 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/246>`_
  Fixing issue `#246 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/246>`_ in gzserver.
* Fixing handling of non-world frame velocities in setModelState.
* update headers to apache 2.0 license
* update headers to apache 2.0 license
* Contributors: John Hsu, Jose Luis Rivero, Martin Pecka, Tom Moore, ayrton04

2.4.6 (2014-09-01)
------------------
* Merge pull request `#232 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/232>`_ from ros-simulation/fix_get_physics_properties_non_ode
  Fix get physics properties non ode
* Merge pull request `#183 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/183>`_ from ros-simulation/issue_182
  Fix STL iterator errors, misc. cppcheck (`#182 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/182>`_)
* check physics engine type before calling set_physics_properties and get_physics_properteis
* check physics engine type before calling set_physics_properties and get_physics_properteis
* Fixes for calling GetParam() with different physic engines.
* 2.3.6
* Update changelogs for the upcoming release
* Fixed boost any cast
* Removed a few warnings
* Update for hydro + gazebo 1.9
* Fix build with gazebo4 and indigo
* Fix STL iterator errors, misc. cppcheck (`#182 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/182>`_)
  There were some errors in STL iterators.
  Initialized values of member variables in constructor.
  Removed an unused variable (model_name).
* Contributors: Carlos Aguero, John Hsu, Jose Luis Rivero, Nate Koenig, Steven Peters, hsu, osrf

2.4.5 (2014-08-18)
------------------
* Port fix_build branch for indigo-devel
  See pull request `#221 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/221>`_
* Contributors: Jose Luis Rivero

2.4.4 (2014-07-18)
------------------
* Fix repo names in package.xml's
* fix issue `#198 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/198>`_
  Operator ``==`` is not recognized by sh scripts.
* Add verbose parameter
  Add verbose parameter for --verbose gazebo flag
* added osx support for gazebo start scripts
* Contributors: Arn-O, Jon Binney, Markus Achtelik, Vincenzo Comito

2.4.3 (2014-05-12)
------------------
* added osx support for gazebo start scripts
* Remove gazebo_ros dependency on gazebo_plugins
* Contributors: Markus Achtelik, Steven Peters

2.4.2 (2014-03-27)
------------------
* merging from hydro-devel
* bump patch version for indigo-devel to 2.4.1
* merging from indigo-devel after 2.3.4 release
* "2.4.0"
* catkin_generate_changelog
* Contributors: John Hsu

2.4.1 (2013-11-13)
------------------

2.3.5 (2014-03-26)
------------------
* gazebo_ros: [less-than-minor] fix newlines
* gazebo_ros: remove assignment to self
  If this is needed for any twisted reason, it should be made clear
  anyway. Assuming this line is harmless and removing it because it
  generates cppcheck warnings.
* Contributors: Paul Mathieu

2.3.4 (2013-11-13)
------------------
* rerelease because sdformat became libsdformat, but we also based change on 2.3.4 in hydro-devel.
* remove debug statement
* fix sdf spawn with initial pose
* fix sdf spawn with initial pose
* Merge branch 'hydro-devel' into ``spawn_model_pose_fix``
* fix indentation
* Merge pull request `#142 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/142>`_ from hsu/hydro-devel
  fix issue `#38 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/38>`_, gui segfault on model deletion
* Merge pull request `#140 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/140>`_ from ``v4hn/spawn_model_sleep``
  replace time.sleep by rospy.Rate.sleep
* fix spawn initial pose.  When model has a non-zero initial pose and user specified initial model spawn pose, add the two.
* fix issue `#38 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/38>`_, gui segfault on model deletion by removing an obsolete call to set selected object state to "normal".
* replace time.sleep by rospy.Rate.sleep
  time was not even imported, so I don't know
  why this could ever have worked...
* Add time import
  When using the -wait option the script fails because is missing the time import
* Use pre-increment for iterators
* Fix iterator erase() problems

2.4.0 (2013-10-14)
------------------

2.3.3 (2013-10-10)
------------------
* Cleaned up unnecessary debug output that was recently added
* Fixed issue where ``catkin_find`` returns more than one library if it is installed from both source and debian

2.3.2 (2013-09-19)
------------------
* Make gazebo includes use full path
  In the next release of gazebo, it will be required to use the
  full path for include files. For example,
  `include <physics/physics.hh>` will not be valid
  `include <gazebo/physics/physics.hh>` must be done instead.
* update gazebo includes
* Fixed a minor typo in spawn_model error message when `-model` not specified

2.3.1 (2013-08-27)
------------------
* Cleaned up template, fixes for header files

2.3.0 (2013-08-12)
------------------
* gazebo_ros: fixed missing dependency on TinyXML
* gazebo_plugins: replace deprecated boost function
  This is related to `this gazebo issue <https://bitbucket.org/osrf/gazebo/issue/581/boost-shared_-_cast-are-deprecated-removed>`_

2.2.1 (2013-07-29)
------------------

2.2.0 (2013-07-29)
------------------
* Switched to pcl_conversions
* Remove find_package(SDF) from CMakeLists.txt
  It is sufficient to find gazebo, which will export the information
  about the SDFormat package.

2.1.5 (2013-07-18)
------------------
* gazebo_ros: fixed variable names in gazebo_ros_paths_plugin

2.1.4 (2013-07-14)
------------------

2.1.3 (2013-07-13)
------------------

2.1.2 (2013-07-12)
------------------
* Added author
* Tweak to make SDFConfig.cmake
* Cleaned up CMakeLists.txt for all gazebo_ros_pkgs
* Cleaned up gazebo_ros_paths_plugin
* 2.1.1

2.1.1 (2013-07-10 19:11)
------------------------
* Merge branch 'hydro-devel' of github.com:ros-simulation/gazebo_ros_pkgs into hydro-devel
* Reduced number of debug msgs
* Fixed physics dynamic reconfigure namespace
* gazebo_ros_api_plugin: set `plugin_loaded_` flag to true in
  GazeboRosApiPlugin::Load() function
* Actually we need `__init__.py`
* Cleaning up code
* Moved gazebo_interface.py from gazebo/ folder to gazebo_ros/ folder
* Removed searching for plugins under 'gazebo' pkg because of rospack warnings
* Minor print modification
* Added dependency to prevent missing msg header, cleaned up CMakeLists

2.1.0 (2013-06-27)
------------------
* gazebo_ros: added deprecated warning for packages that use gazebo as
  package name for exported paths
* Hiding some debug info
* gazebo_ros: use rosrun in debug script, as rospack find gazebo_ros returns the wrong path in install space
* Hide Model XML debut output to console
* gazebo_ros_api_plugin.h is no longer exposed in the include folder
* Added args to launch files, documentation
* Merge pull request `#28 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/28>`_ from osrf/no_roscore_handling
  Better handling of gazebo_ros run when no roscore started
* gazebo_ros: also support gazebo instead of gazebo_ros as package name for plugin_path, gazebo_model_path or gazebo_media_path exports
* gazebo_plugins/gazebo_ros: fixed install directories for include files and gazebo scripts
* changed comment location
* added block comments for walkChildAddRobotNamespace
* SDF and URDF now set robotNamespace for plugins
* Better handling of gazebo_ros run when no roscore started

2.0.2 (2013-06-20)
------------------
* Added Gazebo dependency
* changed the final kill to send a SIGINT and ensure only the last background process is killed.
* modified script to work in bash correctly (tested on ubuntu 12.04 LTS)

2.0.1 (2013-06-19)
------------------
* Incremented version to 2.0.1
* Fixed circular dependency, removed deprecated pkgs since its a stand alone pkg
* Shortened line lengths of function headers

2.0.0 (2013-06-18)
------------------
* Changed version to 2.0.0 based on gazebo_simulator being 1.0.0
* Updated package.xml files for ros.org documentation purposes
* Combined updateSDFModelPose and updateSDFName, added ability to spawn SDFs from model database, updates SDF version to lastest in parts of code, updated the tests
* Renamed Gazebo model to SDF model, added ability to spawn from online database
* Fixed really obvious error checking bug
* Deprecated -gazebo arg in favor of -sdf tag
* Reordered services and messages to be organized and reflect documentation. No code change
* Cleaned up file, addded debug info
* Merged changes from Atlas ROS plugins, cleaned up headers
* Small fixes per ffurrer's code review
* Deprecated warnings fixes
* Cleaned up comment blocks - removed from .cpp and added to .h
* Merged branches and more small cleanups
* Small compile error fix
* Standardized function and variable naming convention, cleaned up function comments
* Reduced debug output and refresh frequency of robot spawner
* Converted all non-Gazebo pointers to boost shared_ptrs
* Removed old Gazebo XML handling functions - has been replaced by SDF, various code cleanup
* Removed the physics reconfigure node handle, switched to async ROS spinner, reduced required while loops
* Fixed shutdown segfault, renamed `rosnode_` to `nh_`, made all member variables have `_` at end, formatted functions
* Added small comment
* adding install for gazebo_ros launchfiles
* Formatted files to be double space indent per ROS standards
* Started fixing thread issues
* Fixing install script names and adding gzserver and gdbrun to install command
* Fixed deprecated warnings, auto formatted file
* Cleaned up status messages
* Added -h -help --help arguemnts to spawn_model
* Removed broken worlds
* Removed deprecated namespace argument
* Using pkg-config to find the script installation path.
  Corrected a bash typo with client_final variable in gazebo script.
* Cleaning up world files
* Deprecated fix
* Moved from gazebo_worlds
* Cleaning up launch files
* Moved from gazebo_worlds
* Fixing renaming errors
* Updated launch and world files and moved to gazebo_ros
* Combined gzclient and gzserver
* Added finished loading msg
* All packages building in Groovy/Catkin
* Imported from bitbucket.org
