^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_ros_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.7 (2020-05-01)
------------------
* gazebo_ros_control: catch all pluginlib exceptions (`#1062 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1062>`_)
* Contributors: Max Schwarz

2.8.6 (2019-12-26)
------------------
* restrict Windows header namespace. (`#1023 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1023>`_)
* [Windows][melodic-devel] more Windows build break fix (`#975 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/975>`_)
  * Fix CMake install error for Windows build.
  * conditionally include <sys/time.h>
* Contributors: Sean Yen

2.8.5 (2019-06-04)
------------------
* use C++11 std sleep instead of usleep. (`#877 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/877>`_)
* Lower minimum cmake version (`#817 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/817>`_)
* Contributors: Paul Bovbel, Sean Yen [MSFT]

2.8.4 (2018-07-06)
------------------

2.8.3 (2018-06-04)
------------------
* Remove legacy in gazebo_ros_control for robotNamespace (`#709 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/709>`_)
  See pull request `#637 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/637>`_
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
* Don't ignore robotNamespace in gazebo_ros_control nodes (lunar-devel) (`#706 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/706>`_)
  This commit restores the intended behavior, i.e., the parameters will now read from <robot_name>/..., where <robot_name> is specified via the robotNamespace plugin parameter or the parent name.
* add physics type for dart with joint velocity interface (`#701 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/701>`_)
* Contributors: Jose Luis Rivero

2.7.4 (2018-02-12)
------------------
* Fix last gazebo8 warnings! (lunar-devel) (`#664 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/664>`_)
* Fix gazebo8 warnings part 7: retry `#642 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/642>`_ on lunar (`#660 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/660>`_)
* Contributors: Jose Luis Rivero, Steven Peters

2.7.3 (2017-12-11)
------------------
* Replace Events::Disconnect* with pointer reset (`#626 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/626>`_)
* Contributors: Jose Luis Rivero

2.7.2 (2017-05-21)
------------------
* Revert gazebo8 changes in Lunar and back to use gazebo7 (`#583 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/583>`_)
* Contributors: Jose Luis Rivero

2.7.1 (2017-04-28)
------------------
* Fixes for compilation and warnings in Lunar-devel  (`#573 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/573>`_)
  Multiple fixes for compilation and warnings coming from Gazebo8 and ignition-math3
* Less exciting console output (`#561 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/561>`_)
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (`#571 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/571>`_)
* Contributors: Dave Coleman, Jose Luis Rivero

2.5.12 (2017-04-25)
-------------------
* Fixed broken gazebo_ros_control tutorial link (`#566 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/566>`_)
* Contributors: Ian McMahon

2.5.11 (2017-04-18)
-------------------
* Change build system to set DEPEND on Gazebo/SDFormat (fix catkin warning)
  Added missing DEPEND clauses to catkin_package to fix gazebo catkin warning. Note that after the change problems could appear related to -lpthreads errors. This is an known issue related to catkin: https://github.com/ros/catkin/issues/856.
* Make gazebo_ros_control compatible with ros_control with respect to <hardwareInterface> tag (`#550 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/550>`_)
  * ros_control expects "<hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>", i.e. "hardware_interface/" prefix
  * add deprecation warning
  * improve warning
  * fix warning message fix
* Contributors: Andreas Bihlmaier, Dave Coleman

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
* Print name of joint with wrong interface
* Removed all trailing whitespace
* Change boost::shared_ptr to urdf::JointConstSharedPtr
* Contributors: Bence Magyar, Dave Coleman, Jochen Sprickerhof

2.5.8 (2016-12-06)
------------------

2.5.7 (2016-06-10)
------------------
* delete CATKIN_IGNORE in gazebo_ros_control (`#456 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/456>`_)
* Contributors: Jackie Kay, Jose Luis Rivero

2.5.3 (2016-04-11)
------------------

2.5.2 (2016-02-25)
------------------
* clean up merge from indigo-devel
* merging from indigo-devel
* Use Joint::SetParam for joint velocity motors
  Before gazebo5, Joint::SetVelocity and SetMaxForce
  were used to set joint velocity motors.
  The API has changed in gazebo5, to use Joint::SetParam
  instead.
  The functionality is still available through the SetParam API.
  cherry-picked from indigo-devel
  Add ifdefs to fix build with gazebo2
  It was broken by `#315 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/315>`_.
  Fixes `#321 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/321>`_.
* 2.4.9
* Generate changelog
* Import changes from jade-branch
* add missing dependencies
* Fix DefaultRobotHWSim puts robotNamespace twice
  DefaultRobotHWSim::initSim() member function uses both
  namespaced NodeHandle and robot_namespace string to create
  parameter names.
  For example,  if a robotNamespace is "rrbot",
  DefaultRobotHWSim tries to get parameters from following names:
  - /rrbot/rrbot/gazebo_ros_control/pid_gains/*
  - /rrbot/rrbot/joint_limits/*
  This commit change these names to:
  - /rrbot/gazebo_ros_control/pid_gains/*
  - /rrbot/joint_limits/*
* Add ifdefs to fix build with gazebo2
  It was broken by `#315 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/315>`_.
  Fixes `#321 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/321>`_.
* Use Joint::SetParam for joint velocity motors
  Before gazebo5, Joint::SetVelocity and SetMaxForce
  were used to set joint velocity motors.
  The API has changed in gazebo5, to use Joint::SetParam
  instead.
  The functionality is still available through the SetParam API.
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Contributors: Akiyoshi Ochiai, John Hsu, Jose Luis Rivero, Steven Peters, ipa-fxm

2.5.1 (2015-08-16)
------------------
* Fix DefaultRobotHWSim puts robotNamespace twice
  DefaultRobotHWSim::initSim() member function uses both
  namespaced NodeHandle and robot_namespace string to create
  parameter names.
  For example,  if a robotNamespace is "rrbot",
  DefaultRobotHWSim tries to get parameters from following names:
  - /rrbot/rrbot/gazebo_ros_control/pid_gains/*
  - /rrbot/rrbot/joint_limits/*
  This commit change these names to:
  - /rrbot/gazebo_ros_control/pid_gains/*
  - /rrbot/joint_limits/*
* Added a comment about the need of libgazebo5-dev in runtime
* Added elevator plugin
* Use c++11
* run_depend on libgazebo5-dev (`#323 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/323>`_)
  Declare the dependency.
  It can be fixed later if we don't want it.
* Contributors: Akiyoshi Ochiai, Jose Luis Rivero, Nate Koenig, Steven Peters

* Fix DefaultRobotHWSim puts robotNamespace twice
  DefaultRobotHWSim::initSim() member function uses both
  namespaced NodeHandle and robot_namespace string to create
  parameter names.
  For example,  if a robotNamespace is "rrbot",
  DefaultRobotHWSim tries to get parameters from following names:
  - /rrbot/rrbot/gazebo_ros_control/pid_gains/*
  - /rrbot/rrbot/joint_limits/*
  This commit change these names to:
  - /rrbot/gazebo_ros_control/pid_gains/*
  - /rrbot/joint_limits/*
* Added a comment about the need of libgazebo5-dev in runtime
* Added elevator plugin
* Use c++11
* run_depend on libgazebo5-dev
* Contributors: Akiyoshi Ochiai, Jose Luis Rivero, Nate Koenig, Steven Peters

2.5.0 (2015-04-30)
------------------
* run_depend on libgazebo5-dev instead of gazebo5
* Changed the rosdep key for gazebo to gazebo5, for Jade Gazebo5 will be used.
* Contributors: Steven Peters, William Woodall

2.4.9 (2015-08-16)
------------------
* Import changes from jade-branch
* add missing dependencies
* Add ifdefs to fix build with gazebo2
  It was broken by `#315 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/315>`_.
  Fixes `#321 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/321>`_.
* Use Joint::SetParam for joint velocity motors
  Before gazebo5, Joint::SetVelocity and SetMaxForce
  were used to set joint velocity motors.
  The API has changed in gazebo5, to use Joint::SetParam
  instead.
  The functionality is still available through the SetParam API.
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Contributors: Akiyoshi Ochiai, Jose Luis Rivero, Steven Peters, ipa-fxm

2.4.8 (2015-03-17)
------------------
* Merge pull request `#244 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/244>`_ from cottsay/control-urdf-fix
  gazebo_ros_control: add urdf to downstream catkin deps
* Added emergency stop support.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Jim Rothrock, Scott K Logan

2.4.7 (2014-12-15)
------------------
* move declaration for DefaultRobotHWSim to header file
* Contributors: ipa-fxm

2.4.6 (2014-09-01)
------------------
* Update default_robot_hw_sim.cpp
* Reduced changes
* Fix to work with gazebo3
* Fix build with gazebo4 and indigo
* Update package.xml
  Add new maintainer.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Jose Luis Rivero, Nate Koenig, hsu

2.4.5 (2014-08-18)
------------------
* Fix typo: GAZEBO_VERSION_MAJOR -> GAZEBO_MAJOR_VERSION
* Port fix_build branch for indigo-devel
  See pull request `#221 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/221>`_
* Contributors: Jose Luis Rivero, Steven Peters

2.4.4 (2014-07-18)
------------------
* Update package.xml
  Add new maintainer.
* Should fix build error for binary releases.
  See: http://www.ros.org/debbuild/indigo.html?q=gazebo_ros_control
* Updated package.xml
* gazebo_ros_control: default_robot_hw_sim:  Suppressing pid error message
  Depends on `ros-controls/control_toolbox#21 <https://github.com/ros-controls/control_toolbox/issues/21>`_
* Revert 4776545, as it belongs in indigo-devel.
* Fix repo names in package.xml's
* gazebo_ros_control: default_robot_hw_sim: Suppressing pid error message, depends on `ros-controls/control_toolbox#21 <https://github.com/ros-controls/control_toolbox/issues/21>`_
* gazebo_ros_control: Add dependency on angles
* gazebo_ros_control: Add build-time dependency on gazebo
  This fixes a regression caused by a889ef8b768861231a67b78781514d834f631b8e
* Contributors: Adolfo Rodriguez Tsouroukdissian, Alexander Bubeck, Dave Coleman, Jon Binney, Jonathan Bohren, Scott K Logan

2.4.3 (2014-05-12)
------------------
* Compatibility with Indigo's ros_control.
  Also fixes `#184 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/184>`_.
* Remove build-time dependency on gazebo_ros.
* Fix broken build due to wrong rosconsole macro use
* Contributors: Adolfo Rodriguez Tsouroukdissian

2.4.2 (2014-03-27)
------------------
* merging from hydro-devel
* bump patch version for indigo-devel to 2.4.1
* merging from indigo-devel after 2.3.4 release
* Merge branch 'hydro-devel' of github.com:ros-simulation/gazebo_ros_pkgs into indigo-devel
* "2.4.0"
* catkin_generate_changelog
* Contributors: John Hsu

2.4.1 (2013-11-13)
------------------

2.3.5 (2014-03-26)
------------------
* Removed some debugging code.
* joint->SetAngle() and joint->SetVelocity() are now used to control
  position-controlled joints and velocity-controlled joints that do not
  have PID gain values stored on the Parameter Server.
* Position-controlled and velocity-controlled joints now use PID controllers
  instead of calling SetAngle() or SetVelocity(). readSim() now longer calls
  angles::shortest_angular_distance() when a joint is prismatic.
  PLUGINLIB_EXPORT_CLASS is now used to register the plugin.
* gazebo_ros_control now depends on control_toolbox.
* Added support for the position hardware interface. Completed support for the
  velocity hardware interface.
* Removed the "support more hardware interfaces" line.
* Contributors: Jim Rothrock

2.3.4 (2013-11-13)
------------------
* rerelease because sdformat became libsdformat, but we also based change on 2.3.4 in hydro-devel.
* Merge pull request `#144 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/144>`_ from meyerj/fix-125
  Fixed `#125 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/125>`_: ``gazebo_ros_control``: controlPeriod greater than the simulation period causes unexpected results
* Merge pull request `#134 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/134>`_ from meyerj/gazebo-ros-control-use-model-nh
  ``gazebo_ros_control``: Use the model NodeHandle to get the ``robot_description`` parameter
* ``gazebo_ros_control``: added GazeboRosControlPlugin::Reset() method that resets the timestamps on world reset
* ``gazebo_ros_control``: call writeSim() for each Gazebo world update independent of the control period
* ``gazebo_ros_pkgs``: use GetMaxStepSize() for the Gazebo simulation period
* ``gazebo_ros_control``: use the model NodeHandle to get the ``robot_description`` parameter
* Add missing ``run_depend`` to urdf in ``gazebo_ros_control``
* Remove dependency to meta-package ``ros_controllers``

2.4.0 (2013-10-14)
------------------

2.3.3 (2013-10-10)
------------------
* Eliminated a joint_name variable and replaced it with `joint_names_[j]`.
  Modified some lines so that they fit in 100 columns. These changes were made
  in order to be consistent with the rest of the file.
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* joint_limits_interface is now used to enforce limits on effort-controlled
  joints.
* Added "joint_limits_interface" and "urdf" to the component list.
* Additional parameters are passed to `robot_hw_sim->initSim()`. These parameters
  are used by the joint limits interface.
* Added "joint_limits_interface" and "urdf" to the build dependency list.
* Added the robot_namespace and urdf_model parameters to `initSim()`.
* Added the urdf_string parameter to `parseTransmissionsFromURDF()`.

2.3.2 (2013-09-19)
------------------

2.3.1 (2013-08-27)
------------------
* Cleaned up template, fixes for header files
* Renamed plugin to match file name, tweaked CMakeLists
* Created a header file for the ros_control gazebo plugin

2.3.0 (2013-08-12)
------------------
* Renamed ros_control_plugin, updated documentation

2.2.1 (2013-07-29)
------------------

2.2.0 (2013-07-29)
------------------
* Standardized the way ROS nodes are initialized in gazebo plugins
* Remove find_package(SDF) from CMakeLists.txt
  It is sufficient to find gazebo, which will export the information
  about the SDFormat package.
* Merge branch 'hydro-devel' into tranmission_parsing
* Doc and debug update
* Merged hydro-devel
* Hid debug info
* Merged from Hydro-devel
* Merge branch 'hydro-devel' into tranmission_parsing
* Moved trasmission parsing to ros_control

2.1.5 (2013-07-18)
------------------

2.1.4 (2013-07-14)
------------------
* Fixed for Jenkins broken dependency on SDF in ros_control

2.1.3 (2013-07-13)
------------------

2.1.2 (2013-07-12)
------------------
* Cleaned up CMakeLists.txt for all gazebo_ros_pkgs
* 2.1.1

2.1.1 (2013-07-10 19:11)
------------------------
* Fixed errors and deprecation warnings from Gazebo 1.9 and the sdformat split
* making RobotHWSim::initSim pure virtual
* Cleaning up code
* Adding install targets

2.1.0 (2013-06-27)
------------------
* Made version match the rest of gazebo_ros_pkgs per bloom
* Added dependency on ros_controllers
* Clarifying language in readme
* Made default period Gazebo's period
* Made control period optional
* Tweaked README
* Added support for reading <tranmission> tags and other cleaning up
* Renamed RobotSim to RobotHWSim
* Renaming all gazebo_ros_control stuff to be in the same package
* Refactoring gazebo_ros_control packages into a single package, removing exampls (they will go elsewhere)
* updating readme for gazebo_ros_control
* Merging in gazebo_ros_control
* making gazebo_ros_control a metapackage
* Moving readme
* Merging readmes
* eating this
* Merging gazebo_ros_control and ros_control_gazebo

2.0.2 (2013-06-20)
------------------

2.0.1 (2013-06-19)
------------------

2.0.0 (2013-06-18)
------------------
