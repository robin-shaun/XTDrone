^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.9.2 (2021-04-21)
------------------
* [Noetic] Bridge to republish PerformanceMetrics in ROS (`#1145 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1145>`_)
  Co-authored-by: Ian Chen <ichen@osrfoundation.org>
* Contributors: Alejandro Hernández Cordero

2.9.1 (2020-05-20)
------------------

2.9.0 (2020-05-19)
------------------
* Bump CMake version to avoid CMP0048 warning (`#1066 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1066>`_)
* add additional light options to 'set_light_properties' service (`#874 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/874>`_)
  The optional 'Light' properties 'cast_shadows', 'specular', 'direction',
  and 'pose' are not optional any more. These properties are now set via the
  corresponding fields in the ROS message. By default, this will be 0.
  https://github.com/ros-simulation/gazebo_ros_pkgs/pull/874
* Contributors: Alejandro Hernández Cordero, Christian Rauch

2.8.5 (2019-06-04)
------------------

2.8.4 (2018-07-06)
------------------
* Correct documentation on SetModelConfiguration.srv
* Contributors: Kevin Allen

2.8.3 (2018-06-04)
------------------

2.8.2 (2018-05-09)
------------------

2.8.1 (2018-05-05)
------------------

2.7.4 (2018-02-12)
------------------

2.7.3 (2017-12-11)
------------------

2.7.2 (2017-05-21)
------------------

2.7.1 (2017-04-28)
------------------
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (`#571 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/571>`_)
* Contributors: Jose Luis Rivero

2.5.12 (2017-04-25)
-------------------

2.5.11 (2017-04-18)
-------------------
* Changed the spawn model methods to spawn also lights. (`#511 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/511>`_)
* Contributors: Alessandro Ambrosano

2.5.10 (2017-03-03)
-------------------

2.5.9 (2017-02-20)
------------------
* Removed all trailing whitespace
* Contributors: Dave Coleman

2.5.8 (2016-12-06)
------------------

2.5.7 (2016-06-10)
------------------

2.5.6 (2016-04-28)
------------------

2.5.5 (2016-04-27)
------------------
* merge indigo, jade to kinetic-devel
* Update maintainer for Kinetic release
* Contributors: Jose Luis Rivero, Steven Peters

2.5.3 (2016-04-11)
------------------

2.5.2 (2016-02-25)
------------------
* merging from indigo-devel
* 2.4.9
* Generate changelog
* GetModelState modification for jade
* Contributors: John Hsu, Jose Luis Rivero, Markus Bader

2.5.1 (2015-08-16)
------------------

2.5.0 (2015-04-30)
------------------

2.4.10 (2016-02-25)
-------------------

2.4.9 (2015-08-16)
------------------

2.4.8 (2015-03-17)
------------------

2.4.7 (2014-12-15)
------------------
* Update Gazebo/ROS tutorial URL
* Contributors: Jose Luis Rivero

2.4.6 (2014-09-01)
------------------

2.4.5 (2014-08-18)
------------------

2.4.4 (2014-07-18)
------------------
* Fix repo names in package.xml's
* Contributors: Jon Binney

2.4.3 (2014-05-12)
------------------

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
* rerelease because sdformat became libsdformat, but we also based change on 2.3.4 in hydro-devel.

2.4.0 (2013-10-14)
------------------

2.3.5 (2014-03-26)
------------------

2.3.4 (2013-11-13)
------------------

2.3.3 (2013-10-10)
------------------

2.3.2 (2013-09-19)
------------------

2.3.1 (2013-08-27)
------------------

2.3.0 (2013-08-12)
------------------

2.2.1 (2013-07-29)
------------------

2.2.0 (2013-07-29)
------------------

2.1.5 (2013-07-18)
------------------

2.1.4 (2013-07-14)
------------------

2.1.3 (2013-07-13)
------------------

2.1.2 (2013-07-12)
------------------
* Cleaned up CMakeLists.txt for all gazebo_ros_pkgs
* 2.1.1

2.1.1 (2013-07-10 19:11)
------------------------

2.1.0 (2013-06-27)
------------------

2.0.2 (2013-06-20)
------------------

2.0.1 (2013-06-19)
------------------
* Incremented version to 2.0.1

2.0.0 (2013-06-18)
------------------
* Changed version to 2.0.0 based on gazebo_simulator being 1.0.0
* Updated package.xml files for ros.org documentation purposes
* Imported from bitbucket.org
