^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_dev
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.7 (2020-05-01)
------------------

2.8.6 (2019-12-26)
------------------

2.8.5 (2019-06-04)
------------------

2.8.4 (2018-07-06)
------------------

2.8.3 (2018-06-04)
------------------

2.8.2 (2018-05-09)
------------------

2.8.1 (2018-05-05)
------------------
* Replace gazebo7 by gazebo9 in gazebo_dev. Gazebo9 is the official version supported in Melodic
* Contributors: Jose Luis Rivero

2.7.4 (2018-02-12)
------------------

2.7.3 (2017-12-11)
------------------

2.7.2 (2017-05-21)
------------------
* Revert gazebo8 changes in Lunar and back to use gazebo7 (`#583 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/583>`_)
* Contributors: Jose Luis Rivero

2.7.1 (2017-04-28)
------------------
* Use gazeob8 as exec_depend
* Use 2.7.0 as starting version
* Depend on gazebo8 instead of gazebo7
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (`#571 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/571>`_)
  * Added catkin package gazebo_dev which provides the cmake config of the installed Gazebo version
  Conflicts:
  gazebo_plugins/package.xml
  gazebo_ros/package.xml
  gazebo_ros_control/package.xml
  * gazebo_plugins/gazebo_ros: removed dependency SDF from CMakeLists.txt
  The sdformat library is an indirect dependency of Gazebo and does not need to be linked explicitly.
  * gazebo_dev: added execution dependency gazebo
* Contributors: Jose Luis Rivero

* Use gazeob8 as exec_depend
* Use 2.7.0 as starting version
* Depend on gazebo8 instead of gazebo7
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (`#571 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/571>`_)
  * Added catkin package gazebo_dev which provides the cmake config of the installed Gazebo version
  Conflicts:
  gazebo_plugins/package.xml
  gazebo_ros/package.xml
  gazebo_ros_control/package.xml
  * gazebo_plugins/gazebo_ros: removed dependency SDF from CMakeLists.txt
  The sdformat library is an indirect dependency of Gazebo and does not need to be linked explicitly.
  * gazebo_dev: added execution dependency gazebo
* Contributors: Jose Luis Rivero

2.5.12 (2017-04-25)
-------------------

2.5.11 (2017-04-18)
-------------------

2.5.10 (2017-03-03)
-------------------

2.5.9 (2017-02-20)
------------------

2.5.8 (2016-12-06)
------------------

2.5.7 (2016-06-10)
------------------

2.5.6 (2016-04-28)
------------------

2.5.4 (2016-04-27)
------------------

2.5.3 (2016-04-11)
------------------

2.5.2 (2016-02-25)
------------------

2.5.1 (2015-08-16 02:31)
------------------------

2.5.0 (2015-04-30)
------------------

2.4.9 (2015-08-16 01:30)
------------------------

2.4.8 (2015-03-17)
------------------

2.4.7 (2014-12-15)
------------------

2.4.6 (2014-09-01)
------------------

2.4.5 (2014-08-18 21:44)
------------------------

2.4.4 (2014-07-18)
------------------

2.4.3 (2014-05-12)
------------------

2.4.2 (2014-03-27)
------------------

2.4.1 (2013-11-13 18:52)
------------------------

2.4.0 (2013-10-14)
------------------

2.3.6 (2014-08-18 20:22)
------------------------

2.3.5 (2014-03-26)
------------------

2.3.4 (2013-11-13 18:05)
------------------------

2.3.3 (2013-10-10)
------------------

2.3.2 (2013-09-19)
------------------

2.3.1 (2013-08-27)
------------------

2.3.0 (2013-08-12)
------------------

2.2.1 (2013-07-29 18:02)
------------------------

2.2.0 (2013-07-29 13:55)
------------------------

2.1.5 (2013-07-18)
------------------

2.1.4 (2013-07-14)
------------------

2.1.3 (2013-07-13)
------------------

2.1.2 (2013-07-12)
------------------

2.1.1 (2013-07-10)
------------------

2.1.0 (2013-06-27)
------------------

2.0.2 (2013-06-20)
------------------

2.0.1 (2013-06-19)
------------------

2.0.0 (2013-06-18)
------------------
