^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package catvehicle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
2.1.1 (2018-06-21)
------------------
* Added tf for human cars
* 'Matlab files' is now 'mfiles'
* Added an m function convert a bagfile to a mat file.
* Added a simulink block for ramp generation with ros parameters, useful for system identification
* Removed depracted functions in gazebo cpp plugins while migrating from ROS Indigo to ROS Kinetic
* Added tf nodes for publishing tf frames for humancars.
* Contributors: Rahul Kumar Bhadani

2.1.0 (2018-06-10)
------------------
* Updated Car model to include current Inertial tensor of car body and coefficient of frictions to prevent car from toppling over at high speed.
* Contributors: Rahul Kumar Bhadani

2.0.2 (2017-02-02)
------------------
* Removed worlds intended for the cvchallenge_task2 repository
* Contributors: Jonathan Sprinkle
