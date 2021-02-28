roboticsgroup_gazebo_plugins
================

Collection of small gazebo plugins
----------------------------------

MimicJointPlugin
----------------

A simple (Model) plugin for Gazebo in order to add to Gazebo the mimic joint functionality that exists in URDF (ROS). Inspired by code of Goncalo Cabrita.

  - *XML Parameters*

    - joint (Required)

      A **string** specifying the name of the joint to be mimic-ed.

    - mimicJoint (Required)

      A **string** specifying the name of the mimic joint.

    - multiplier

      A **double** specifying the multiplier parameter of the mimic joint. Defaults to 1.0.

    - offset

      A **double** specifying the offset parameter of the mimic joint. Defaults to 0.0.

    - maxEffort

      A **double** specifying the max effort the mimic joint can generate. Defaults to 1.0.

    - sensitiveness

      A **double** specifying the sensitiveness of the mimic joint. Defaults to 0.0. It basically is the threshold of the difference between the 2 angles (joint's and mimic's) before applying the "mimicness".

    - robotNamespace

      A **string** specifying the namespace the robot is under. Defaults to '/'.

    - hasPID

      Determines whether the joint has PID in order to be controlled via PID position/effort controller. Takes no value: *\<hasPID/\>* means that the mimic joint is controlled via PID. Ommit it so that the mimic joint is controlled via setAngle.

DisableLinkPlugin
-----------------

A simple (Model) plugin for Gazebo that allows you to disable a link in Gazebo's physics engine.

  - *XML Parameters*

    - link (Required)

      A **string** specifying the name of the link to be disabled. It should be a valid sdf (not urdf) link.

### Hoping to add more plugins....

Usage
------

Standard Gazebo plugin import inside xacro/urdf. Use **libroboticsgroup_gazebo_** prefix. E.g. if you want to import MimicJointPlugin:

```
libroboticsgroup_gazebo_mimic_joint_plugin.so
```

Notes
------

If there is a need, please make an issue and I'll see what I can do to add that functionality/plugin.

License
----

BSD


Copyright (c) 2014, **Konstantinos Chatzilygeroudis**
