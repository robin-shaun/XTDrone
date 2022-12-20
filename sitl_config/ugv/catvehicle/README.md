# Cognitive and Autonomous Test Vehicle (CAT Vehicle) Testbed
The CAT Vehicle is a ROS based simulator to facilitate the development of autonomous vehicle applications. This repository houses the files that utilize the Gazebo simulator, and additional interfaces to the physical CAT Vehicle Testbed available at the University of Arizona - Department of Electrical and Computer Engineering.

# Dependencies
* ROS
* [obstaclestopper](https://github.com/jmscslgroup/obstaclestopper)
* [control_toolbox](https://github.com/jmscslgroup/control_toolbox)
* [sicktoolbox](https://github.com/jmscslgroup/sicktoolbox)
* [sicktoolbox_wrapper](https://github.com/jmscslgroup/sicktoolbox_wrapper)
* [stepvel](https://github.com/jmscslgroup/stepvel)
* [cmdvel2gazebo](https://github.com/jmscslgroup/cmdvel2gazebo)
* Controller manager (allows the car to move around with ROS messages)
```shell
  sudo apt-get update
  sudo apt-get install ros-noetic-controller-manager
  sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
  sudo apt-get install ros-noetic-gazebo-ros-control
  ```

# System Requirements
* Ubuntu 20.04 LTS (We cannot guarantee if it works on any other version of Ubuntu)
* RAM: 4GB required, > 8GB recommended.


# Citing this work
If you find this work useful please give credits to the authors and developers by citing:
```json
Rahul Bhadani, Jonathan Sprinkle, Matthew Bunting. "The CAT Vehicle Testbed: 
A Simulator with Hardware in the Loop for Autonomous Vehicle Applications". 
Proceedings 2nd International Workshop on Safe Control of Autonomous Vehicles (SCAV 2018),
Porto, Portugal, 10th April 2018, Electronic Proceedings in Theoretical Computer Science 269,
pp. 32–47.  Download:  http://dx.doi.org/10.4204/EPTCS.269.4.
```

bibtex:
```
@article{bhadani2018cat,
  title={{The CAT Vehicle Testbed: A Simulator with Hardware 
  in the Loop for Autonomous Vehicle Applications}},
  author={Bhadani, Rahul and Sprinkle, Jonathan and Bunting, Matthew},
  journal={{Proceedings of 2nd International Workshop on Safe Control of Autonomous Vehicles
  (SCAV 2018), Porto, Portugal, 10th April 2018, Electronic Proceedings
  in Theoretical Computer Science 269, pp. 32–47}},
year={2018}
}
```

# What's new
* Released for Ubuntu 20.04 LTS, ROS Noetic and Gazebo 11.0
* Support for front camera
* More stable vehicle dynamics
* Bug fixes and improvements

# How to use it

## Installing ROS
* Follow the steps mentioned in the [ROS wiki page](http://wiki.ros.org/noetic/Installation/Ubuntu%C2%A0) on how to install ROS Noetic. 
* In addition to that we are required to install some additional ros packages
```shell
sudo apt-get install ros-noetic-velodyne
```

## Creating catkin workspace
In order to use the catvehicle ROS package, you should work within a catkin workspace. If you do not already have one:
```shell
cd ~
mkdir -p catvehicle_ws/src
cd catvehicle_ws/src
catkin_init_workspace
cd ..
catkin_make
```

At this point, you can clone this repo and other dependent package into your src directory
```shell
cd ~/catvehicle_ws/src
git clone https://github.com/jmscslgroup/catvehicle
git clone https://github.com/jmscslgroup/obstaclestopper
git clone https://github.com/jmscslgroup/control_toolbox
git clone https://github.com/jmscslgroup/sicktoolbox
git clone https://github.com/jmscslgroup/sicktoolbox_wrapper
git clone https://github.com/jmscslgroup/stepvel
git clone https://github.com/jmscslgroup/cmdvel2gazebo
cd ..
catkin_make
```
## Sourcing workspace to the environment path
```bash
echo "source ~/catvehicle_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# Simple tutorial and examples
Follow the tutorials on the CAT Vehicle Testbed group on the [CPS Virtual Organization](https://cps-vo.org/node/31792) to see how to use the testbed.

# Issues
If you run into a problem, please feel free to post to [issues](https://github.com/jmscslgroup/catvehicle/issues). If the issue is urgent, please email to catvehicle@list.arizona.edu.

# Acknowledgements
## License
Copyright (c) 2013-2020 Arizona Board of Regents; The University of Arizona
All rights reserved

Permission is hereby granted, without written agreement and without 
license or royalty fees, to use, copy, modify, and distribute this
software and its documentation for any purpose, provided that the 
above copyright notice and the following two paragraphs appear in 
all copies of this software.
 
IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY 
FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES 
ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN 
IF THE ARIZONA BOARD OF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.

THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, 
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
IS ON AN "AS IS" BASIS, AND THE ARIZONA BOARD OF REGENTS HAS NO OBLIGATION
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

## Authors and contributors
* Jonathan Sprinkle (sprinkjm@email.arizona.edu)
* Rahul Bhadani (rahulbhadani@email.arizona.edu)
* Sam Taylor
* Kennon McKeever (kennondmckeever@email.arizona.edu)
* Alex Warren
* Swati Munjal (smunjal@email.arizona.edu)
* Ashley Kang (askang@email.arizona.edu)
* Matt Bunting (mosfet@email.arizona.edu)
* Sean Whitsitt

## Support
This work was supported by the National Science Foundation and AFOSR under awards 1659428, 1521617, 1446435, 1262960 and 1253334. Any opinions, findings, and conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the National Science Foundation.

