# sicktoolbox_wrapper
sicktoolbox_wrapper is a ROS wrapper for the outstanding sicktoolbox library for interfacing with the SICK LMS2XX lasers.

For this package to work on ROS-KINETIC, follow the instructions below:

1. change the working directory to catkin workspace
```
cd ~/catkin_ws/src
```

2. Clone the two required repositories
```
git clone <git url for sicktoolbox> (https://github.com/SantoshBanisetty/sicktoolbox.git)
git clone <git url for sicktoolbox_wrapper> 
```

3. Make the project
```
cd ~/catkin_ws
catkin_make
```

4. Make the required connections as shown in datasheet. 

5. Connect the USBtoRS232 and make sure the premissions are set properly.
```
ls -l /dev/ttyUSB0
``` 
You will see something similar to:
```
crw-rw-XX-
```
XX should be rw if not, so the following:
```
sudo chmod a+rw /dev/ttyUSB0
```

6. Now that the laser is configured properly, run a ros master like
```
roscore
```

7. Run the node as follows:(make sure you source the workspace)
```
rosrun sicktoolbox_wrapper sicklms _port:=/dev/ttyUSB0 _baud:=38400
```

8. You may use RVIZ to visualize or simply rostopic will suffice to confirm the working
```
rosrun rviz rviz
```

 

