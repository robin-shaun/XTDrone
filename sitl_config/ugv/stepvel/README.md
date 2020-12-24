# stepvel
A parametrized ROS node to send step input on cmd_vel topic

## How to use with default arguments
By default stepvel sends constant velocity  of 2.0m/s with 0 steering angle on the /cmd_vel
```shell
rosrun stepvel stepvel_node
```
There are two parameters that __stepvel__ node reads: ```constVel``` ```and strAngle```.
Parameters can be set as follows which will immediately change the input:

```shell
rosparam set /constVel 4.0
```

```shell
rosparam set /strAngle 0.04
```
