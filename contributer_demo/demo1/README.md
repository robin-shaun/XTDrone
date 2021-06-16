
# UAV Collaborative Simulation 

To simulate UAVs Collaborative for  target  searching ,  recognition and localization.

## Introduction
To run the project, do as follows:
### 1.Source Download
```bash
git clone https://codechina.csdn.net/qq_44715174/uav_collaborative_simulation.git
```
### 2.Simulation Environment
1.copy  .launch and .world to px4
```bash
cd uav_collaborative_simulation
cp worlds/* ~/PX4_Firmware/Tools/sitl_gazebo/worlds/
cp launch/* ~/PX4_Firmware/launch/
```
2.run the simulation environment
```bash
roslaunch px4 cic2021.launch
```
