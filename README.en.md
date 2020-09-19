# XTDrone

<div id="sidebar"><a href="./README.md" target="_blank"><font color=#0000FF size=5px >[中文版]<font></center><a></div>

#### Description

XTDrone is a customizable Multi-Rotor UAVs simulation platform based on PX4 and ROS. Now the simulator of XTDrone is Gazebo, and the connection to Airsim is being developed. XTDrone supports mulitrotors (including quadrotors and hexarotors), fixed wings, VTOLs (including quadplanes, tailsitters and tiltrotors) and rovers. It's convenient to deploy the algorithm to real UAVs after testing and debugging on the simulation platform.

<img src="./image/vehicles.png" width="640"  /> 

Architecture for single vehicle simulation is shown as the below figure.  For more details, see the paper

Xiao, K., Tan, S., Wang, G., An, X., Wang, X., Wang, X.: Xtdrone: A customizable multi-rotor uavs simulation platform. arXiv preprint **[ arXiv:2003.09700](https://arxiv.org/abs/2003.09700)** (2020)

<img src="./image/architecture1.png" width="640" height="480" />  

Architecture for multiple vehicle simulation is shown as the below figure.  For more details, see the paper

Xiao, K., Ma, L., Tan, S., Cong, Y., Wang, X.: Implementation of UAV Coordination Based on a Hierarchical Multi-UAV Simulation Platform. arXiv preprint **[ arXiv:2005.01125](https://arxiv.org/abs/2005.01125)** (2020)

<img src="./image/architecture2.png" width="640" />  

Developers can quickly verify algorithms with XTDrone, such as:

1. Object Detection and Tracking

<img src="./image/human_tracking.gif" width="640" height="368" /> 

2. Stereo SLAM

<img src="./image/vslam.gif" width="640" height="368" /> 

3. RGBD-SLAM

<img src="./image/rgbdslam.gif" width="640" height="368" /> 

4. 2D Laser SLAM

<img src="./image/laser_slam_2d.gif" width="640" height="368" /> 

5. 3D Laser SLAM

<img src="./image/laser_slam_3d.gif" width="640" height="368"/>  

6. VIO 

<img src="./image/vio.gif" width="640" height="368" />  

7. Motion Planning

<img src="./image/motion_planning.gif" width="640" height="368" />  

8. Formation

<img src="./image/formation_1.gif" width="640" height="368" />  

<img src="./image/formation_2.gif" width="640" height="368" />  

9. Fixed wing

<img src="./image/planes.gif" width="640" height="368" />  

10. VTOLs

<img src="./image/vtols.gif" width="640" height="368" />  

11. Self driving

<img src="./image/self_driving.gif" width="640" height="368" />  


#### User manual

 [XTDrone Manual](https://www.yuque.com/xtdrone/manual_en)

#### Developing Team

- Founders: Kun Xiao, Shaochang Tan
- Developers: Kun Xiao, Shaochang Tan, Guanzheng Wang, Lan Ma, Qipeng Wang, Ruoqiao Guan, Keyan Chen, Gao Chen

#### Thanks to Contributers

Changhao Sun, Yao He