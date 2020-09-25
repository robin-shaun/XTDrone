# XTDrone

<div id="sidebar"><a href="./README.en.md" target="_blank"><font color=#0000FF size=5px >[ENGLISH]<font></center><a></div>

### 介绍
这是基于PX4和ROS的无人机仿真平台(目前模拟器使用Gazebo，与Airsim的连接正在开发中)。目前支持多旋翼飞行器（包含四轴和六轴）、固定翼飞行器、可垂直起降固定翼飞行器（包含quadplane，tailsitter和tiltrotor）和无人车。在XTDrone上验证过的算法，可以方便地部署到真实无人机上。

<img src="./image/vehicles.png" width="640"  />

单机仿真架构如下图所示，详见论文

Xiao, K., Tan, S., Wang, G., An, X., Wang, X., Wang, X.: Xtdrone: A customizable multi-rotor uavs simulation platform. arXiv preprint **[ arXiv:2003.09700](https://arxiv.org/abs/2003.09700)** (2020)

<img src="./image/architecture1.png" width="640" height="480" /> 

多机仿真架构如下图所示，详见论文 

Xiao, K., Ma, L., Tan, S., Cong, Y., Wang, X.: Implementation of UAV Coordination Based on a Hierarchical Multi-UAV Simulation Platform. arXiv preprint **[ arXiv:2005.01125](https://arxiv.org/abs/2005.01125)** (2020)

<img src="./image/architecture2.png" width="640" />

在这个平台上，开发者可以快速验证算法。如：

1. 目标检测与追踪

<img src="./image/human_tracking.gif" width="640" height="368" /> 

2. 双目SLAM

<img src="./image/vslam.gif" width="640" height="368" /> 

3. RGBD-SLAM

<img src="./image/rgbdslam.gif" width="640" height="368" /> 

4. 2D激光SLAM

<img src="./image/laser_slam_2d.gif" width="640" height="368" /> 

5. 3D激光SLAM

<img src="./image/laser_slam_3d.gif" width="640" height="368"/>  

6. 视觉惯性导航

<img src="./image/vio.gif" width="640" height="368" />  

7. 运动规划

<img src="./image/motion_planning.gif" width="640" height="368" />  

8. 多机协同

<img src="./image/formation_1.gif" width="640" height="368" />  

<img src="./image/formation_2.gif" width="640" height="368" />  

9. 固定翼

<img src="./image/planes.gif" width="640" height="368" />  

10. VTOL

<img src="./image/vtols.gif" width="640" height="368" />  


11. 自动驾驶

<img src="./image/self_driving.gif" width="640" height="368" />  


### 教程

见[XTDrone使用文档](https://www.yuque.com/xtdrone/manual_cn)

### 空中机器人大赛

2020中国机器人大赛暨RoboCup机器人世界杯中国赛中国机器人大赛正在报名中，详细可参见[大赛官网](http://crc.drct-caa.org.cn/index.php/)，XTDrone作为无人机仿真组的仿真平台，将第一次正式登上舞台，详细比赛规则可参见[附件](http://crc.drct-caa.org.cn/index.php/)。

### 项目团队

- 创立者：肖昆，谭劭昌
- 开发团队：肖昆，谭劭昌，王冠政，马澜，王齐鹏，管若乔，陈科研，陈皋

### 感谢贡献者们

孙长浩 何瑶