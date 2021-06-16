# XTDrone

<div id="sidebar"><a href="./README.en.md" target="_blank"><font color=#0000FF size=5px >[ENGLISH]<font></center><a></div>

### 介绍
XTDrone是基于PX4、ROS与Gazebo的无人机仿真平台。支持多旋翼飞行器（包含四轴和六轴）、固定翼飞行器、复合翼飞行器（包含quadplane，tailsitter和tiltrotor）与其他无人装备（如无人车、无人船与机械臂）。在XTDrone上验证过的算法，可以方便地部署到真实无人机上。

<img src="./images/vehicles.png" width="640"  />

单机仿真架构如下图所示，详见论文

Xiao, K., Tan, S., Wang, G., An, X., Wang, X., Wang, X.: XTDrone: A Customizable Multi-rotor UAVs Simulation Platform. arXiv preprint **[ arXiv:2003.09700](https://arxiv.org/abs/2003.09700)** (2020)

<img src="./images/architecture_1_cn.png" width="640" height="480" /> 

多机仿真架构如下图所示，详见论文 

Xiao, K., Ma, L., Tan, S., Cong, Y., Wang, X.: Implementation of UAV Coordination Based on a Hierarchical Multi-UAV Simulation Platform. arXiv preprint **[ arXiv:2005.01125](https://arxiv.org/abs/2005.01125)** (2020)

<img src="./images/architecture_2_cn.png" width="640" />

在这个平台上，开发者可以快速验证算法。如：

双目SLAM

<img src="./images/vslam.gif" width="640" height="360" /> 

视觉惯性导航

<img src="./images/vio.gif" width="640" height="360" />  

视觉稠密重建

<img src="./images/dense_reconstruction.gif" width="640" height="360" /> 

2D激光SLAM

<img src="./images/laser_slam_2d.gif" width="640" height="360" /> 

3D激光SLAM

<img src="./images/laser_slam_3d.gif" width="640" height="360"/>  

2D运动规划

<img src="./images/2d_motion_planning.gif" width="640" height="360" />  

3D运动规划

<img src="./images/3d_motion_planning.gif" width="640" height="360" />  

目标检测与追踪

<img src="./images/human_tracking.gif" width="640" height="360" /> 

多机编队

<img src="./images/formation_1.gif" width="640" height="360" />  

<img src="./images/formation_2.gif" width="640" height="360" />  

多机精准降落

<img src="./images/multi_precision_landing.gif" width="640" height="360" />  

固定翼

<img src="./images/planes.gif" width="640" height="360" />  

复合翼

<img src="./images/vtols.gif" width="640" height="360" />  


无人车

<img src="./images/ugv.gif" width="640" height="360" />  

无人船

<img src="./images/usv.gif" width="640" height="360" />  


### 教程

见[XTDrone使用文档](https://www.yuque.com/xtdrone/manual_cn)

### 中国机器人大赛无人机挑战赛仿真组

2021年中国机器人大赛将于10月在青岛举办，比赛详情见http://robocup.drct-caa.org.cn/index.php/race/view?id=787，其中无人机挑战赛仿真组的平台使用XTDrone，欢迎大家积极报名，展示自己的风采。

### 项目团队

- 创立者：肖昆，谭劭昌
- 指导老师：王祥科
- 开发团队：肖昆，谭劭昌，王冠政，马澜，王齐鹏，胡新雨，管若乔，胡文信，易丰，颜佳润，鲍毅，陈科研，陈皋

### 加入我们

欢迎广大无人机开发者们加入我们的团队，共同学习进步。如有意向，请把简历（包含对PX4 ROS与Gazebo的掌握情况）发到robin_shaun@foxmail.com，让我们一起完善XTDrone仿真平台。

### 贡献者

非常感谢你们为XTDrone的贡献

孙长浩，聂莹，孔凡杰，李超然，李旭东，林梓涵，何瑶 

### 捐赠

如果您觉得XTDrone对您有帮助，可以给XTDrone团队捐赠，如果您愿意，可以备注上姓名或个人主页，我们将把您添加到赞助人名单中。

<img src="./images/donation.png" width="640" height="360" /> 

### 赞助人

非常感谢你们对XTDrone团队的支持

高多多 张宇翔 李照祥
