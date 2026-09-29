#### 概述

1. 实现功能
    控制固定翼无人机集群：长机路径跟随、僚机仿射编队跟随、碰撞规避。
2. 任务想定
    以控制6架无人机为例，Vehicle0、Vehicle1、vehicle2为长机、vehicle3、vehicle4、vehicle5为僚机
    目标曲线为y=x，起飞原点为纬度：47.3977419、经度：8.5455946（此原点根据QGC软件载入无人机默认位置设置）
    在QGC里对六架无人机上传任务，无人机通过QGC起飞到达一定高度后，通过switch模块切换无人机为offboard模式，此时无人机开始执行任务
3. 实现逻辑
    首先订阅控制无人机所需要的MAVROS消息，赋值给自定义无人机全部的状态量FWstates，发布自定义无人机的节点；订阅长机和长机邻居的曲线位置数据，基于长机导航律control_law()获得长机的速度和角速度；订阅僚机的数据和其余无人机的状态信息，基于僚机控制律follower_control_law()获得僚机的速度和角速度；将获得的速度和角速度指定转换为姿态和油门，发布给PX4
4. 启动
    编译前先将fixed_wing_formation_control.launch拷贝到PX4_Firmware的launch文件中；
    首先配置好相关依赖（CMakeLists）、在工作空间下catkin_make，编译完成后，直接bash在scripts文件夹下的multi_uav_sim_ba_6vtol.sh；
    并未写完全部都键盘控制指令,本仿真几乎不采用键盘控制，可以预先通过QGC规划好航线来执行起飞飞行轨迹；
    然后通过QGC实现Mission下的起飞，待无人机起飞到所需高度后，通过键盘输入切换到offboard模式
    具体步骤：
    1、~/QGroundControl.AppImage
    2、另起终端
        cd ~/catkin_ws/src/fixed_wing_formation_control/scripts
        bash multi_uav_sim_ba_6vtol.sh

5. 注意事项
    编译失败时，可以将include的头文件复制到devel/include/fixed_wing_formation_control;
    如果Gazebo启动失败，可能是进程没完全关闭，通过killall gzserver；killall gzclient 两条指令关闭。
    QGC参数设置（每架无人机都需要设置）：
    (1).EKF2_AID_MASK:1选择使用GPS(此参数解决not ready问题)；
    (2).COM_RCL_EXCEPT:4选择offbord（此参数解决 No manual control stick input问题）;
    输入输出限幅的时候要注意QGC里边对飞机模型参数的设定；
    这里与XTDrone的关系只在于使用了和XTDrone一样的PX4，ROS等配置环境；
    尚未添加无人机起飞程序，需要采用QGC部署任务起飞；
6. 重点
    代码控制律部分在abs_formation_controller.cpp，主控程序为task_main.cpp，自定义的消息类型在msg文件夹下。
    有一些关于代码的注意在代码注释中，整体框架来源于  
    https://github.com/lee-shun/fixed_wing_formation_control.git；
    代码的消息订阅等部分有很多冗余设置，可以只选择自身需要的；
    此代码还有很多可以完善的地方，比如参数的设定直接通过结构体赋值来改变了，键盘控制部分还不完善，  
    所用的TECS源码也不是最接近XTDrone环境的，函数架构还能进一步精简等，后续有机会继续改进；
    僚机跟踪控制代码读取邻居无人机的方式是逐个读取，可以考虑用数组方式简化代码；
    若做完一次仿真，重新启动后遇到一些意想不到的问题，可以试试rosclean purge。

#### bash及各个函数主要功能介绍

1. 在bash 文件中依次给出了所启动的C++功能，其对应的C++实现在muti_uav_sim文件中，通过这个文件的功能链接到指定的源码中，这样也对应了每个源码的功能了；
2. bash依次实现的功能是：编译源码；启动launch环境；订阅6架无人机并向领机发送mavros消息(实现所需消息的订阅，飞行模式的切换，最终控制指令的发送)；长机0、1、2号的主控程序(这里实现了整个的控制逻辑)；僚机3、4、5号的主控程序；最后4个依次是切换无人机飞行模式和队形设定的(主要用到从Mission切换到Offboard的同时设定队形)。






