# outdoor_demo
#### 介绍
用于在比赛中随机生成房屋障碍物、车辆障碍物及人类初始位置
#### 使用说明
1.  代码说明:
包含robocup.world/rover_static.world/outdoor_demo.py，输出为outdoor.world及obstacle.txt
运行:python outdoor_demo.py
2.  注意事项:
根据文件位置修改以下参数：
robocup.world的文件位置：world_file_path;
obstacle.txt行人避障列表的生成位置：actor_avoid_file_path;
outdoor.world生成位置：output_path;
可修改车辆障碍物的数量，默认为5：rover_num=5
3.  若运行一段时间没有输出结果，则需要重新运行代码
