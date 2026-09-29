#include"../../task_main/task_main.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follower5_main"); //初始化ROS节点，节点名
    TASK_MAIN follower5_main;
    if (true)
    {
        follower5_main.set_planeID(5);
        follower5_main.run();
    }
    return 0;
}
