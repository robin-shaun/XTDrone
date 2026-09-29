#include"../../task_main/task_main.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follower0_main"); //初始化ROS节点，节点名
    TASK_MAIN follower0_main;
    if (true)
    {
        follower0_main.set_planeID(0);
        follower0_main.run();
    }
    return 0;
}
