#include"../../task_main/task_main.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follower4_main"); //初始化ROS节点，节点名
    TASK_MAIN follower4_main;
    if (true)
    {
        follower4_main.set_planeID(4);
        follower4_main.run();
    }
    return 0;
}
