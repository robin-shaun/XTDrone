#include"../../task_main/task_main.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leader_main");
    TASK_MAIN leader_main;
    if (true)
    {
        leader_main.set_planeID(0);
       leader_main.run();
    }
    return 0;
}