#include <ros/ros.h>
#include <iostream>
#include <memory>

#include "hybrid_astar_test/global_definition/constants.hpp"
#include "hybrid_astar_test/navigation/path_search.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "hybrid_a_star_node");
    ros::NodeHandle nh("~");

    if(!HybridAStar::Constants::LoadParamFromYaml()){
        ros::shutdown();
        nh.shutdown();
        return -1;
    }

    std::shared_ptr<HybridAStar::HybridAStarGraphSearcher> ptr = std::make_shared<HybridAStar::HybridAStarGraphSearcher>(nh);

    ros::Rate r(5);
    while (ros::ok()){
        ros::spinOnce();
        ptr->Run();
        r.sleep();
    }

    return 0;
}