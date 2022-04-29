#include "hybrid_astar_test/subscriber/costmap_subscriber.hpp"

CostmapSubscriber::CostmapSubscriber(ros::NodeHandle &nh, const std::string &topic) : nh_(nh){

    costmap_subscriber_ptr_ = nh_.subscribe<nav_msgs::OccupancyGrid>(topic.c_str(), 1000, &CostmapSubscriber::CostMapCallback, this);

}

CostmapSubscriber::~CostmapSubscriber(){

}

void CostmapSubscriber::CostMapCallback(const nav_msgs::OccupancyGridConstPtr& msg){
    global_mtx_.lock();
    costmap_ = *msg;
    get_map = true;
    global_mtx_.unlock();
}

bool CostmapSubscriber::GetCostMap(nav_msgs::OccupancyGrid& map, float& map_origin_x, float& map_origin_y){

    map = costmap_;

    map_origin_x = costmap_.info.origin.position.x;
    map_origin_y = costmap_.info.origin.position.y;

    map.info.origin.position.x -= map_origin_x;
    map.info.origin.position.y -= map_origin_y;

    for(int row = 0; row < costmap_.info.height; row++) {
        for (int col = 0; col < costmap_.info.width; col++) {
            int p = int(costmap_.data[col + row * costmap_.info.width]);
            if (p > 50) {
                map.data[col + row * costmap_.info.width] = 100;
                int ww = 0;
                for(int i = -1 * ww; i <= ww; i++){
                    for(int j = -1 * ww; j <= ww; j++){
                        if(col + i < 0 || col +i > costmap_.info.width ||
                           row + j < 0 || row +j > costmap_.info.height){
                            continue;
                        }
                        map.data[col+ i + (row +j) * costmap_.info.width] = 100;
                    }
                }
            } else if (p >= 0 && p <= 50) {
                map.data[col + row * costmap_.info.width] = 0;
            } else {
                map.data[col + row * costmap_.info.width] = 0;
            }
        }
    }

    return get_map;
}
