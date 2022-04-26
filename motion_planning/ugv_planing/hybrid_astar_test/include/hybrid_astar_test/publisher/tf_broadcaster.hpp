#ifndef HYBRID_ASTAR_TEST_PUBLISHER_TF_BROADCASTER_H_PP
#define HYBRID_ASTAR_TEST_PUBLISHER_TF_BROADCASTER_H_PP

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>

#include <thread>
#include <boost/bind.hpp>

class TfBroadcaster{
public:
    TfBroadcaster(ros::NodeHandle& nh, const std::string& child_frame, const std::string& frame);
    ~TfBroadcaster(){
        end_flag = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    };

private:
    void TFThread();

private:
    ros::NodeHandle nh_;
    std::string     child_frame_;
    std::string     frame_;

    bool            end_flag = false;
};

#endif
