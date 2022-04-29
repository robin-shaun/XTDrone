#include "hybrid_astar_test/publisher/tf_broadcaster.hpp"

TfBroadcaster::TfBroadcaster(ros::NodeHandle& nh, const std::string& child_frame, const std::string& frame) :
        nh_(nh) ,  child_frame_(child_frame), frame_(frame) {

    std::thread th(boost::bind(&TfBroadcaster::TFThread, this));
    th.detach();

}


void TfBroadcaster::TFThread(){
    tf::TransformBroadcaster broadcaster;
    tf::Pose tfPose;

    while (true){
        broadcaster.sendTransform(
                tf::StampedTransform(
                        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
                        ros::Time::now(), frame_.c_str(), child_frame_.c_str()));
        if(end_flag)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}