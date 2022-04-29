#include "hybrid_astar_test/subscriber/start_subscriber.hpp"

StartSubscriber::StartSubscriber(ros::NodeHandle& nh, const std::string& base_frame, const std::string& world_frame)
    :  base_frame_(base_frame), world_frame_(world_frame) {

//    tf::TransformListener listener_;
//    listener = &listener_;

    std::thread costmap_thread(boost::bind(&StartSubscriber::StartCallback, this) );
    costmap_thread.detach();

}

StartSubscriber::~StartSubscriber(){
    end_flag = true;
}

bool StartSubscriber::GetStart(Eigen::Vector3f& start){
    if(get_start)
        start = start_point;
    auto flag = get_start;
    get_start = false;
    return flag;
}

void StartSubscriber::StartCallback(){

    tf::StampedTransform transform;
    static bool first_step = false;

    tf::TransformListener listener_;
    listener = &listener_;

    while(true) {
        if(end_flag)
            break;
        try {
            auto rostime = ros::Time(0);
            if (!first_step) {
                ros::Duration(1).sleep();
                first_step = true;
            } else
                ros::Duration(0.1).sleep();
            listener->lookupTransform(world_frame_.c_str(), base_frame_.c_str(), rostime, transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        global_mtx.lock();
        start_point(0) = transform.getOrigin().x();
        start_point(1) = transform.getOrigin().y();
        start_point(2) = static_cast<float>(tf::getYaw(transform.getRotation()));
        get_start = true;
        global_mtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}