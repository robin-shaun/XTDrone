#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <mutex>
#include <thread>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pure_pursuit/subscriber/path_subscriber.hpp>

#define PI 3.1415926535

std::mutex global_mtx_;
Eigen::VectorXf start_point(6);
Eigen::VectorXf robot_localization(7);  // x y yaw liner_v angle_v liner_acc angle_acc
std::vector<Eigen::Vector3f> Trajectory;


tf::TransformListener* listener;
ros::Publisher         vel_publisher_;
ros::Publisher         path_publisher_;
ros::Publisher         odom_publisher_;
std::shared_ptr<PathSubscriber> path_subscriber_ptr_;

bool get_new_trajectory = false;
bool end_flag = false;
bool get_localization = false;

/********************************************pure pursuit*********************************************/
float pure_pursuit_k = 0.1;          
float pure_pursuit_Lfc = 2;         // l_d
float pure_pursuit_Kp = 1.0;         
float dt = 0.1;                      
float pure_pursuit_L = 2.7;         
float pure_pursuit_liner_v = 3.5;   
/*****************************************************************************************************/

float get_yaw(const geometry_msgs::Quaternion& q){
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(q, quaternion);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll,pitch,yaw);
    return float(yaw);
}

float calculation_yaw_difference(float now_yaw, float last_yaw){
    if(now_yaw - last_yaw > PI){
        return (2 * PI - now_yaw + last_yaw);
    }
    else if( now_yaw - last_yaw < -PI){
        return (2 * PI +now_yaw - last_yaw);
    }
    else{
        return (now_yaw - last_yaw);
    }
}

Eigen::Vector2f calculation_velocity(const Eigen::Vector3f& last_pose, const Eigen::Vector3f& now_pose, float delta_time){
    float liner_v = (now_pose.block<2,1>(0,0) - last_pose.block<2,1>(0,0)).norm() / delta_time;
    float angle_v = calculation_yaw_difference(now_pose(2), last_pose(2)) / delta_time;
    if(abs(liner_v) < 0.001)
        liner_v = 0.0;
    if(abs(angle_v) < 0.001)
        angle_v = 0.0;
    return Eigen::Vector2f(liner_v, angle_v);
}

Eigen::Vector2f calculation_acceleration(const Eigen::Vector2f& last_v, const Eigen::Vector2f& now_v, const Eigen::Vector3f& last_pose, const Eigen::Vector3f& now_pose){
    float x = (now_pose.block<2,1>(0,0) - last_pose.block<2,1>(0,0)).norm();
    float yaw = calculation_yaw_difference(now_pose(2), last_pose(2));
    float acc_x = (now_v(0) * now_v(0) - last_v(0) * last_v(0)) / (2.0 * x);
    float acc_a = (now_v(1) * now_v(1) - last_v(1) * last_v(1)) / (2.0 * yaw);
    if(abs(x) < 0.001)
        acc_x = 0.0;
    if(abs(yaw) < 0.001)
        acc_a = 0.0;
    return Eigen::Vector2f(acc_x, acc_a);
}

void odom_callback(const ros::TimerEvent&){

    tf::StampedTransform transform;
    static bool first_step = false;
    try{
        auto rostime = ros::Time(0);
        if(!first_step){
            ros::Duration(1).sleep();
            first_step = true;
        }
        else
            ros::Duration(0.1).sleep();
        listener->lookupTransform("/map","catvehicle/base_link",rostime,transform);
    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s", ex.what());
        return;
    }

    geometry_msgs::Quaternion q;
    q.x = transform.getRotation().x();
    q.y = transform.getRotation().y();
    q.z = transform.getRotation().z();
    q.w = transform.getRotation().w();

    Eigen::Vector3f now_T(transform.getOrigin().x(), transform.getOrigin().y(), get_yaw(q));
    static bool first_cal = true;
    static Eigen::Vector3f history_T = now_T;
    static Eigen::Vector2f history_velocity = Eigen::Vector2f::Zero();
    static ros::Time t0 = ros::Time::now();
    ros::Time t1 = ros::Time::now();

    global_mtx_.lock();
    start_point(0) = transform.getOrigin().x();
    start_point(1) = transform.getOrigin().y();
    start_point(2) = transform.getRotation().w();
    start_point(3) = transform.getRotation().x();
    start_point(4) = transform.getRotation().y();
    start_point(5) = transform.getRotation().z();
    if(first_cal){
        first_cal = false;
        global_mtx_.unlock();
        return;
    }
    auto robot_velocity = calculation_velocity(history_T, now_T, (t1 - t0).toSec());
    auto robot_acceleration = calculation_acceleration(history_velocity, robot_velocity, history_T, now_T);
    robot_localization << now_T(0), now_T(1), now_T(2),
                          robot_velocity(0), robot_velocity(1),
                          robot_acceleration(0), robot_acceleration(1);
    global_mtx_.unlock();
    get_localization = true;
    history_velocity = robot_velocity;
    history_T = now_T;
    t0 = t1;
}

//void path_callback(const visualization_msgs::MarkerArray::ConstPtr& msg){
//
//    if (msg->markers.size() == 0)
//        return;
//    global_mtx_.lock();
//    Trajectory.clear();
//    nav_msgs::Path path_msg;
//    nav_msgs::Odometry odom_msg;
//    path_msg.header.frame_id = "/map";
//    path_msg.header.stamp = ros::Time::now();
//    odom_msg.header.frame_id = "/map";
//    odom_msg.header.stamp = ros::Time::now();
//    path_publisher_.publish(path_msg);
//    for(size_t num = 0; num < msg->markers.size(); num++){
//        geometry_msgs::PoseStamped poses;
//        poses.pose.position.x  = msg->markers[num].pose.position.x;
//        poses.pose.position.y  = msg->markers[num].pose.position.y;
//        poses.pose.orientation = msg->markers[num].pose.orientation;
//        odom_msg.pose.pose = poses.pose;
//        odom_publisher_.publish(odom_msg);
//        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//
//        path_msg.poses.push_back(poses);
//
//        Trajectory.push_back(Eigen::Vector3f(msg->markers[num].pose.position.x,
//                                             msg->markers[num].pose.position.y,
//                                             get_yaw(msg->markers[num].pose.orientation)));
//        std::cout << "Trajectory : " << Trajectory.back().transpose() << std::endl;
//    }
//    path_publisher_.publish(path_msg);
//    std::reverse(Trajectory.begin(),Trajectory.end());
//    global_mtx_.unlock();
//    get_new_trajectory = true;
//
//}
void path_callback(const nav_msgs::Path::ConstPtr& msg){

    if (msg->poses.size() == 0)
        return;
    global_mtx_.lock();
    Trajectory.clear();
    nav_msgs::Path path_msg;
    nav_msgs::Odometry odom_msg;
    path_msg.header.frame_id = "/map";
    path_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "/map";
    odom_msg.header.stamp = ros::Time::now();
    path_publisher_.publish(path_msg);
    for(size_t num = 0; num < msg->poses.size(); num++){
        geometry_msgs::PoseStamped poses;
        poses.pose.position.x  = msg->poses[num].pose.position.x;
        poses.pose.position.y  = msg->poses[num].pose.position.y;
        if(num == 0)
            poses.pose.orientation = msg->poses[num + 1].pose.orientation;
        else
            poses.pose.orientation = msg->poses[num].pose.orientation;
        odom_msg.pose.pose = poses.pose;
//        odom_publisher_.publish(odom_msg);
//        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        path_msg.poses.push_back(poses);

        Trajectory.push_back(Eigen::Vector3f(msg->poses[num].pose.position.x,
                                             msg->poses[num].pose.position.y,
                                             get_yaw(msg->poses[num].pose.orientation)));
    }
    path_publisher_.publish(path_msg);
    std::reverse(Trajectory.begin(),Trajectory.end());
    global_mtx_.unlock();
    get_new_trajectory = true;

}

int calculation_current_index(){
    global_mtx_.lock();
    Eigen::Vector3f current_state = robot_localization.block<3,1>(0,0);
    int index = 0;
    float diff_distance_ = 1000000.0;
    for(size_t index_ = 0; index_ < Trajectory.size(); index_++){
        float temp_diff_distance_ = (current_state.block<2,1>(0,0) - Trajectory[index_].block<2,1>(0,0)).norm();
        if(temp_diff_distance_ <= diff_distance_){
            diff_distance_ = temp_diff_distance_;
            index = index_;
        }
    }

    float  Lf = pure_pursuit_k * robot_localization(3) + pure_pursuit_Lfc;  
    float L = 0.0;
    while(Lf > L && (index+1) < Trajectory.size()){
        Eigen::Vector2f diff_pts = Trajectory[index+1].block<2,1>(0,0) - Trajectory[index].block<2,1>(0,0);
        L += diff_pts.norm();
        index++;
    }
    global_mtx_.unlock();
    return index;
}

float calculation_control_liner_acceleration(float current_velocity){
    global_mtx_.lock();
    float acc = pure_pursuit_Kp * (pure_pursuit_liner_v - current_velocity);
    global_mtx_.unlock();
    return acc;
}

float calculation_control_angle_acceleration(int& current_index){
    int index = calculation_current_index();
    if(current_index >= index)
        index = current_index;
    Eigen::Vector3f current_point;
    if(index < Trajectory.size())
        current_point = Trajectory[index];
    else{
        current_point = Trajectory.back();
        index = Trajectory.size() - 1;
    }
    auto diff_P = current_point - robot_localization.block<3,1>(0,0);
    float alpha = std::atan2(diff_P.y(), diff_P.x()) - robot_localization(2);

    if(robot_localization(3) < 0){
        alpha = PI - alpha;
    }
    float Lf = pure_pursuit_k * robot_localization(3) + pure_pursuit_Lfc;  
    float delta = std::atan2(2.0 * pure_pursuit_L * sin(alpha) / Lf, 1.0);
    current_index = index;
    static float delta_ = 0.0;
    float return_delta_ = (delta - delta_);
    delta_ = delta;
    return delta;
}

void cmd_publish(float acc_liner, float acc_angle, float dt){
    global_mtx_.lock();
    float v_l = robot_localization(3) + acc_liner * dt;
//    float v_a = robot_localization(4) + acc_angle * dt;
     float v_a = acc_angle - robot_localization(4);
    // float v_a = acc_angle;
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = v_l;
    vel_cmd.angular.z = v_a;
    global_mtx_.unlock();
    std::cout << "\e[1;33;49m >>> v_l ：" << v_l << " v_a : "<< v_a <<" \e[0m"<< std::endl;
//    std::cout << "\e[1;33;49m >>> acc_liner ：" << acc_liner << " \e[0m" << std::endl;
    vel_publisher_.publish(vel_cmd);
}

void zero_velocity(){
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = 0.0;
    vel_cmd.angular.z = 0.0;
    vel_publisher_.publish(vel_cmd);
}

bool arrived_goal(){
    std::cout << "\e[1;33;49m >>> to the goal" << (Trajectory.back().block<2,1>(0,0) - robot_localization.block<2,1>(0,0)).norm() << "m \e[0m"<< std::endl;
    if((Trajectory.back().block<2,1>(0,0) - robot_localization.block<2,1>(0,0)).norm() < 1.0) {
        std::cout << "\e[1;33;49m >>> reached \e[0m"<< std::endl;
        return true;
    }
    else
        return false;
}

void pure_pursuit(){

    bool new_trajectory_flag = false;
    while(true){
        std::cout << "\e[1;33;49m >>> wait for the path \e[0m"<< std::endl;
        while(true) {
            if(end_flag)
                break;
            if(new_trajectory_flag){
                new_trajectory_flag =false;
                break;
            }
            if (path_subscriber_ptr_->GetPath(Trajectory) && get_localization) {
                std::cout << "\e[1;33;49m >>> new path! \e[0m"<< std::endl;
                // get_new_trajectory = false;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "\e[1;33;49m >>> tracking \e[0m"<< std::endl;
        int index = calculation_current_index();
        ros::Time t_0 = ros::Time::now();
        std::cout << "\e[1;33;49m >>> Trajectory.size() : "<< Trajectory.size() << "\e[0m"<< std::endl;
        std::cout << "\e[1;33;49m >>> index : "<< index << "\e[0m"<< std::endl;
        if(Trajectory.size() == 0)
            continue;
        while(index < Trajectory.size() - 1){
            if(arrived_goal()) {
                zero_velocity();
                break;
            }
            if(path_subscriber_ptr_->GetPath(Trajectory)){
                new_trajectory_flag = true;
                break;
            }
            index = calculation_current_index();
            std::cout << "\e[1;33;49m >>> index : "<< index << "\e[0m"<< std::endl;
            std::cout << "\e[1;33;49m >>> Trajectory(index) : "<< Trajectory[index].transpose() << "\e[0m"<< std::endl;
            std::cout << "\e[1;33;49m >>> robot_localization : "<< robot_localization.transpose() << "\e[0m"<< std::endl;
            float acc_liner = calculation_control_liner_acceleration(robot_localization(3));
            float acc_angle = calculation_control_angle_acceleration(index);
            ros::Time t_1 = ros::Time::now();
            float delta_t = (t_1 - t_0).toSec();
            cmd_publish(acc_liner, acc_angle, delta_t);
            t_0 = t_1;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        zero_velocity();

        if(end_flag)
            break;
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle nh("~");

    tf::TransformListener listener_;
    listener = &listener_;

//    ros::Subscriber path_sub =  nh.subscribe<visualization_msgs::MarkerArray>("/new_path_vehicle_node", 100, &path_callback);
    // ros::Subscriber path_sub =  nh.subscribe<nav_msgs::Path>("/new_path", 100, &path_callback);

    vel_publisher_ = nh.advertise<geometry_msgs::Twist>("/catvehicle/cmd_vel_safe", 1);
    path_publisher_= nh.advertise<nav_msgs::Path>("/visual_path",1, true);
    path_subscriber_ptr_ = std::make_shared<PathSubscriber>(nh, "/front_end_poses");
    odom_publisher_= nh.advertise<nav_msgs::Odometry>("/odom_path",1, true);

    ros::Timer      odom_timer = nh.createTimer(ros::Duration(0.1), &odom_callback);

    std::thread pure_pursuit_thread(pure_pursuit);
    pure_pursuit_thread.detach();

    ros::spin();
    end_flag = true;
    zero_velocity();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    return 0;
}
