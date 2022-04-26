#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <thread>
#include "uav_track_car/pid_interface.hpp"
#include <memory>
#include <boost/bind.hpp>

tf::TransformListener* listener;
std::shared_ptr<PIDInterface> x_control_ptr;
std::shared_ptr<PIDInterface> y_control_ptr;
std::shared_ptr<PIDInterface> z_control_ptr;
std::shared_ptr<PIDInterface> yaw_control_ptr;

Eigen::Vector4f uav_pose_target_car;
bool get_localization = false;

ros::Publisher cmd_vel_publisher;
ros::Publisher cmd_vel_flu_publisher;
ros::Publisher pid_plot_publisher;

void uav_pose_callback(const ros::TimerEvent&){

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
        listener->lookupTransform("tag_0", "iris_0/base_link", rostime,transform);
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

    Eigen::Vector4f now_T(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), tf::getYaw(q));
    uav_pose_target_car = now_T;
    get_localization = true;
}

void SendZeroVelocity(){
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = 0;
    cmd_msg.linear.y = 0;
    cmd_msg.linear.z = 0;
    cmd_msg.angular.z = 0;
    std_msgs::String s_msg;
    s_msg.data = 'HOVER';
    cmd_vel_flu_publisher.publish(cmd_msg);
    cmd_vel_publisher.publish(s_msg);
}

static inline float normalizeHeadingRad(float t) {
    if (t < 0) {
        t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
        return 2.f * M_PI + t;
    }

    return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}

void pid_control_uav(){

    geometry_msgs::Twist cmd_msg;
    nav_msgs::Odometry odom;
    odom.header.frame_id = "map";
    float fly_h = 5.0;
    while(true){
        if(!get_localization) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        if((uav_pose_target_car.block<3,1>(0,0) -Eigen::Vector3f(0,0,fly_h)).norm() > 5){
            std::cout << "追踪失败， 请重新定位" << std::endl;
            SendZeroVelocity();
            continue;
        }
        std::cout << "当前位置 ： " << uav_pose_target_car.transpose() << std::endl;
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = uav_pose_target_car(0);
        odom.pose.pose.position.y = uav_pose_target_car(1);
        odom.pose.pose.position.z = uav_pose_target_car(2);
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(uav_pose_target_car(3));
        pid_plot_publisher.publish(odom);
        float vel_x_output = x_control_ptr->Control(0, uav_pose_target_car.x());
        float vel_y_output = y_control_ptr->Control(0, uav_pose_target_car.y());
        float vel_z_output = z_control_ptr->Control(fly_h, uav_pose_target_car.z());
        // YAW
        float yaw = normalizeHeadingRad(uav_pose_target_car.w());
        std::cout << "yaw : " << yaw << std::endl;
        float vel_yaw_output = yaw_control_ptr->Control(M_PI, yaw);
        if(std::abs(vel_yaw_output) < 0.2)
            vel_yaw_output = 0;
        std::cout << "控制输出 ： " << vel_x_output << " " << vel_y_output << " " << vel_z_output << " " << vel_yaw_output << std::endl;
        if(std::abs(uav_pose_target_car(0)) < 0.3)
            cmd_msg.linear.x = 0;
        else
            cmd_msg.linear.x = -vel_x_output * 2;

        if(std::abs(uav_pose_target_car(1)) < 0.3)
            cmd_msg.linear.y = 0;
        else
            cmd_msg.linear.y = -vel_y_output * 2;

        if(std::abs(uav_pose_target_car(2) - fly_h) < 0.3)
            cmd_msg.linear.z = 0;
        else
            cmd_msg.linear.z =  vel_z_output * 2;

        cmd_msg.angular.z = vel_yaw_output;
        cmd_vel_flu_publisher.publish(cmd_msg);
        
	std_msgs::String s_msg;
        s_msg.data = "";
        cmd_vel_publisher.publish(s_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

}

int main(int argc, char **argv){

    ros::init(argc, argv, "vio_mapping_node");
    ros::NodeHandle nh("~");

    // 主要是监听 tag(车顶二维码标签) 到 无人机的坐标关系图
    tf::TransformListener listener_;
    listener = &listener_;

    cmd_vel_publisher = nh.advertise<std_msgs::String>("/xtdrone/iris_0/cmd", 100);
    cmd_vel_flu_publisher = nh.advertise<geometry_msgs::Twist>("/xtdrone/iris_0/cmd_vel_flu", 100);;

    pid_plot_publisher = nh.advertise<nav_msgs::Odometry>("/pid_plot", 100);

    x_control_ptr = std::make_shared<PIDInterface>(0.6, 0.0, 0.4);
    y_control_ptr = std::make_shared<PIDInterface>(0.6, 0.0, 0.4);
    z_control_ptr = std::make_shared<PIDInterface>(0.6, 0.0, 0.4);
    yaw_control_ptr = std::make_shared<PIDInterface>(0.6, 0, 0.4);

    ros::Timer      odom_timer = nh.createTimer(ros::Duration(0.03), &uav_pose_callback);
    std::thread th(boost::bind(&pid_control_uav));
    th.detach();

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
