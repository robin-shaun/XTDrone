/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/xtdgroundcontrol/qnode.hpp"
//#include <boost/bind.hpp>
#include "QDebug"
#include <math.h>


typedef QList<geometry_msgs::Twist> LISTTWIST;
typedef QList<geometry_msgs::Pose> LISTPOSE;
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xtdgroundcontrol {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv) :
    init_argc(argc),
    init_argv(argv)
    {}
//QNode::QNode(LISTINT multi_select, int *multi_num, QString *multi_type) :
//    {}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}

/*****************************************************************************
** callback functions
*****************************************************************************/


void odm_groundtruth_callback(const geometry_msgs::PoseStamped::ConstPtr& msg, int num_odm, geometry_msgs::Pose local_pose[])
{
    local_pose[num_odm] = msg->pose;
//    qDebug()<<"pose::"<<num_odm<<"::"<<local_pose[num_odm].position.y;
}

/*****************************************************************************
** main functions
*****************************************************************************/

//bool QNode::init() {
//	ros::init("xtdgroundcontrol");
//    ros::init(init_argc,init_argv,"xtdgroundcontrol");
//	if ( ! ros::master::check() ) {
//		return false;
//	}
//	ros::start(); // explicitly needed since our nodehandle is going out of scope.
//	ros::NodeHandle n;
//	// Add your ros communications here.
//	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

//	start();
//	return true;
//}

bool QNode::init(const LISTINT multi_select, const int *multi_num, const LISTSTR multi_type, std::string q_control_type) {
//	ros::init("xtdgroundcontrol");
    int leng_select = multi_select.size();
    int counnnt = 0;
    int counnnnnt = 0;
    ctrl_leader = false;
    cmd_vel_mask = false;
    start_control_flag = false;
    stop_flag = false;
    cmd_change_flag = false;
    control_type = q_control_type;
    for (int i = 0; i < leng_select; i ++)
        multirotor_num = multirotor_num + multi_num[multi_select[i]];
    multirotor_type = multi_type;
    for (int j = 0; j < multirotor_num; j++)
        qDebug()<<"multi_type:::"<<multirotor_type[j].c_str();

    qRegisterMetaType<QVector<int>>("QVector<int>");
    for (int i = 0; i < multirotor_num; i ++)
    {
        vel.append(geometry_msgs::Twist());
        cmd.append(std_msgs::String());
        multirotor_get_control.append(0);
    }
    for (int i = 0; i < multirotor_num; i ++)
    {
        vel[i].angular.x = 0.0;
        vel[i].angular.y = 0.0;
        cmd[i].data = "";
    }
    ros::init(init_argc,init_argv,"xtdgroundcontrol");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    // Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("chatter_0", 1000);
    goal_sub = n.subscribe("/move_base_simple/goal", 1000, &QNode::goal_callback, this);
    path_pub = n.advertise<nav_msgs::Path>("trajectory", 1, true);
    if (control_type == "vel")
    {
        for (int i = 0; i < leng_select; i ++)
        {
            for (int k = 0; k< multi_num[multi_select[i]]; k++)
            {
//                qDebug()<<k;
                if (multi_select[i] == 7)
                {
                    buffer_multi_cmd_vel_flu_pub.push_back(n.advertise<geometry_msgs::Twist>("/ugv_"+std::to_string(k)+"/cmd_vel", 1000));
                    buffer_multi_cmd_pub.push_back(n.advertise<std_msgs::String>("/ugv"+std::to_string(k)+"/cmd",1000));
                }
                else
                {
                    buffer_multi_cmd_vel_flu_pub.push_back(n.advertise<geometry_msgs::Twist>("/xtdrone/"+multi_type[counnnnnt]+"_"+std::to_string(k)+"/cmd_vel_enu", 1000));
                    buffer_multi_cmd_pub.push_back(n.advertise<std_msgs::String>("/xtdrone/"+multi_type[counnnnnt]+"_"+std::to_string(k)+"/cmd",1000));
                    qDebug()<<"multi_type"<<multi_type[counnnnnt].c_str();
                    counnnnnt++;
                }
            }
        }
        leader_cmd_vel_flu_pub = n.advertise<geometry_msgs::Twist>("/xtdrone/leader/cmd_vel_flu",1000);
        leader_cmd_pub = n.advertise<std_msgs::String>("/xtdrone/leader/cmd",1000);
    }
    else
    {
        counnnnnt = 0;
        for (int i = 0; i < leng_select; i ++)
        {
            for (int k = 0; k< multi_num[multi_select[i]]; k++)
            {
//                qDebug()<<k;
                buffer_multi_cmd_accel_flu_pub.push_back(n.advertise<geometry_msgs::Twist>("/xtdrone/"+multi_type[counnnnnt]+"_"+std::to_string(k)+"/cmd_accel_flu", 1000));
                buffer_multi_cmd_pub.push_back(n.advertise<std_msgs::String>("/xtdrone/"+multi_type[counnnnnt]+"_"+std::to_string(k)+"/cmd",1000));
                qDebug()<<"multi_type"<<multi_type[counnnnnt].c_str();
                counnnnnt++;
            }
        }
        leader_cmd_accel_flu_pub = n.advertise<geometry_msgs::Twist>("/xtdrone/leader/cmd_accel_flu",1000);
        leader_cmd_pub = n.advertise<std_msgs::String>("/xtdrone/leader/cmd",1000);
    }

    for (int i = 0; i < leng_select; i ++)
    {
        for (int k = 0; k < multi_num[multi_select[i]]; k ++)
        {
            buffer_multi_odom_groundtruth_sub.push_back(n.subscribe<geometry_msgs::PoseStamped>(multi_type[counnnt]+'_'+std::to_string(k)+"/mavros/local_position/pose", 1000, boost::bind(&odm_groundtruth_callback, _1, counnnt, local_pose)));
            counnnt ++;
        }
    }

    start();
//    qDebug()<<"size"<<sizee;

    return true;
}

// functions for messages transmit from ui to qnode:
void QNode::set_cmd(double q_forward, double q_upward, double q_leftward, double q_orientation, bool q_ctrl_leader, std::string &q_cmd, bool q_cmd_vel_mask)
{
//    int leng = sizeof(multi_num)/sizeof(int);
    for (int i = 0; i < multirotor_num; i++)
    {
//        qDebug()<<"multi_num:"<<multi_num[i];
//        qDebug()<<"multi_control:"<<multirotor_get_control;
        if (multirotor_get_control[i])
        {
            vel[i].linear.x = q_forward;
            vel[i].linear.y = q_leftward;
            vel[i].linear.z = q_upward;
            vel[i].angular.z = q_orientation;
            cmd[i].data = q_cmd;
        }
    }
    ctrl_leader = q_ctrl_leader;
    cmd_vel_mask = q_cmd_vel_mask;
    cmd_change_flag = true;
//    qDebug()<<"forward:"<<q_forward;
//    qDebug()<<"upward:"<<q_upward;
//    qDebug()<<"leftward:"<<q_leftward;
//    qDebug()<<"orientation:"<<q_orientation;
//    qDebug()<<"cmd:"<<QString::fromStdString(cmd[0].data);
//    ROS_INFO_STREAM(q_cmd);

}
void QNode::get_uav_control(LISTINT multirotor_control)
{
    multirotor_get_control = multirotor_control;
    qDebug()<<"multi_control"<< multirotor_get_control;
}
void QNode::stop_control(bool q_stop_flag)
{
    stop_flag = q_stop_flag;
    start_control_flag = false;
    qDebug()<<"stop!!!";
}
void QNode::start_control(bool q_start_control_flag)
{
    start_control_flag = q_start_control_flag;
    stop_flag = false;
    qDebug()<<"control!!!";
}

void QNode::goal_callback(const geometry_msgs::PoseStamped &msg)
{
    goal_pose.position.x  = msg.pose.position.x;
    goal_pose.position.y = msg.pose.position.y;
    get_goal_flag = true;
    qDebug()<<"goal x:"<<goal_pose.position.x;
    qDebug()<<"goal y:"<<goal_pose.position.y;
    qDebug()<<"current x:"<<local_pose[0].position.x;
    qDebug()<<"current y:"<<local_pose[0].position.y;
    emit rvizsetgoal();
}
// functions of ROS
void QNode::run() {
    ros::Rate loop_rate(100);
    int count = 0;
    std::string text_all;
    std::string text[multirotor_num];
    std_msgs::String msg;
    geometry_msgs::Twist vel_goal;
    bool arrive_flag = false;
    int arrive_count = 0;
    int control_num = 0;
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    nav_msgs::Path path;
    //nav_msgs::Path path;
    path.header.stamp = current_time;
    path.header.frame_id = "odom";

//    geometry_msgs::Twist twist[multirotor_num];
    qDebug()<<"start!!!";
    while ( ros::ok() ) {
        //rviz draw path
        current_time = ros::Time::now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = local_pose[0].position.x;
        this_pose_stamped.pose.position.y = local_pose[0].position.y;
        this_pose_stamped.pose.position.z = local_pose[0].position.z;
        this_pose_stamped.pose.orientation.x = local_pose[0].orientation.x;
        this_pose_stamped.pose.orientation.y = local_pose[0].orientation.y;
        this_pose_stamped.pose.orientation.z = local_pose[0].orientation.z;
        this_pose_stamped.pose.orientation.w = local_pose[0].orientation.w;
        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        path_pub.publish(path);
        last_time = current_time;

        text_all = "";
        // show log:
        if (count%30 == 0)
        {
            if (multirotor_num < 10)
            {
                for (int id = 0; id <multirotor_num; id++)
                {
                    text[id] = multirotor_type[id] + " pose:\n" + "uav" + std::to_string(id) + ":\n" + "x:" + std::to_string(local_pose[id].position.x)\
                            + "   y:" + std::to_string(local_pose[id].position.y) + "   z:" + std::to_string(local_pose[id].position.z) + "\n";
                    text_all = text_all + text[id];
                    if (count > 30)
                        emit uavposition(local_pose[id].position.x, local_pose[id].position.y);
                }
                msg.data = text_all;
                chatter_publisher.publish(msg);
//                chatter_publisher_2.publish(msg);
                log(Info,msg.data);
            }
        }
        if (get_goal_flag)
        {
            for (int i = 0; i < multirotor_num; i ++)
            {
                if (multirotor_get_control[i])
                {
                    control_num = i;
                    break;
                }
            }
            double distance_tar_cur = (pow((goal_pose.position.x - local_pose[control_num].position.x),2)+pow((goal_pose.position.y - local_pose[control_num].position.y),2));
            if (count_control%30 == 0)
            {
                qDebug()<< "distance" << distance_tar_cur;
                qDebug()<<"arrive flag"<<arrive_flag;
                qDebug()<<"current pose x"<<local_pose[control_num].position.x;
                qDebug()<<"current pose y"<<local_pose[control_num].position.y;
            }
            if  (distance_tar_cur < 0.1)
            {
                arrive_count += 1;
                if (arrive_count > 3)
                {
                    arrive_flag = true;
                    arrive_count = 0;
                    get_goal_flag = false;
                }
                else
                    arrive_flag = false;
            }
            else
            {
                arrive_count = 0;
                arrive_flag = false;
            }
            if (!arrive_flag)
            {
                tf::quaternionMsgToTF(local_pose[control_num].orientation, quat);
                tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
//                qDebug()<<"yaw:"<<yaw;
                vel_goal = get_uav_control(distance_tar_cur,control_num);
                for (int i = 0; i < multirotor_num; i ++)
                {
                    cmd[i].data = "";
                    if (multirotor_get_control[i])
                    {
                        vel[i].linear.x = vel_goal.linear.x;
                        vel[i].linear.y = vel_goal.linear.y;
                        vel[i].linear.z = vel_goal.linear.z;
                        vel[i].angular.z = vel_goal.angular.z;
                    }
                }
            }
            else
            {
                for (int i = 0; i < multirotor_num; i ++)
                {
                    cmd[i].data = "";
                    if (multirotor_get_control[i])
                    {
                        vel[i].linear.x = 0.0;
                        vel[i].linear.y = 0.0;
                        vel[i].linear.z = 0.0;
                        vel[i].angular.z = 0.0;
                    }
                }
            }
            QNode::publish();
        }

        if (start_control_flag)
        {
            // update cmd:
            cmd_buffer = false;
//            qDebug()<<"cmd_change_flag"<<cmd_change_flag;
            for (int i = 0; i < multirotor_num; i ++)
            {
                if (!cmd_change_flag)
                {
                    cmd_buffer = true;
                    cmd[i].data = "";
                }
                if (cmd[i].data != "")
                {
                    qDebug()<<"cmd222:"<<QString::fromStdString(cmd[i].data);
                    qDebug()<<"forward2:"<<vel[i].linear.x;
                    qDebug()<<"upward2:"<<vel[i].linear.z;
                    qDebug()<<"leftward2:"<<vel[i].linear.y;
                    qDebug()<<"orientation2:"<<vel[i].angular.z;
                }
            }
            cmd_change_flag = false;
            if (!get_goal_flag)
            {
                QNode::publish();
            }
        }
        if (stop_flag)
        {
            for (int i = 0; i < multirotor_num; i++)
            {
                vel[i].linear.x = 0.0;
                vel[i].linear.y = 0.0;
                vel[i].linear.z = 0.0;
                vel[i].angular.z = 0.0;
                cmd[i].data = "AUTO.RTL";
            }
            QNode::publish();
            stop_flag = false;
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    qDebug()<<"finish!";
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
void QNode::publish()
{
    if (ctrl_leader)
    {
        if (control_type == "vel")
            leader_cmd_vel_flu_pub.publish(vel[1]);
        else
            leader_cmd_accel_flu_pub.publish(vel[1]);
        leader_cmd_pub.publish(cmd[1]);
    }
    else
    {
        for(int i = 0; i < multirotor_num; i++)
        {
            if(!cmd_vel_mask)
            {
                if (control_type == "vel")
                    buffer_multi_cmd_vel_flu_pub[i].publish(vel[i]);
                else
                    buffer_multi_cmd_accel_flu_pub[i].publish(vel[i]);
                buffer_multi_cmd_pub[i].publish(cmd[i]);
//                qDebug()<<"cmd333333:"<<QString::fromStdString(cmd[i].data);
            }
        }
    }
}

void QNode::log( const LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
        case(Debug) : {
                ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Info) : {
//				ROS_INFO_STREAM(msg);
//				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                logging_model_msg << msg;
                break;
        }
        case(Warn) : {
                ROS_WARN_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Error) : {
                ROS_ERROR_STREAM(msg);
                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Fatal) : {
                ROS_FATAL_STREAM(msg);
                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

geometry_msgs::Twist QNode::get_uav_control(double distance, int control_num)
{
    geometry_msgs::Twist vel_goal;
    double uav_vel_total =  0.5*distance;
    if (uav_vel_total > 2.0)
        uav_vel_total = 2.0;
    double target_yaw = pos2ang(goal_pose.position.x, goal_pose.position.y, local_pose[control_num].position.x, local_pose[control_num].position.y);
    count_control ++;
    double mid_yaw = target_yaw - yaw;
    if (mid_yaw > 3.1415926)
         mid_yaw = mid_yaw - 3.1415926*2;
    else if(mid_yaw < -3.1415926)
        mid_yaw = 2*3.1415926 + mid_yaw;
    vel_goal.angular.x = 0.0;
    vel_goal.angular.y = 0.0;
    vel_goal.angular.z = 1.0* mid_yaw;
    if (vel_goal.angular.z > 3.0)
        vel_goal.angular.z = 3.0;
    else if (vel_goal.angular.z < -3.0)
        vel_goal.angular.z = -3.0;
    vel_goal.linear.x  = uav_vel_total * cos(target_yaw);
    vel_goal.linear.y = uav_vel_total * sin(target_yaw);
    vel_goal.linear.z = 0.0;
    if (count_control %30 == 0)
    {
    qDebug()<<"target_yaw"<<target_yaw;
    qDebug()<<"current_yaw"<<yaw;
    qDebug()<<"vel_goal x" << vel_goal.linear.x;
    qDebug()<<"vel_goal y" << vel_goal.linear.y;
    qDebug()<<"vel_goal ang" << vel_goal.angular.z;
    }
    return vel_goal;
}

double QNode::pos2ang(double xa, double ya, double xb, double yb)
{
    double angle = 0.0;
        if (xa-xb != 0)
        {
           angle = atan2((ya - yb),(xa - xb));
            if (ya-yb > 0 && angle < 0)
                angle = angle + 3.1415926;
            else if (ya-yb < 0 && angle > 0)
                angle = angle - 3.1415926;
            else if (ya-yb == 0)
            {
                if (xa-xb > 0)
                    angle = 0.0;
                else
                    angle = 3.1415926;
            }
        }
        else
        {
            if (ya-yb > 0)
                angle = 3.1415926 / 2;
            else if (ya-yb <0)
                angle = -3.1415926 / 2;
            else
                angle = 0.0;
        }
        if (angle < 0)
            angle = angle + 2 * 3.1415926;
        return angle;
}
}  // namespace xtdgroundcontrol
