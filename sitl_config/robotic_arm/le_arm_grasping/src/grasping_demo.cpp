/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include "le_arm_grasping/grasping_demo.h"
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

GraspingDemo::GraspingDemo(ros::NodeHandle n_, float pregrasp_x, float pregrasp_y, float pregrasp_z, float length, float breadth) :
    it_(n_), 
    armgroup("arm"), 
    grippergroup("gripper"), 
    vMng_(length, breadth)
{
  this->nh_ = n_;

  //获取base_link和camera_link之间的坐标变换的关系，手眼标定的结果，未用上，直接在机器人上
  try
  {
    this->tf_camera_to_robot.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(50.0));
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
    ros::Duration(1.0).sleep();
  }

  try
  {
    this->tf_camera_to_robot.lookupTransform("/base_link", "/camera_link", ros::Time(0), (this->camera_to_robot_));
  }

  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
  }

  grasp_running = false;
  
  this->pregrasp_x = pregrasp_x;
  this->pregrasp_y = pregrasp_y;
  this->pregrasp_z = pregrasp_z;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(5.0).sleep();
  ROS_INFO_STREAM("Getting into the Grasping Position....");
  attainPosition(pregrasp_x, pregrasp_y, pregrasp_z);

  // Subscribe to input video feed and publish object location
  image_sub_ = it_.subscribe("/iris_0/le_arm/camera/image_raw", 1, &GraspingDemo::imageCb, this);
}

void GraspingDemo::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
  if (!grasp_running)
  {
    ROS_INFO_STREAM("Processing the Image to locate the Object...");
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // ROS_INFO("Image Message Received");
    float obj_x, obj_y;
    vMng_.get2DLocation(cv_ptr->image, obj_x, obj_y);

    // Temporary Debugging
    std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
    std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

    obj_camera_frame.setZ(-obj_y);
    obj_camera_frame.setY(-obj_x);
    obj_camera_frame.setX(0.45);

    obj_robot_frame = camera_to_robot_ * obj_camera_frame;
    grasp_running = true;

    // Temporary Debugging
    std::cout<< " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX() << std::endl;
    std::cout<< " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY() << std::endl;
    std::cout<< " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ() << std::endl;
  }
}

void GraspingDemo::attainPosition(float x, float y, float z)
{
  // ROS_INFO("The attain position function called");

  // For getting the pose
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;

  // Starting Postion before picking
  target_pose1.position.x = x;
  target_pose1.position.y = y;
  target_pose1.position.z = z;
  armgroup.setPoseTarget(target_pose1);

  armgroup.move();
}

void GraspingDemo::attainObject()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("arm");
    ROS_INFO_STREAM("process 1.0");
    arm.setGoalJointTolerance(0.001);

    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    //double targetPose[5] = {0.0, 0.715585, -1.047198, 0.349066, 0.0};
    //double targetPose[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double targetPose[5] = {0.226893, 0.191986, -1.029745, 1.396264, 0.0};
    std::vector<double> joint_group_positions(5);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    ROS_INFO_STREAM("process 1.1");

    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    sleep(1);
    ROS_INFO_STREAM("process 1.2");

}

void GraspingDemo::grasp()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("gripper");
    ROS_INFO_STREAM("process 2.0");
    arm.setGoalJointTolerance(0.001);

    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    //double targetPose[5] = {0.0, 0.506146, -1.274091, 0.785399, 0.017453};
    //double targetPose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double targetPose[6] = {-0.20944, 0.20944, -0.20944, 0.20944, -0.20944, 0.20944};
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];
    ROS_INFO_STREAM("process 2.1");

    arm.setJointValueTarget(joint_group_positions);
    ROS_INFO_STREAM("process 2.2");
    arm.move();
    sleep(1);
    ROS_INFO_STREAM("process 2.3");
}

void GraspingDemo::stop()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //stop the process
    moveit::planning_interface::MoveGroupInterface arm("aaa");
}

void GraspingDemo::initiateGrasping()
{
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(3.0).sleep();

  homePose = armgroup.getCurrentPose();
  
  ROS_INFO_STREAM("Approaching the Object....");
  attainObject();

  ROS_INFO_STREAM("Attempting to Grasp the Object now..");
  grasp();

  ROS_INFO_STREAM("Stop the Process....");
  stop();

  grasp_running = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_grasping");
  float length, breadth, pregrasp_x, pregrasp_y, pregrasp_z;
  ros::NodeHandle n;

  //获取一些参数，完成像素和实际位置之间的坐标变换
  if (!n.getParam("probot_grasping/table_length", length))
    length = 0.1;	//0.3
  if (!n.getParam("probot_grasping/table_breadth", breadth))
    breadth = 0.1;	//0.3
  //准备夹取之前，甲爪所在的位置，在画面之外
  if (!n.getParam("probot_grasping/pregrasp_x", pregrasp_x))
    pregrasp_x = 0.01;	//0.00
  if (!n.getParam("probot_grasping/pregrasp_y", pregrasp_y))
    pregrasp_y = 0.00;	//-0.17
  if (!n.getParam("probot_grasping/pregrasp_z", pregrasp_z))
    pregrasp_z = 0.01;	//0.28

  //创建对象存储上述的参数
  GraspingDemo simGrasp(n, pregrasp_x, pregrasp_y, pregrasp_z, length, breadth);
  ROS_INFO_STREAM("Waiting for five seconds..");

  //创建循环检测图像的输入
  ros::WallDuration(5.0).sleep();
  while (ros::ok())
  {
    // Process image callback
    ros::spinOnce();

    simGrasp.initiateGrasping();
  }
  return 0;
}
