#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

#include <gtest/gtest.h>
#include <iostream>

#include <tf/tf.h>

double curX_;
double curY_;
double curHeading_;
bool vehicleSpawned_;

void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
  // Find the robot's state, update pose variables
  size_t modelCount = msg->name.size();
  tf::Quaternion quat;
  double dummy;

  for(size_t modelInd = 0; modelInd < modelCount; ++modelInd)
  {
    if(msg->name[modelInd] == "pioneer2dx")
    {
      vehicleSpawned_ = true;

      tf::quaternionMsgToTF(msg->pose[modelInd].orientation, quat);
      tf::Matrix3x3 mat(quat);
      mat.getRPY(dummy, dummy, curHeading_);
      curX_ = msg->pose[modelInd].position.x;
      curY_ = msg->pose[modelInd].position.y;

      break;
    }
  }
}

TEST (ModelStateTest, FrameTest)
{
  vehicleSpawned_ = false;
  curHeading_ = 0;

  ros::NodeHandle nh;
  ros::Publisher modelStatePub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 5);
  ros::Subscriber modelStatesSub = nh.subscribe("/gazebo/model_states", 1, &modelStatesCallback);

  gazebo_msgs::ModelState state;

  // Issue commands in the chassis frame
  state.model_name = "pioneer2dx";
  state.pose.orientation.w = 1.0;
  state.reference_frame = "pioneer2dx::chassis";

  // Wait for model to spawn
  while(!vehicleSpawned_)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  // First, turn the robot so it's going about 45 degrees
  state.pose.orientation.z = 0.04018;
  do
  {
    modelStatePub.publish(state);
    ros::Duration(0.2).sleep();
    ros::spinOnce();
  } while(::fabs(curHeading_ - M_PI/4) > 0.05);

  state.pose.orientation.z = 0;
  state.pose.position.x = 0.1;

  // Now, stop the robot and drive forwards about 5 meters
  do
  {
    modelStatePub.publish(state);
    ros::Duration(0.2).sleep();
    ros::spinOnce();
  } while(::sqrt(curX_*curX_ + curY_*curY_) < 5.0);

  // The X and Y values should be approximately the same,
  // and should be roughly sqrt(25/2)
  EXPECT_LT(::fabs(curX_ - 3.535533906), 0.2);
  EXPECT_LT(::fabs(curY_ - 3.535533906), 0.2);

  state.pose.position.x = 0;
  state.reference_frame = "world";

  // Now, send the robot to (0, 0) in the world frame
  do
  {
    modelStatePub.publish(state);
    ros::Duration(0.2).sleep();
    ros::spinOnce();
  } while(::sqrt(curX_*curX_ + curY_*curY_) > 0.1);

  EXPECT_LT(::fabs(curX_), 0.01);
  EXPECT_LT(::fabs(curY_), 0.01);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "set_model_state-test");
  ros::Time::init();

  return RUN_ALL_TESTS();
}
