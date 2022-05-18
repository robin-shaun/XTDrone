#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <gazebo_msgs/SetJointTrajectory.h>
#include <ignition/math/Quaternion.hh>
#include <math.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "pub_joint_trajectory_test");
  ros::NodeHandle rosnode;
  ros::Publisher pub_ = rosnode.advertise<trajectory_msgs::JointTrajectory>("set_joint_trajectory",100);
  ros::ServiceClient srv_ = rosnode.serviceClient<gazebo_msgs::SetJointTrajectory>("set_joint_trajectory");

  trajectory_msgs::JointTrajectory jt;

  jt.header.stamp = ros::Time::now();
  jt.header.frame_id = "pr2::torso_lift_link";

  jt.joint_names.push_back("r_shoulder_pan_joint");
  jt.joint_names.push_back("r_shoulder_lift_joint");
  jt.joint_names.push_back("r_upper_arm_roll_joint");
  jt.joint_names.push_back("r_elbow_flex_joint");
  jt.joint_names.push_back("r_forearm_roll_joint");
  jt.joint_names.push_back("r_wrist_flex_joint");
  jt.joint_names.push_back("l_shoulder_pan_joint");
  jt.joint_names.push_back("l_shoulder_lift_joint");
  jt.joint_names.push_back("l_upper_arm_roll_joint");
  jt.joint_names.push_back("l_elbow_flex_joint");
  jt.joint_names.push_back("l_forearm_roll_joint");
  jt.joint_names.push_back("l_wrist_flex_joint");

  int n = 500;
  double dt = 0.1;
  double rps = 0.05;
  jt.points.resize(n);
  for (int i = 0; i < n; i++)
  {
    double theta = rps*2.0*M_PI*i*dt;
    double x1 = -0.5*sin(2*theta);
    double x2 =  0.5*sin(1*theta);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(3.14);
    jt.points[i].positions.push_back(-10.0);
    jt.points[i].positions.push_back(-0.2);
    jt.points[i].positions.push_back(-0.2);
    jt.points[i].positions.push_back(x1);
    jt.points[i].positions.push_back(x2);
    jt.points[i].positions.push_back(3.14);
    jt.points[i].positions.push_back(10.0);
    jt.points[i].positions.push_back(-0.2);
    jt.points[i].positions.push_back(-0.2);
    // set duration
    jt.points[i].time_from_start = ros::Duration(dt);
    ROS_INFO_NAMED("joint_trajectory_test", "test: angles[%d][%f, %f]",n,x1,x2);
  }

  // pub_.publish(jt); // use publisher

  gazebo_msgs::SetJointTrajectory sjt;
  sjt.request.joint_trajectory = jt;
  sjt.request.disable_physics_updates = false;

  ignition::math::Quaterniond r(0, 0, M_PI);
  geometry_msgs::Pose p;
  p.position.x = 0;
  p.position.y = 0;
  p.position.z = 0;
  p.orientation.x = r.X();
  p.orientation.y = r.Y();
  p.orientation.z = r.Z();
  p.orientation.w = r.W();
  sjt.request.model_pose = p;
  sjt.request.set_model_pose = true;

  srv_.call(sjt); // use service

  return 0;
}
