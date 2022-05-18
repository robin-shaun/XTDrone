#ifndef GAZEBO_PLUGINS_TEST_CAMERA_CAMERA_H
#define GAZEBO_PLUGINS_TEST_CAMERA_CAMERA_H

#include <gtest/gtest.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

class CameraTest : public testing::Test
{
protected:
  virtual void SetUp()
  {
    has_new_image_ = false;
  }

  ros::NodeHandle nh_;
  image_transport::Subscriber cam_sub_;
  bool has_new_image_;
  ros::Time image_stamp_;
public:
  void subscribeTest();

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    image_stamp_ = msg->header.stamp;
    has_new_image_ = true;
  }
};

// Test if the camera image is published at all, and that the timestamp
// is not too long in the past.
void CameraTest::subscribeTest()
{
  image_transport::ImageTransport it(nh_);
  cam_sub_ = it.subscribe("camera1/image_raw", 1,
                          &CameraTest::imageCallback,
                          dynamic_cast<CameraTest*>(this));

#if 0
  // wait for gazebo to start publishing
  // TODO(lucasw) this isn't really necessary since this test
  // is purely passive
  bool wait_for_topic = true;
  while (wait_for_topic)
  {
    // @todo this fails without the additional 0.5 second sleep after the
    // publisher comes online, which means on a slower or more heavily
    // loaded system it may take longer than 0.5 seconds, and the test
    // would hang until the timeout is reached and fail.
    if (cam_sub_.getNumPublishers() > 0)
       wait_for_topic = false;
    ros::Duration(0.5).sleep();
  }
#endif

  while (!has_new_image_)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  // This check depends on the update period being much longer
  // than the expected difference between now and the received image time
  // TODO(lucasw)
  // this likely isn't that robust - what if the testing system is really slow?
  double time_diff = (ros::Time::now() - image_stamp_).toSec();
  ROS_INFO_STREAM(time_diff);
  EXPECT_LT(time_diff, 1.0);
  cam_sub_.shutdown();

  // make sure nothing is subscribing to image_trigger topic
  // there is no easy API, so call getSystemState
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();
  EXPECT_TRUE(ros::master::execute("getSystemState", args, result, payload, true));
  // [publishers, subscribers, services]
  // subscribers in index 1 of payload
  for (int i = 0; i < payload[1].size(); ++i)
  {
    // [ [topic1, [topic1Subscriber1...topic1SubscriberN]] ... ]
    // topic name i is in index 0
    std::string topic = payload[1][i][0];
    EXPECT_EQ(topic.find("image_trigger"), std::string::npos);
  }
}

#endif
