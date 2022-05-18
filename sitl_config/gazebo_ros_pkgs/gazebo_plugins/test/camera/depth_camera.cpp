#include <gtest/gtest.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class DepthCameraTest : public testing::Test
{
protected:
  virtual void SetUp()
  {
    has_new_image_ = false;
    has_new_depth_ = false;
    has_new_points_ = false;
  }

  ros::NodeHandle nh_;
  image_transport::Subscriber cam_sub_;
  image_transport::Subscriber depth_sub_;
  ros::Subscriber points_sub_;
  bool has_new_image_;
  ros::Time image_stamp_;
  bool has_new_depth_;
  ros::Time depth_stamp_;
  bool has_new_points_;
  ros::Time points_stamp_;
public:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    image_stamp_ = msg->header.stamp;
    has_new_image_ = true;
  }
  void depthCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    depth_stamp_ = msg->header.stamp;
    has_new_depth_ = true;
  }
  void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    points_stamp_ = msg->header.stamp;
    has_new_points_ = true;
  }
};

// Test if the camera image is published at all, and that the timestamp
// is not too long in the past.
TEST_F(DepthCameraTest, cameraSubscribeTest)
{
  image_transport::ImageTransport it(nh_);
  cam_sub_ = it.subscribe("camera1/image_raw", 1,
                          &DepthCameraTest::imageCallback,
                          dynamic_cast<DepthCameraTest*>(this));
  depth_sub_ = it.subscribe("camera1/depth/image_raw", 1,
                            &DepthCameraTest::depthCallback,
                            dynamic_cast<DepthCameraTest*>(this));
  points_sub_ = nh_.subscribe("camera1/points", 1,
                             &DepthCameraTest::pointsCallback,
                             dynamic_cast<DepthCameraTest*>(this));
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

  while (!has_new_image_ || !has_new_depth_ || !has_new_points_)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  EXPECT_EQ(depth_stamp_.toSec(), image_stamp_.toSec());
  EXPECT_EQ(depth_stamp_.toSec(), points_stamp_.toSec());
  // This check depends on the update period (currently 1.0/update_rate = 2.0 seconds)
  // being much longer than the expected difference between now and the
  // received image time.
  const double max_time = 1.0;
  const ros::Time current_time = ros::Time::now();
  // TODO(lucasw)
  // this likely isn't that robust - what if the testing system is really slow?
  double time_diff;
  time_diff = (current_time - image_stamp_).toSec();
  ROS_INFO_STREAM(time_diff);
  EXPECT_LT(time_diff, max_time);

  time_diff = (current_time - depth_stamp_).toSec();
  ROS_INFO_STREAM(time_diff);
  EXPECT_LT(time_diff, max_time);

  time_diff = (current_time - points_stamp_).toSec();
  ROS_INFO_STREAM(time_diff);
  EXPECT_LT(time_diff, max_time);

  cam_sub_.shutdown();
  depth_sub_.shutdown();
  points_sub_.shutdown();

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_depth_camera_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
