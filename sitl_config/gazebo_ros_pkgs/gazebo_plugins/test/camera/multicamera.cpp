#include <gtest/gtest.h>
// #include <image_transport/image_transport.h>
// #include <image_transport/subscriber_filter.h>
// #include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class MultiCameraTest : public testing::Test
{
protected:
  virtual void SetUp()
  {
    has_new_image_ = false;
  }

  ros::NodeHandle nh_;
  // image_transport::SubscriberFilter cam_left_sub_;
  // image_transport::SubscriberFilter cam_right_sub_;
  message_filters::Subscriber<sensor_msgs::Image> cam_left_sub_;
  message_filters::Subscriber<sensor_msgs::Image> cam_right_sub_;

  bool has_new_image_;
  ros::Time image_left_stamp_;
  ros::Time image_right_stamp_;

  // typedef message_filters::sync_policies::ApproximateTime<
  //     sensor_msgs::Image, sensor_msgs::Image
  //     > MySyncPolicy;
  // message_filters::Synchronizer< MySyncPolicy > sync_;


public:
  void imageCallback(
      const sensor_msgs::ImageConstPtr& left_msg,
      const sensor_msgs::ImageConstPtr& right_msg)
  {
    image_left_stamp_ = left_msg->header.stamp;
    image_right_stamp_ = right_msg->header.stamp;
    has_new_image_ = true;
  }
};

// Test if the camera image is published at all, and that the timestamp
// is not too long in the past.
TEST_F(MultiCameraTest, cameraSubscribeTest)
{
  // image_transport::ImageTransport it(nh_);
  // cam_left_sub_.subscribe(it, "stereo/camera/left/image_raw", 1);
  // cam_right_sub_.subscribe(it, "stereo/camera/right/image_raw", 1);
  // sync_ = message_filters::Synchronizer<MySyncPolicy>(
  //     MySyncPolicy(4), cam_left_sub_, cam_right_sub_);
  cam_left_sub_.subscribe(nh_, "stereo/camera/left/image_raw", 1);
  cam_right_sub_.subscribe(nh_, "stereo/camera/right/image_raw", 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(
      cam_left_sub_, cam_right_sub_, 4);
  sync.registerCallback(boost::bind(&MultiCameraTest::imageCallback,
      dynamic_cast<MultiCameraTest*>(this), _1, _2));
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

  double sync_diff = (image_left_stamp_ - image_right_stamp_).toSec();
  EXPECT_EQ(sync_diff, 0.0);

  // This check depends on the update period being much longer
  // than the expected difference between now and the received image time
  // TODO(lucasw)
  // this likely isn't that robust - what if the testing system is really slow?
  double time_diff = (ros::Time::now() - image_left_stamp_).toSec();
  ROS_INFO_STREAM(time_diff);
  EXPECT_LT(time_diff, 1.0);
  // cam_sub_.shutdown();

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
  ros::init(argc, argv, "gazebo_multicamera_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
