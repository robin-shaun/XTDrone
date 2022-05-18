#include <gtest/gtest.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

class TriggeredCameraTest : public testing::Test
{
protected:
  virtual void SetUp()
  {
    images_received_ = 0;
  }

  ros::NodeHandle nh_;
  image_transport::Subscriber cam_sub_;
  int images_received_;
  ros::Time image_stamp_;
public:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    image_stamp_ = msg->header.stamp;
    images_received_++;
  }
};

// Test if the camera image is published at all, and that the timestamp
// is not too long in the past.
TEST_F(TriggeredCameraTest, cameraSubscribeTest)
{
  image_transport::ImageTransport it(nh_);
  cam_sub_ = it.subscribe("camera1/image_raw", 5,
                          &TriggeredCameraTest::imageCallback,
                          dynamic_cast<TriggeredCameraTest*>(this));

  // wait for 3 seconds to confirm that we don't receive any images
  for (unsigned int i = 0; i < 30; ++i)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(images_received_, 0);

  // make sure something is subscribing to image_trigger topic
  // there is no easy API, so call getSystemState
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();
  EXPECT_TRUE(ros::master::execute("getSystemState", args, result, payload, true));
  // [publishers, subscribers, services]
  // subscribers in index 1 of payload
  int trigger_listeners = 0;
  for (int i = 0; i < payload[1].size(); ++i)
  {
    // [ [topic1, [topic1Subscriber1...topic1SubscriberN]] ... ]
    // topic name i is in index 0
    std::string topic = payload[1][i][0];
    if (topic.find("image_trigger") != std::string::npos)
    {
      trigger_listeners++;
    }
  }
  EXPECT_EQ(trigger_listeners, 1);

  // publish to trigger topic and expect an update within one second:
  ros::Publisher trigger_pub = nh_.advertise<std_msgs::Empty>("camera1/image_trigger", 1, true);
  std_msgs::Empty msg;

  trigger_pub.publish(msg);
  for (unsigned int i = 0; i < 10 && !images_received_; ++i)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(images_received_, 1);

  // then wait for 3 seconds to confirm that we don't receive any more images
  for (unsigned int i = 0; i < 30; ++i)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(images_received_, 1);

  // then send two trigger messages very close together, and expect two more
  // images
  trigger_pub.publish(msg);
  ros::spinOnce();
  ros::Duration(0.01).sleep();
  trigger_pub.publish(msg);
  ros::spinOnce();
  ros::Duration(0.01).sleep();
  for (unsigned int i = 0; i < 10 && images_received_ < 2; ++i)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(images_received_, 3);

  // then wait for 3 seconds to confirm that we don't receive any more images
  for (unsigned int i = 0; i < 30; ++i)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  EXPECT_EQ(images_received_, 3);

  cam_sub_.shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_camera_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
