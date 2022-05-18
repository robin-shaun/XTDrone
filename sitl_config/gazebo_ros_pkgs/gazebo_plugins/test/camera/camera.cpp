#include "camera.h"

// Test if the camera image is published at all, and that the timestamp
// is not too long in the past.
TEST_F(CameraTest, cameraSubscribeTest)
{
  subscribeTest();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_camera_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
