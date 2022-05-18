#include "distortion.h"

TEST_F(DistortionTest, barrelDistortion)
{
  cameraDistortionTest();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_camera_barrel_distortion_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
