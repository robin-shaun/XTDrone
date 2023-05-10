#include <gazebo_test_tools/FakeObjectRecognizer.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_object_recognizer");
    gazebo_test_tools::FakeObjectRecognizer recognizer;
    ros::spin();
    return 0;
}
