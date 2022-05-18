#ifndef GAZEBO_PLUGINS_TEST_CAMERA_DISTORTION_H
#define GAZEBO_PLUGINS_TEST_CAMERA_DISTORTION_H

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Test includes
#include <gtest/gtest.h>

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

void diffBetween(Mat& orig, Mat& diff, long& total_diff)
{
  MatIterator_<Vec3b> it, end;
  Vec3b orig_pixel, diff_pixel;
  total_diff = 0;

  for(int i=0; i<orig.rows; i++)
  {
    for(int j=0; j<orig.cols; j++)
    {
      orig_pixel = orig.at<cv::Vec3b>(i,j);
      diff_pixel = diff.at<cv::Vec3b>(i,j);
      total_diff += abs(orig_pixel[0] - diff_pixel[0]) +
                    abs(orig_pixel[1] - diff_pixel[1]) +
                    abs(orig_pixel[2] - diff_pixel[2]);
    }
  }
}

class DistortionTest : public testing::Test
{
 protected:
  ros::NodeHandle nh_;

  // Used to listen for images
  image_transport::Subscriber cam_sub_distorted_;
  image_transport::Subscriber cam_sub_undistorted_;

  // Stores found images
  sensor_msgs::ImageConstPtr cam_image_distorted_;
  sensor_msgs::ImageConstPtr cam_image_undistorted_;

  // Listens for camera metadata to be published
  ros::Subscriber cam_info_distorted_sub_;
  // Stores received camera metadata
  sensor_msgs::CameraInfoConstPtr cam_info_distorted_;

 public:
  void cameraDistortionTest();

  void imageCallback(const sensor_msgs::ImageConstPtr& msg, int cam_index)
  {
    // for now, only support 2 cameras
    assert(cam_index == 0 || cam_index == 1);
    if(cam_index == 0)
    {
      cam_image_undistorted_ = msg;
    }
    else
    {
      cam_image_distorted_ = msg;
    }
  }
  void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    cam_info_distorted_ = msg;
  }
};

void DistortionTest::cameraDistortionTest()
{
  ros::AsyncSpinner spinner(2);
  spinner.start();

  image_transport::ImageTransport trans(nh_);
  cam_sub_undistorted_ =
      trans.subscribe("/camera_undistorted/image_raw",
                      1,
                      boost::bind(&DistortionTest::imageCallback,
                      dynamic_cast<DistortionTest*>(this), _1, 0)
                     );

  cam_info_distorted_ = nullptr;
  cam_image_distorted_ = nullptr;
  // acquire information from ROS topics
  cam_sub_distorted_ =
      trans.subscribe("/camera_distorted/image_raw",
                      1,
                      boost::bind(&DistortionTest::imageCallback,
                      dynamic_cast<DistortionTest*>(this), _1, 1)
                     );
  cam_info_distorted_sub_ =
      nh_.subscribe("/camera_distorted/camera_info",
                    1,
                    &DistortionTest::camInfoCallback,
                    dynamic_cast<DistortionTest*>(this)
                   );

  // keep waiting until we have an image

  if(cam_info_distorted_ && cam_image_distorted_) {
    std::cerr << "available immediately" << std::endl;
  }
  while (!cam_info_distorted_ ||
      !cam_image_distorted_ ||
      !cam_image_undistorted_)
  {
    ros::Duration(0.1).sleep();
  }
  cam_sub_undistorted_.shutdown();
  cam_sub_distorted_.shutdown();
  cam_info_distorted_sub_.shutdown();

  // load camera coefficients from published ROS information
  Mat intrinsic_distorted_matrix = Mat(3, 3, CV_64F);
  if(cam_info_distorted_->K.size() == 9)
  {
    memcpy(intrinsic_distorted_matrix.data, cam_info_distorted_->K.data(),
      cam_info_distorted_->K.size()*sizeof(double));
  }
  Mat distortion_coeffs = Mat(5, 1, CV_64F);
  if(cam_info_distorted_->D.size() == 5)
  {
    memcpy(distortion_coeffs.data, cam_info_distorted_->D.data(),
      cam_info_distorted_->D.size()*sizeof(double));
  }

  // Information acquired, now test the quality of the undistortion

  Mat distorted = Mat(cv_bridge::toCvCopy(cam_image_distorted_)->image);
  Mat fixed = distorted.clone();
  Mat undistorted = Mat(cv_bridge::toCvCopy(cam_image_undistorted_)->image);

  //crop the image to remove black borders leftover from (un)distortion
  int cropBorder = 50;
  cv::Rect myROI(cropBorder, cropBorder,
    fixed.rows - 2 * cropBorder, fixed.cols - 2 * cropBorder);
  cv::Mat fixed_crop = fixed(myROI);
  cv::Mat undistorted_crop = undistorted(myROI);


  undistort(distorted, fixed, intrinsic_distorted_matrix, distortion_coeffs);

  //Ensure that we didn't crop away everything
  ASSERT_GT(distorted.rows, 0);
  ASSERT_GT(distorted.cols, 0);
  ASSERT_GT(undistorted.rows, 0);
  ASSERT_GT(undistorted.cols, 0);
  ASSERT_GT(fixed.rows, 0);
  ASSERT_GT(fixed.cols, 0);

  // The difference between the undistorted image and the no-distortion camera
  // image should be the lowest when we use the correct distortion parameters.
  long diff1 = 0, diff2 = 0;
  diffBetween(fixed_crop, undistorted_crop, diff1);

  const double OFFSET = 0.01;

  // test each parameter, varying one at a time
  for(size_t i = 0; i < 5; ++i)
  {
    distortion_coeffs.at<double>(i,0) += OFFSET;
    undistort(distorted, fixed, intrinsic_distorted_matrix, distortion_coeffs);
    diffBetween(fixed_crop, undistorted_crop, diff2);
    EXPECT_GE(diff2, diff1);
    distortion_coeffs.at<double>(i,0) -= OFFSET;

    distortion_coeffs.at<double>(i,0) -= OFFSET;
    undistort(distorted, fixed, intrinsic_distorted_matrix, distortion_coeffs);
    diffBetween(fixed_crop, undistorted_crop, diff2);
    EXPECT_GE(diff2, diff1);
    distortion_coeffs.at<double>(i,0) += OFFSET;
  }
}

#endif
