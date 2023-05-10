/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include "le_arm_vision/vision_manager.h"

VisionManager::VisionManager(ros::NodeHandle n_, float length, float breadth) : it_(n_)
{
	this->table_length = length;
	this->table_breadth = breadth;

  	// Subscribe to input video feed and publish object location
  	image_sub_  = it_.subscribe("/iris_0/le_arm/camera/image_raw", 1, &VisionManager::imageCb, this);
	image1_pub_ = it_.advertise("/table_detect", 1);
	image2_pub_ = it_.advertise("/object_detect", 1);
	ROS_INFO_STREAM("Processing 1");
}

void VisionManager::get2DLocation(const sensor_msgs::ImageConstPtr &msg, float &x, float &y)
{
	cv::Rect tablePos;
	detectTable(msg, tablePos);

	detect2DObject(msg, x, y, tablePos);
	convertToMM(x, y);
	ROS_INFO_STREAM("Processing 2");
}

void VisionManager::detectTable(const sensor_msgs::ImageConstPtr &msg, cv::Rect &tablePos)
{
	// Extract Table from the image and assign values to pixel_per_mm fields
    cv::Mat BGR[3];
    
	try
    {
      cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    ROS_INFO_STREAM("Processing 3");

	cv::Mat &image = cv_ptr_->image;

	split(image, BGR);
	cv::Mat gray_image_red = BGR[2];
	cv::Mat gray_image_green = BGR[1];
	cv::Mat denoiseImage;
	cv::medianBlur(gray_image_red, denoiseImage, 3);
	ROS_INFO_STREAM("Processing 4");

	// Threshold the Image
	cv::Mat binaryImage = denoiseImage;
	for (int i = 0; i < binaryImage.rows; i++)
	{
		for (int j = 0; j < binaryImage.cols; j++)
		{
			int editValue = binaryImage.at<uchar>(i, j);
			int editValue2 = gray_image_green.at<uchar>(i, j);

			if ((editValue >= 0) && (editValue < 20) && (editValue2 >= 0) && (editValue2 < 20))
			{ // check whether value is within range.
				binaryImage.at<uchar>(i, j) = 255;
			}
			else
			{
				binaryImage.at<uchar>(i, j) = 0;
			}
		}
	}
	dilate(binaryImage, binaryImage, cv::Mat());
	ROS_INFO_STREAM("Processing 5");

	// Get the centroid of the of the blob
	std::vector<cv::Point> nonZeroPoints;
	cv::findNonZero(binaryImage, nonZeroPoints);
	cv::Rect bbox = cv::boundingRect(nonZeroPoints);
	cv::Point pt;
	pt.x = bbox.x + bbox.width / 2;
	pt.y = bbox.y + bbox.height / 2;
	cv::circle(image, pt, 4, cv::Scalar(0, 0, 255), -1, 8);

	// Update pixels_per_mm fields
	pixels_permm_y = bbox.height / table_length;
	pixels_permm_x = bbox.width  / table_breadth;

    tablePos = bbox;

	// Test the conversion values
	std::cout << "Pixels in y" << pixels_permm_y << std::endl;
	std::cout << "Pixels in x" << pixels_permm_x << std::endl;
	ROS_INFO_STREAM("Processing 6");

	// Draw Contours - For Debugging
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	ROS_INFO_STREAM("Processing 8");

	cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(255, 0, 0);
		cv::drawContours(image, contours, i, color, 3, 8, hierarchy, 0, cv::Point());
	}

	// Output modified video stream
 	image1_pub_.publish(cv_ptr_->toImageMsg());
}

void VisionManager::detect2DObject(const sensor_msgs::ImageConstPtr &msg, float &pixel_x, float &pixel_y, cv::Rect &tablePos)
{
	// Implement Color Thresholding and contour findings to get the location of object to be grasped in 2D
	cv::Mat gray_image_green;
	cv::Mat BGR[3];
    
	try
    {
      cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	cv::Mat &image = cv_ptr_->image;

	cv::split(image, BGR);

	gray_image_green = BGR[1];

	// Denoise the Image
	cv::Mat denoiseImage;
	cv::medianBlur(gray_image_green, denoiseImage, 3);

	// Threshold the Image
	cv::Mat binaryImage = denoiseImage;
	for (int i = 0; i < binaryImage.rows; i++)
	{
		for (int j = 0; j < binaryImage.cols; j++)
		{
			int editValue = binaryImage.at<uchar>(i, j);

			if ((editValue > 200) && (editValue <= 255))
			{ // check whether value is within range.
				binaryImage.at<uchar>(i, j) = 255;
			}
			else
			{
				binaryImage.at<uchar>(i, j) = 0;
			}
		}
	}
	dilate(binaryImage, binaryImage, cv::Mat());
	ROS_INFO_STREAM("Processing 7");

	// Get the centroid of the of the blob
	std::vector<cv::Point> nonZeroPoints;
	cv::findNonZero(binaryImage, nonZeroPoints);
	cv::Rect bbox = cv::boundingRect(nonZeroPoints);
	cv::Point pt;
	pixel_x = bbox.x + bbox.width / 2;
	pixel_y = bbox.y + bbox.height / 2;

	// Test the conversion values
	std::cout << "pixel_x" << pixel_x << std::endl;
	std::cout << "pixel_y" << pixel_y << std::endl;

	// For Drawing
	pt.x = bbox.x + bbox.width / 2;
	pt.y = bbox.y + bbox.height / 2;
	cv::circle(image, pt, 4, cv::Scalar(0, 0, 255), -1, 8);

	// Draw Contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(255, 0, 0);
		cv::drawContours(image, contours, i, color, 3, 8, hierarchy, 0, cv::Point());
	}

	// Output modified video stream
 	image2_pub_.publish(cv_ptr_->toImageMsg());
}

void VisionManager::convertToMM(float &x, float &y)
{
	img_centre_x_ = 640 / 2;
	img_centre_y_ = 480 / 2;

	// Convert from pixel to world co-ordinates in the camera frame
	x = (x - img_centre_x_) / pixels_permm_x;
	y = (y - img_centre_y_) / pixels_permm_y;
}


void VisionManager::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO_STREAM("Processing the Image to locate the Object...");

    // ROS_INFO("Image Message Received");
    float obj_x, obj_y;
    get2DLocation(msg, obj_x, obj_y);

    // Temporary Debugging
    std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
    std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;
}

// Temporary Main Function for testing- This should go away later
int main(int argc, char** argv ) 
{
  	ros::init(argc, argv, "simple_grasping_vision_detection");
  	ros::NodeHandle n_;

  	ROS_INFO_STREAM("Waiting for two seconds..");
  	ros::WallDuration(2.0).sleep();

	float length = 0.1;
	float breadth = 0.1;

	VisionManager vm(n_, length, breadth);

	while (ros::ok())
	{
		// Process image callback
		ros::spinOnce();

		ros::WallDuration(2.0).sleep();
	}
	return 0;
}
