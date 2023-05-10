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

#include "le_arm_grasping/vision_manager.h"

VisionManager::VisionManager(float length, float breadth)
{
	this->table_length = length;
	this->table_breadth = breadth;
}

void VisionManager::get2DLocation(cv::Mat img, float &x, float &y)
{
	this->curr_img = img;
	img_centre_x_ = img.rows / 2;
	img_centre_y_ = img.cols / 2;

	cv::Rect tablePos;

	detectTable(tablePos);

	detect2DObject(x, y, tablePos);
	convertToMM(x, y);
}

void VisionManager::detectTable(cv::Rect &tablePos)
{
	// Extract Table from the image and assign values to pixel_per_mm fields
	cv::Mat BGR[3];
	cv::Mat image = curr_img.clone();
	split(image, BGR);
	cv::Mat gray_image_red = BGR[2];
	cv::Mat gray_image_green = BGR[1];
	cv::Mat denoiseImage;
	cv::medianBlur(gray_image_red, denoiseImage, 3);

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

	// Get the centroid of the of the blob
	std::vector<cv::Point> nonZeroPoints;
	cv::findNonZero(binaryImage, nonZeroPoints);
	cv::Rect bbox = cv::boundingRect(nonZeroPoints);
	cv::Point pt;
	pt.x = bbox.x + bbox.width / 2;
	pt.y = bbox.y + bbox.height / 2;
	cv::circle(image, pt, 2, cv::Scalar(0, 0, 255), -1, 8);

	// Update pixels_per_mm fields
	pixels_permm_y = bbox.height / table_length;
	pixels_permm_x = bbox.width  / table_breadth;

    tablePos = bbox;

	// Test the conversion values
	std::cout << "Pixels in y" << pixels_permm_y << std::endl;
	std::cout << "Pixels in x" << pixels_permm_x << std::endl;

	// Draw Contours - For Debugging
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(255, 0, 0);
		cv::drawContours(image, contours, i, color, 1, 8, hierarchy, 0, cv::Point());
	}

	// cv::namedWindow("Table Detection", cv::WINDOW_AUTOSIZE);
	// cv::imshow("Table Detection", image);
	// cv::waitKey(100);
}

void VisionManager::detect2DObject(float &pixel_x, float &pixel_y, cv::Rect &tablePos)
{
	// Implement Color Thresholding and contour findings to get the location of object to be grasped in 2D
	cv::Mat image, gray_image_green;
	cv::Mat BGR[3];
	image = curr_img.clone();
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
/*			if((j<tablePos.x+3) || j>(tablePos.x+tablePos.width-3) || (i<tablePos.y+3) || i>(tablePos.y + tablePos.height-3))
			{
				binaryImage.at<uchar>(i, j) = 0;
			}
			else
			{
				int editValue = binaryImage.at<uchar>(i, j);

				if ((editValue > 100) && (editValue <= 255))
				{ // check whether value is within range.
					binaryImage.at<uchar>(i, j) = 255;
				}
				else
				{
					binaryImage.at<uchar>(i, j) = 0;
				}
			}
*/
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
	cv::circle(image, pt, 2, cv::Scalar(0, 0, 255), -1, 8);

	// Draw Contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(255, 0, 0);
		cv::drawContours(image, contours, i, color, 1, 8, hierarchy, 0, cv::Point());
	}

	// cv::namedWindow("Centre point", cv::WINDOW_AUTOSIZE);
	// cv::imshow("Centre point", image);
	// cv::waitKey(100);
}

void VisionManager::convertToMM(float &x, float &y)
{
	// Convert from pixel to world co-ordinates in the camera frame
	x = (x - img_centre_x_) / pixels_permm_x;
	y = (y - img_centre_y_) / pixels_permm_y;
}

// Temporary Main Function for testing- This should go away later
// int main(int argc, char** argv ) {
// 	if ( argc != 2 )
//     {
//         printf("usage: VisionManager <Image_Path>\n");
//         return -1;
//     }

//     cv::Mat image;
//     image = cv::imread( argv[1], 1 );

//     if ( !image.data )
//     {
//         printf("No image data \n");
//         return -1;
//     }

//     float length = 0.1;
//     float breadth = 0.1;
//     float obj_x, obj_y;

//     VisionManager vm(length, breadth);
//     vm.get2DLocation(image, obj_x, obj_y);
//     std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
//     std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

//     cv::waitKey(0);
// }
