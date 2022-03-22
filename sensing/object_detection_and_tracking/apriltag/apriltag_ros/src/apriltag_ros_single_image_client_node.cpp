/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/common_functions.h"
#include <apriltag_ros/AnalyzeSingleImage.h>

bool getRosParameter (ros::NodeHandle& pnh, std::string name, double& param)
{
  // Write parameter "name" from ROS Parameter Server into param
  // Return true if successful, false otherwise
  if (pnh.hasParam(name.c_str()))
  {
    pnh.getParam(name.c_str(), param);
    ROS_INFO_STREAM("Set camera " << name.c_str() << " = " << param);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Could not find " << name.c_str() << " parameter!");
    return false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apriltag_ros_single_image_client");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::ServiceClient client =
      nh.serviceClient<apriltag_ros::AnalyzeSingleImage>(
          "single_image_tag_detection");

  // Get the request parameters
  apriltag_ros::AnalyzeSingleImage service;
  service.request.full_path_where_to_get_image =
      apriltag_ros::getAprilTagOption<std::string>(
          pnh, "image_load_path", "");
  if (service.request.full_path_where_to_get_image.empty())
  {
    return 1;
  }
  service.request.full_path_where_to_save_image =
      apriltag_ros::getAprilTagOption<std::string>(
          pnh, "image_save_path", "");
  if (service.request.full_path_where_to_save_image.empty())
  {
    return 1;
  }

  // Replicate sensors_msgs/CameraInfo message (must be up-to-date with the
  // analyzed image!)  
  service.request.camera_info.distortion_model = "plumb_bob";
  double fx, fy, cx, cy;
  if (!getRosParameter(pnh, "fx", fx))
    return 1;
  if (!getRosParameter(pnh, "fy", fy))
    return 1;
  if (!getRosParameter(pnh, "cx", cx))
    return 1;
  if (!getRosParameter(pnh, "cy", cy))
    return 1;
  // Intrinsic camera matrix for the raw (distorted) images
  service.request.camera_info.K[0] = fx;
  service.request.camera_info.K[2] = cx;
  service.request.camera_info.K[4] = fy;
  service.request.camera_info.K[5] = cy;
  service.request.camera_info.K[8] = 1.0;

  // Call the service (detect tags in the image specified by the
  // image_load_path)
  if (client.call(service))
  {
    // use parameter run_quielty=false in order to have the service
    // print out the tag position and orientation
    if (service.response.tag_detections.detections.size() == 0)
    {
      ROS_WARN_STREAM("No detected tags!");
    }
  }
  else
  {
    ROS_ERROR("Failed to call service single_image_tag_detection");
    return 1;
  }

  return 0; // happy ending
}
