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
 *
 ** common_functions.h *********************************************************
 *
 * Wrapper classes for AprilTag standalone and bundle detection. Main function
 * is TagDetector::detectTags which wraps the call to core AprilTag 2
 * algorithm, apriltag_detector_detect().
 *
 * $Revision: 1.0 $
 * $Date: 2017/12/17 13:23:14 $
 * $Author: dmalyuta $
 *
 * Originator:        Danylo Malyuta, JPL
 ******************************************************************************/

#ifndef APRILTAG_ROS_COMMON_FUNCTIONS_H
#define APRILTAG_ROS_COMMON_FUNCTIONS_H

#include <string>
#include <sstream>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/console.h>
#include <XmlRpcException.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

#include <apriltag.h>

#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

namespace apriltag_ros
{

template<typename T>
T getAprilTagOption(ros::NodeHandle& pnh,
                    const std::string& param_name, const T & default_val)
{
  T param_val;
  pnh.param<T>(param_name, param_val, default_val);
  return param_val;
}

// Stores the properties of a tag member of a bundle
struct TagBundleMember
{
  int id; // Payload ID
  double size; // [m] Side length
  cv::Matx44d T_oi; // Rigid transform from tag i frame to bundle origin frame
};

class StandaloneTagDescription
{
 public:
  StandaloneTagDescription() {};
  StandaloneTagDescription(int id, double size,
                           std::string &frame_name) :
      id_(id),
      size_(size),
      frame_name_(frame_name) {}

  double size() { return size_; }
  int id() { return id_; }
  std::string& frame_name() { return frame_name_; }

 private:
  // Tag description
  int id_;
  double size_;
  std::string frame_name_;
};

class TagBundleDescription
{
 public:
  std::map<int, int > id2idx_; // (id2idx_[<tag ID>]=<index in tags_>) mapping

  TagBundleDescription(std::string name) :
      name_(name) {}

  void addMemberTag(int id, double size, cv::Matx44d T_oi) {
    TagBundleMember member;
    member.id = id;
    member.size = size;
    member.T_oi = T_oi;
    tags_.push_back(member);
    id2idx_[id] = tags_.size()-1;
  }

  std::string name () const { return name_; }
  // Get IDs of bundle member tags
  std::vector<int> bundleIds () {
    std::vector<int> ids;
    for (unsigned int i = 0; i < tags_.size(); i++) {
      ids.push_back(tags_[i].id);
    }
    return ids;
  }
  // Get sizes of bundle member tags
  std::vector<double> bundleSizes () {
    std::vector<double> sizes;
    for (unsigned int i = 0; i < tags_.size(); i++) {
      sizes.push_back(tags_[i].size);
    }
    return sizes;
  }
  int memberID (int tagID) { return tags_[id2idx_[tagID]].id; }
  double memberSize (int tagID) { return tags_[id2idx_[tagID]].size; }
  cv::Matx44d memberT_oi (int tagID) { return tags_[id2idx_[tagID]].T_oi; }

 private:
  // Bundle description
  std::string name_;
  std::vector<TagBundleMember > tags_;
};

class TagDetector
{
 private:
  // Detections sorting
  static int idComparison(const void* first, const void* second);

  // Remove detections of tags with the same ID
  void removeDuplicates();

  // AprilTag 2 code's attributes
  std::string family_;
  int threads_;
  double decimate_;
  double blur_;
  int refine_edges_;
  int debug_;

  // AprilTag 2 objects
  apriltag_family_t *tf_;
  apriltag_detector_t *td_;
  zarray_t *detections_;

  // Other members
  std::map<int, StandaloneTagDescription> standalone_tag_descriptions_;
  std::vector<TagBundleDescription > tag_bundle_descriptions_;
  bool remove_duplicates_;
  bool run_quietly_;
  bool publish_tf_;
  tf::TransformBroadcaster tf_pub_;
  std::string camera_tf_frame_;

 public:

  TagDetector(ros::NodeHandle pnh);
  ~TagDetector();

  // Store standalone and bundle tag descriptions
  std::map<int, StandaloneTagDescription> parseStandaloneTags(
      XmlRpc::XmlRpcValue& standalone_tag_descriptions);
  std::vector<TagBundleDescription > parseTagBundles(
      XmlRpc::XmlRpcValue& tag_bundles);
  double xmlRpcGetDouble(
      XmlRpc::XmlRpcValue& xmlValue, std::string field) const;
  double xmlRpcGetDoubleWithDefault(
      XmlRpc::XmlRpcValue& xmlValue, std::string field,
      double defaultValue) const;

  bool findStandaloneTagDescription(
      int id, StandaloneTagDescription*& descriptionContainer,
      bool printWarning = true);

  geometry_msgs::PoseWithCovarianceStamped makeTagPose(
      const Eigen::Matrix4d& transform,
      const Eigen::Quaternion<double> rot_quaternion,
      const std_msgs::Header& header);

  // Detect tags in an image
  AprilTagDetectionArray detectTags(
      const cv_bridge::CvImagePtr& image,
      const sensor_msgs::CameraInfoConstPtr& camera_info);

  // Get the pose of the tag in the camera frame
  // Returns homogeneous transformation matrix [R,t;[0 0 0 1]] which
  // takes a point expressed in the tag frame to the same point
  // expressed in the camera frame. As usual, R is the (passive)
  // rotation from the tag frame to the camera frame and t is the
  // vector from the camera frame origin to the tag frame origin,
  // expressed in the camera frame.
  Eigen::Matrix4d getRelativeTransform(
      std::vector<cv::Point3d > objectPoints,
      std::vector<cv::Point2d > imagePoints,
      double fx, double fy, double cx, double cy) const;
  
  void addImagePoints(apriltag_detection_t *detection,
                      std::vector<cv::Point2d >& imagePoints) const;
  void addObjectPoints(double s, cv::Matx44d T_oi,
                       std::vector<cv::Point3d >& objectPoints) const;

  // Draw the detected tags' outlines and payload values on the image
  void drawDetections(cv_bridge::CvImagePtr image);
};

} // namespace apriltag_ros

#endif // APRILTAG_ROS_COMMON_FUNCTIONS_H
