/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/
#ifndef ZBAR_ROS__BARCODE_READER_NODE_HPP_
#define ZBAR_ROS__BARCODE_READER_NODE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "opencv2/aruco.hpp"
#include "opencv2/objdetect.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "zbar_ros/msg/barcode_detection.hpp"
#include "zbar_ros/msg/barcode_detections.hpp"
#include <zbar.h>

namespace zbar_ros
{

class BarcodeReaderNode : public rclcpp::Node
{
public:
  BarcodeReaderNode();

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using BarcodeDetectionMsg = zbar_ros::msg::BarcodeDetection;
  using BarcodeDetectionsMsg = zbar_ros::msg::BarcodeDetections;

  void imageCb(ImageMsg::ConstSharedPtr image);
  BarcodeDetectionMsg buildDetection(
    const zbar::Symbol & symbol, double x_scale, double y_scale) const;
  BarcodeDetectionMsg buildAprilTagDetection(
    int tag_id, const std::vector<cv::Point2f> & corners, double x_scale, double y_scale) const;
  std::optional<std::vector<cv::Point2f>> findQrCodeCorners(
    const cv::Mat & image, const BarcodeDetectionMsg & rough_detection) const;
  void replacePolygonWithCorners(
    BarcodeDetectionMsg & detection, const std::vector<cv::Point2f> & corners) const;
  void detectAprilTags(
    const cv::Mat & scan_image, double x_scale, double y_scale,
    std::vector<BarcodeDetectionMsg> & detections);
  void publishDebugImage(
    const ImageMsg::ConstSharedPtr & image,
    const std::vector<BarcodeDetectionMsg> & detections);

  rclcpp::Subscription<ImageMsg>::SharedPtr image_sub_;
  rclcpp::Publisher<BarcodeDetectionsMsg>::SharedPtr detections_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr debug_image_pub_;
  zbar::ImageScanner scanner_;
  cv::QRCodeDetector qr_code_detector_;
  bool publish_debug_image_{true};
  bool publish_empty_detections_{true};
  bool qrcode_only_{true};
  bool try_inverted_{false};
  bool equalize_histogram_{false};
  bool use_reliable_image_qos_{false};
  int scanner_x_density_{1};
  int scanner_y_density_{1};
  double scan_scale_{1.0};
  bool enable_apriltag_{false};
  std::string apriltag_family_{"tag36h11"};
  cv::Ptr<cv::aruco::Dictionary> apriltag_dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> apriltag_detector_parameters_;
};

}  // namespace zbar_ros

#endif  // ZBAR_ROS__BARCODE_READER_NODE_HPP_
