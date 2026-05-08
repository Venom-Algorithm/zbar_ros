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
#include <functional>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include "zbar_ros/barcode_reader_node.hpp"
#include <geometry_msgs/msg/point32.hpp>
#include <opencv2/imgproc.hpp>

namespace zbar_ros
{
namespace
{
const auto kSensorDataQos = rclcpp::SensorDataQoS();
constexpr char kZbarMonoEncoding[] = "Y800";
const cv::Scalar kPolygonColor(0, 255, 0);
const cv::Scalar kLabelColor(0, 255, 255);
const zbar::zbar_symbol_type_t kCommonSymbologies[] = {
  zbar::ZBAR_QRCODE,
  zbar::ZBAR_EAN13,
  zbar::ZBAR_EAN8,
  zbar::ZBAR_UPCA,
  zbar::ZBAR_UPCE,
  zbar::ZBAR_CODE128,
  zbar::ZBAR_CODE93,
  zbar::ZBAR_CODE39,
  zbar::ZBAR_I25,
  zbar::ZBAR_CODABAR,
};

int clampNonNegative(int value, const char * parameter_name, const rclcpp::Logger & logger)
{
  if (value >= 0) {
    return value;
  }

  RCLCPP_WARN(
    logger,
    "Parameter %s must be non-negative; using 0 instead of %d",
    parameter_name,
    value);
  return 0;
}

double clampScanScale(double value, const rclcpp::Logger & logger)
{
  if (value > 0.0 && value <= 1.0) {
    return value;
  }

  RCLCPP_WARN(
    logger,
    "Parameter scan_scale must be in (0.0, 1.0]; using 1.0 instead of %.3f",
    value);
  return 1.0;
}
}  // namespace

BarcodeReaderNode::BarcodeReaderNode()
: Node("qr_code_detector")
{
  publish_debug_image_ = this->declare_parameter<bool>("publish_debug_image", true);
  publish_empty_detections_ = this->declare_parameter<bool>("publish_empty_detections", true);
  qrcode_only_ = this->declare_parameter<bool>("qrcode_only", true);
  try_inverted_ = this->declare_parameter<bool>("try_inverted", false);
  equalize_histogram_ = this->declare_parameter<bool>("equalize_histogram", false);
  scanner_x_density_ = clampNonNegative(
    this->declare_parameter<int>("scanner_x_density", 1), "scanner_x_density", get_logger());
  scanner_y_density_ = clampNonNegative(
    this->declare_parameter<int>("scanner_y_density", 1), "scanner_y_density", get_logger());
  scan_scale_ = clampScanScale(this->declare_parameter<double>("scan_scale", 1.0), get_logger());

  scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  if (qrcode_only_) {
    scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
  } else {
    for (const auto symbology : kCommonSymbologies) {
      scanner_.set_config(symbology, zbar::ZBAR_CFG_ENABLE, 1);
    }
  }
  scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_X_DENSITY, scanner_x_density_);
  scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_Y_DENSITY, scanner_y_density_);
  scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_POSITION, 1);
  scanner_.set_config(
    zbar::ZBAR_NONE, zbar::ZBAR_CFG_TEST_INVERTED, try_inverted_ ? 1 : 0);

  image_sub_ = this->create_subscription<ImageMsg>(
    "image", kSensorDataQos, std::bind(&BarcodeReaderNode::imageCb, this, std::placeholders::_1));

  detections_pub_ = this->create_publisher<BarcodeDetectionsMsg>("detections", kSensorDataQos);

  if (publish_debug_image_) {
    debug_image_pub_ = this->create_publisher<ImageMsg>("debug_image", kSensorDataQos);
  }
}

void BarcodeReaderNode::imageCb(ImageMsg::ConstSharedPtr image)
{
  cv_bridge::CvImagePtr mono_image;
  try {
    mono_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Failed to convert image to mono8 for ZBar decoding: %s", ex.what());
    return;
  }

  cv::Mat scan_image = mono_image->image;
  cv::Mat preprocessed_image;
  if (equalize_histogram_) {
    cv::equalizeHist(scan_image, preprocessed_image);
    scan_image = preprocessed_image;
  }

  cv::Mat resized_image;
  if (scan_scale_ < 1.0) {
    cv::resize(scan_image, resized_image, cv::Size(), scan_scale_, scan_scale_, cv::INTER_AREA);
    scan_image = resized_image;
  }

  if (!scan_image.isContinuous()) {
    scan_image = scan_image.clone();
  }

  zbar::Image zbar_image(
    scan_image.cols,
    scan_image.rows,
    kZbarMonoEncoding,
    scan_image.data,
    scan_image.total() * scan_image.elemSize());
  scanner_.scan(zbar_image);

  BarcodeDetectionsMsg detections_msg;
  detections_msg.header = image->header;

  const double x_scale = static_cast<double>(mono_image->image.cols) / scan_image.cols;
  const double y_scale = static_cast<double>(mono_image->image.rows) / scan_image.rows;

  std::vector<BarcodeDetectionMsg> detections;
  for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
    symbol != zbar_image.symbol_end(); ++symbol)
  {
    detections.push_back(buildDetection(*symbol, x_scale, y_scale));
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Detected %s: %s",
      detections.back().symbology.c_str(),
      detections.back().data.c_str());
  }
  detections_msg.detections = detections;
  if (publish_empty_detections_ || !detections.empty()) {
    detections_pub_->publish(detections_msg);
  }

  if (publish_debug_image_ && debug_image_pub_) {
    publishDebugImage(image, detections);
  }

  zbar_image.set_data(nullptr, 0);
}

BarcodeReaderNode::BarcodeDetectionMsg BarcodeReaderNode::buildDetection(
  const zbar::Symbol & symbol, double x_scale, double y_scale) const
{
  BarcodeDetectionMsg detection;
  detection.data = symbol.get_data();
  detection.symbology = symbol.get_type_name();

  const auto location_size = symbol.get_location_size();
  detection.polygon.points.reserve(location_size);
  for (int index = 0; index < location_size; ++index) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(symbol.get_location_x(index) * x_scale);
    point.y = static_cast<float>(symbol.get_location_y(index) * y_scale);
    point.z = 0.0F;
    detection.polygon.points.push_back(point);
  }

  return detection;
}

void BarcodeReaderNode::publishDebugImage(
  const ImageMsg::ConstSharedPtr & image,
  const std::vector<BarcodeDetectionMsg> & detections)
{
  cv_bridge::CvImagePtr debug_image;
  try {
    debug_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Failed to convert image to bgr8 for debug output: %s", ex.what());
    return;
  }

  for (const auto & detection : detections) {
    std::vector<cv::Point> polygon;
    polygon.reserve(detection.polygon.points.size());
    for (const auto & point : detection.polygon.points) {
      polygon.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
    }

    if (polygon.size() >= 2) {
      cv::polylines(debug_image->image, polygon, true, kPolygonColor, 2);
      cv::putText(
        debug_image->image,
        detection.data,
        polygon.front(),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        kLabelColor,
        1);
    }
  }

  auto debug_msg = debug_image->toImageMsg();
  debug_image_pub_->publish(*debug_msg);
}

}  // namespace zbar_ros
