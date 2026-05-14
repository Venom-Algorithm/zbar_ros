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
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include "zbar_ros/barcode_reader_node.hpp"
#include <geometry_msgs/msg/point32.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

namespace zbar_ros
{
namespace
{
const auto kSensorDataQos = rclcpp::SensorDataQoS();
const auto kDetectionsQos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
constexpr char kZbarMonoEncoding[] = "Y800";
const cv::Scalar kPolygonColor(0, 255, 0);
const cv::Scalar kLabelColor(0, 255, 255);
constexpr double kQrCornerMatchMargin = 8.0;
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

int resolveAprilTagDictionaryId(
  const std::string & family, const rclcpp::Logger & logger)
{
  static const std::unordered_map<std::string, int> kFamilies = {
    {"tag16h5", cv::aruco::DICT_APRILTAG_16h5},
    {"tag25h9", cv::aruco::DICT_APRILTAG_25h9},
    {"tag36h10", cv::aruco::DICT_APRILTAG_36h10},
    {"tag36h11", cv::aruco::DICT_APRILTAG_36h11},
  };

  const auto it = kFamilies.find(family);
  if (it != kFamilies.end()) {
    return it->second;
  }

  RCLCPP_WARN(
    logger,
    "Unsupported AprilTag family %s; falling back to tag36h11",
    family.c_str());
  return cv::aruco::DICT_APRILTAG_36h11;
}

cv::Rect2f boundingRectFromPolygon(const zbar_ros::msg::BarcodeDetection & detection)
{
  if (detection.polygon.points.empty()) {
    return {};
  }

  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();
  for (const auto & point : detection.polygon.points) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }

  return cv::Rect2f(min_x, min_y, max_x - min_x, max_y - min_y);
}

bool cornersMatchRoughDetection(
  const std::vector<cv::Point2f> & corners,
  const zbar_ros::msg::BarcodeDetection & rough_detection)
{
  const auto rough_rect = boundingRectFromPolygon(rough_detection);
  if (rough_rect.empty()) {
    return true;
  }

  const auto corner_rect = cv::boundingRect(corners);
  const auto expanded_rect = rough_rect + cv::Size2f(
    static_cast<float>(kQrCornerMatchMargin * 2.0),
    static_cast<float>(kQrCornerMatchMargin * 2.0));
  const auto shifted_rect = cv::Rect2f(
    expanded_rect.x - static_cast<float>(kQrCornerMatchMargin),
    expanded_rect.y - static_cast<float>(kQrCornerMatchMargin),
    expanded_rect.width,
    expanded_rect.height);
  return (shifted_rect & cv::Rect2f(corner_rect)).area() > 0.0F;
}

std::optional<std::vector<cv::Point2f>> qrCornersFromPointsMat(
  const cv::Mat & points, int qr_index)
{
  if (points.empty() || points.type() != CV_32FC2) {
    return std::nullopt;
  }

  const int qr_count = points.rows;
  if (qr_index < 0 || qr_index >= qr_count || points.cols < 4) {
    return std::nullopt;
  }

  std::vector<cv::Point2f> corners;
  corners.reserve(4);
  for (int corner_index = 0; corner_index < 4; ++corner_index) {
    corners.push_back(points.at<cv::Point2f>(qr_index, corner_index));
  }
  return corners;
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
  use_reliable_image_qos_ = this->declare_parameter<bool>("use_reliable_image_qos", false);
  scanner_x_density_ = clampNonNegative(
    this->declare_parameter<int>("scanner_x_density", 1), "scanner_x_density", get_logger());
  scanner_y_density_ = clampNonNegative(
    this->declare_parameter<int>("scanner_y_density", 1), "scanner_y_density", get_logger());
  scan_scale_ = clampScanScale(this->declare_parameter<double>("scan_scale", 1.0), get_logger());
  enable_apriltag_ = this->declare_parameter<bool>("enable_apriltag", false);
  apriltag_family_ = this->declare_parameter<std::string>("apriltag_family", "tag36h11");

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

  if (enable_apriltag_) {
    apriltag_dictionary_ = cv::aruco::getPredefinedDictionary(
      resolveAprilTagDictionaryId(apriltag_family_, get_logger()));
    apriltag_detector_parameters_ = cv::aruco::DetectorParameters::create();
    apriltag_detector_parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
    RCLCPP_INFO(
      get_logger(), "AprilTag detection enabled with family %s", apriltag_family_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "AprilTag detection disabled");
  }

  rclcpp::QoS image_qos = rclcpp::SensorDataQoS();
  if (use_reliable_image_qos_) {
    image_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  }

  image_sub_ = this->create_subscription<ImageMsg>(
    "image", image_qos, std::bind(&BarcodeReaderNode::imageCb, this, std::placeholders::_1));

  detections_pub_ = this->create_publisher<BarcodeDetectionsMsg>("detections", kDetectionsQos);

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

  cv::Mat apriltag_image = mono_image->image;
  if (!apriltag_image.isContinuous()) {
    apriltag_image = apriltag_image.clone();
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
  if (enable_apriltag_) {
    detectAprilTags(apriltag_image, 1.0, 1.0, detections);
  }

  for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
    symbol != zbar_image.symbol_end(); ++symbol)
  {
    detections.push_back(buildDetection(*symbol, x_scale, y_scale));
    if (detections.back().symbology == "QR-Code") {
      const auto qr_corners = findQrCodeCorners(mono_image->image, detections.back());
      if (qr_corners.has_value()) {
        replacePolygonWithCorners(detections.back(), *qr_corners);
      } else {
        detections.back().polygon.points.clear();
      }
    }
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
  if (location_size == 0) {
    return detection;
  }

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (int index = 0; index < location_size; ++index) {
    const double x = symbol.get_location_x(index) * x_scale;
    const double y = symbol.get_location_y(index) * y_scale;
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
  }

  detection.polygon.points.reserve(4);
  for (const auto & point_xy : {
      std::pair<float, float>{static_cast<float>(min_x), static_cast<float>(min_y)},
      std::pair<float, float>{static_cast<float>(max_x), static_cast<float>(min_y)},
      std::pair<float, float>{static_cast<float>(max_x), static_cast<float>(max_y)},
      std::pair<float, float>{static_cast<float>(min_x), static_cast<float>(max_y)}})
  {
    geometry_msgs::msg::Point32 point;
    point.x = point_xy.first;
    point.y = point_xy.second;
    point.z = 0.0F;
    detection.polygon.points.push_back(point);
  }

  return detection;
}

std::optional<std::vector<cv::Point2f>> BarcodeReaderNode::findQrCodeCorners(
  const cv::Mat & image, const BarcodeDetectionMsg & rough_detection) const
{
  cv::Mat points;
  if (!qr_code_detector_.detectMulti(image, points) || points.empty()) {
    if (!qr_code_detector_.detect(image, points) || points.empty()) {
      return std::nullopt;
    }
  }

  if (points.rows == 1) {
    const auto corners = qrCornersFromPointsMat(points, 0);
    if (corners.has_value() && cornersMatchRoughDetection(*corners, rough_detection)) {
      return corners;
    }
    return std::nullopt;
  }

  const int qr_count = points.rows;
  for (int qr_index = 0; qr_index < qr_count; ++qr_index) {
    const auto corners = qrCornersFromPointsMat(points, qr_index);
    if (!corners.has_value()) {
      continue;
    }
    if (cornersMatchRoughDetection(*corners, rough_detection)) {
      return corners;
    }
  }

  return std::nullopt;
}

void BarcodeReaderNode::replacePolygonWithCorners(
  BarcodeDetectionMsg & detection, const std::vector<cv::Point2f> & corners) const
{
  detection.polygon.points.clear();
  detection.polygon.points.reserve(corners.size());
  for (const auto & corner : corners) {
    geometry_msgs::msg::Point32 point;
    point.x = corner.x;
    point.y = corner.y;
    point.z = 0.0F;
    detection.polygon.points.push_back(point);
  }
}

BarcodeReaderNode::BarcodeDetectionMsg BarcodeReaderNode::buildAprilTagDetection(
  int tag_id, const std::vector<cv::Point2f> & corners, double x_scale, double y_scale) const
{
  BarcodeDetectionMsg detection;
  detection.data = std::to_string(tag_id);
  detection.symbology = "APRILTAG";

  detection.polygon.points.reserve(corners.size());
  for (const auto & corner : corners) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(corner.x * x_scale);
    point.y = static_cast<float>(corner.y * y_scale);
    point.z = 0.0F;
    detection.polygon.points.push_back(point);
  }

  return detection;
}

void BarcodeReaderNode::detectAprilTags(
  const cv::Mat & scan_image, double x_scale, double y_scale,
  std::vector<BarcodeDetectionMsg> & detections)
{
  if (!apriltag_dictionary_ || !apriltag_detector_parameters_) {
    return;
  }

  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  cv::aruco::detectMarkers(
    scan_image, apriltag_dictionary_, corners, ids, apriltag_detector_parameters_);

  for (size_t index = 0; index < ids.size(); ++index) {
    detections.push_back(buildAprilTagDetection(ids[index], corners[index], x_scale, y_scale));
    RCLCPP_INFO(get_logger(), "Detected APRILTAG: %s", detections.back().data.c_str());
  }
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
