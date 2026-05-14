#include <algorithm>
#include <cstdint>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "zbar_ros/msg/barcode_detection.hpp"
#include "zbar_ros/msg/barcode_detections.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace zbar_ros
{
namespace
{
using BarcodeDetectionsMsg = zbar_ros::msg::BarcodeDetections;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using ImageMsg = sensor_msgs::msg::Image;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

std::vector<cv::Point3f> buildObjectPoints(double target_size_m)
{
  const auto half_size = static_cast<float>(target_size_m * 0.5);
  return {
    {-half_size, half_size, 0.0F},
    {half_size, half_size, 0.0F},
    {half_size, -half_size, 0.0F},
    {-half_size, -half_size, 0.0F},
  };
}

std::vector<cv::Point2f> toImagePoints(const zbar_ros::msg::BarcodeDetection & detection)
{
  std::vector<cv::Point2f> image_points;
  image_points.reserve(detection.polygon.points.size());
  for (const auto & point : detection.polygon.points) {
    image_points.emplace_back(point.x, point.y);
  }
  return image_points;
}

cv::Mat cameraMatrixFromInfo(const CameraInfoMsg & camera_info)
{
  return (cv::Mat_<double>(3, 3) <<
    camera_info.k[0], camera_info.k[1], camera_info.k[2],
    camera_info.k[3], camera_info.k[4], camera_info.k[5],
    camera_info.k[6], camera_info.k[7], camera_info.k[8]);
}

cv::Mat distortionFromInfo(const CameraInfoMsg & camera_info)
{
  if (camera_info.d.empty()) {
    return cv::Mat();
  }

  cv::Mat distortion(1, static_cast<int>(camera_info.d.size()), CV_64F);
  for (size_t index = 0; index < camera_info.d.size(); ++index) {
    distortion.at<double>(0, static_cast<int>(index)) = camera_info.d[index];
  }
  return distortion;
}

double quaternionNorm(const cv::Vec4d & quaternion)
{
  return std::sqrt(
    quaternion[0] * quaternion[0] +
    quaternion[1] * quaternion[1] +
    quaternion[2] * quaternion[2] +
    quaternion[3] * quaternion[3]);
}

cv::Vec4d rotationMatrixToQuaternion(const cv::Mat & rotation_matrix)
{
  const double m00 = rotation_matrix.at<double>(0, 0);
  const double m01 = rotation_matrix.at<double>(0, 1);
  const double m02 = rotation_matrix.at<double>(0, 2);
  const double m10 = rotation_matrix.at<double>(1, 0);
  const double m11 = rotation_matrix.at<double>(1, 1);
  const double m12 = rotation_matrix.at<double>(1, 2);
  const double m20 = rotation_matrix.at<double>(2, 0);
  const double m21 = rotation_matrix.at<double>(2, 1);
  const double m22 = rotation_matrix.at<double>(2, 2);

  cv::Vec4d quaternion;
  const double trace = m00 + m11 + m22;
  if (trace > 0.0) {
    const double scale = std::sqrt(trace + 1.0) * 2.0;
    quaternion[3] = 0.25 * scale;
    quaternion[0] = (m21 - m12) / scale;
    quaternion[1] = (m02 - m20) / scale;
    quaternion[2] = (m10 - m01) / scale;
  } else if (m00 > m11 && m00 > m22) {
    const double scale = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    quaternion[3] = (m21 - m12) / scale;
    quaternion[0] = 0.25 * scale;
    quaternion[1] = (m01 + m10) / scale;
    quaternion[2] = (m02 + m20) / scale;
  } else if (m11 > m22) {
    const double scale = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    quaternion[3] = (m02 - m20) / scale;
    quaternion[0] = (m01 + m10) / scale;
    quaternion[1] = 0.25 * scale;
    quaternion[2] = (m12 + m21) / scale;
  } else {
    const double scale = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    quaternion[3] = (m10 - m01) / scale;
    quaternion[0] = (m02 + m20) / scale;
    quaternion[1] = (m12 + m21) / scale;
    quaternion[2] = 0.25 * scale;
  }

  const double norm = quaternionNorm(quaternion);
  if (norm > 0.0) {
    quaternion /= norm;
  }
  return quaternion;
}

}  // namespace

class TargetPoseEstimator : public rclcpp::Node
{
public:
  TargetPoseEstimator()
  : Node("target_pose_estimator")
  {
    target_size_m_ = declare_parameter<double>("target_size_m", 0.16);
    use_depth_ = declare_parameter<bool>("use_depth", false);
    depth_scale_ = declare_parameter<double>("depth_scale", 0.001);
    depth_window_radius_ = declare_parameter<int>("depth_window_radius", 3);
    preferred_symbology_ = declare_parameter<std::string>("preferred_symbology", "");
    publish_frame_id_ = declare_parameter<std::string>("publish_frame_id", "");

    if (target_size_m_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "target_size_m must be positive; using 0.16 m");
      target_size_m_ = 0.16;
    }
    if (depth_scale_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "depth_scale must be positive; using 0.001");
      depth_scale_ = 0.001;
    }
    if (depth_window_radius_ < 0) {
      RCLCPP_WARN(get_logger(), "depth_window_radius must be non-negative; using 0");
      depth_window_radius_ = 0;
    }

    pose_pub_ = create_publisher<PoseStampedMsg>("target_pose", 10);

    camera_info_sub_ = create_subscription<CameraInfoMsg>(
      "camera_info", rclcpp::SensorDataQoS(),
      [this](CameraInfoMsg::ConstSharedPtr msg) {
        camera_info_ = msg;
      });

    if (use_depth_) {
      depth_sub_ = create_subscription<ImageMsg>(
        "depth", rclcpp::SensorDataQoS(),
        [this](ImageMsg::ConstSharedPtr msg) {
          depth_image_ = msg;
        });
    }

    detections_sub_ = create_subscription<BarcodeDetectionsMsg>(
      "detections", rclcpp::QoS(10).reliable(),
      [this](BarcodeDetectionsMsg::ConstSharedPtr msg) {
        detectionsCb(msg);
      });
  }

private:
  void detectionsCb(const BarcodeDetectionsMsg::ConstSharedPtr & detections_msg)
  {
    if (!camera_info_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Waiting for camera_info before estimating target pose");
      return;
    }

    const auto detection = selectDetection(*detections_msg);
    if (detection == nullptr) {
      return;
    }

    const auto image_points = toImagePoints(*detection);
    if (image_points.size() != 4U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Skipping %s detection with %zu polygon points; PnP requires 4 corners",
        detection->symbology.c_str(), image_points.size());
      return;
    }

    cv::Mat rvec;
    cv::Mat tvec;
    const bool solved = cv::solvePnP(
      buildObjectPoints(target_size_m_),
      image_points,
      cameraMatrixFromInfo(*camera_info_),
      distortionFromInfo(*camera_info_),
      rvec,
      tvec,
      false,
      cv::SOLVEPNP_IPPE_SQUARE);

    if (!solved) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "solvePnP failed");
      return;
    }

    if (use_depth_) {
      applyDepthScale(image_points, tvec);
    }

    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    const auto quaternion = rotationMatrixToQuaternion(rotation_matrix);

    PoseStampedMsg pose_msg;
    pose_msg.header = detections_msg->header;
    if (!publish_frame_id_.empty()) {
      pose_msg.header.frame_id = publish_frame_id_;
    }
    pose_msg.pose.position.x = tvec.at<double>(0);
    pose_msg.pose.position.y = tvec.at<double>(1);
    pose_msg.pose.position.z = tvec.at<double>(2);
    pose_msg.pose.orientation.x = quaternion[0];
    pose_msg.pose.orientation.y = quaternion[1];
    pose_msg.pose.orientation.z = quaternion[2];
    pose_msg.pose.orientation.w = quaternion[3];
    pose_pub_->publish(pose_msg);
  }

  const zbar_ros::msg::BarcodeDetection * selectDetection(
    const BarcodeDetectionsMsg & detections_msg) const
  {
    for (const auto & detection : detections_msg.detections) {
      if (!preferred_symbology_.empty() && detection.symbology != preferred_symbology_) {
        continue;
      }
      if (detection.polygon.points.size() == 4U) {
        return &detection;
      }
    }
    return nullptr;
  }

  void applyDepthScale(const std::vector<cv::Point2f> & image_points, cv::Mat & tvec)
  {
    const auto depth_m = sampleDepthAtCenter(image_points);
    if (!depth_m.has_value() || *depth_m <= 0.0) {
      return;
    }

    const double pnp_z = tvec.at<double>(2);
    if (std::abs(pnp_z) < 1e-6) {
      return;
    }

    const double scale = *depth_m / pnp_z;
    tvec.at<double>(0) *= scale;
    tvec.at<double>(1) *= scale;
    tvec.at<double>(2) = *depth_m;
  }

  std::optional<double> sampleDepthAtCenter(const std::vector<cv::Point2f> & image_points)
  {
    if (!depth_image_) {
      return std::nullopt;
    }

    cv_bridge::CvImageConstPtr depth_cv;
    try {
      depth_cv = cv_bridge::toCvShare(depth_image_);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Failed to read depth image: %s", ex.what());
      return std::nullopt;
    }

    cv::Point2f center(0.0F, 0.0F);
    for (const auto & point : image_points) {
      center += point;
    }
    center *= 1.0F / static_cast<float>(image_points.size());

    const int center_x = static_cast<int>(std::round(center.x));
    const int center_y = static_cast<int>(std::round(center.y));
    const int min_x = std::max(0, center_x - depth_window_radius_);
    const int max_x = std::min(depth_cv->image.cols - 1, center_x + depth_window_radius_);
    const int min_y = std::max(0, center_y - depth_window_radius_);
    const int max_y = std::min(depth_cv->image.rows - 1, center_y + depth_window_radius_);

    std::vector<double> samples;
    for (int y = min_y; y <= max_y; ++y) {
      for (int x = min_x; x <= max_x; ++x) {
        const auto depth = depthAt(depth_cv->image, x, y);
        if (depth > 0.0 && std::isfinite(depth)) {
          samples.push_back(depth);
        }
      }
    }

    if (samples.empty()) {
      return std::nullopt;
    }

    std::sort(samples.begin(), samples.end());
    return samples.at(samples.size() / 2U);
  }

  double depthAt(const cv::Mat & depth_image, int x, int y)
  {
    if (depth_image.type() == CV_16UC1) {
      return static_cast<double>(depth_image.at<uint16_t>(y, x)) * depth_scale_;
    }
    if (depth_image.type() == CV_32FC1) {
      return static_cast<double>(depth_image.at<float>(y, x));
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000, "Unsupported depth image type: %d", depth_image.type());
    return 0.0;
  }

  rclcpp::Subscription<BarcodeDetectionsMsg>::SharedPtr detections_sub_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr depth_sub_;
  rclcpp::Publisher<PoseStampedMsg>::SharedPtr pose_pub_;
  CameraInfoMsg::ConstSharedPtr camera_info_;
  ImageMsg::ConstSharedPtr depth_image_;
  double target_size_m_{0.16};
  bool use_depth_{false};
  double depth_scale_{0.001};
  int depth_window_radius_{3};
  std::string preferred_symbology_;
  std::string publish_frame_id_;
};

}  // namespace zbar_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<zbar_ros::TargetPoseEstimator>());
  rclcpp::shutdown();
  return 0;
}
