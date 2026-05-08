from pathlib import Path
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_video_device():
    """Prefer stable by-id device names, then fall back to /dev/videoN."""
    by_id_dir = Path("/dev/v4l/by-id")
    candidates = []

    if by_id_dir.exists():
        index_zero_devices = sorted(by_id_dir.glob("*-video-index0"))
        all_by_id_devices = sorted(by_id_dir.iterdir())
        candidates.extend(index_zero_devices)
        candidates.extend(
            device for device in all_by_id_devices if device not in index_zero_devices
        )

    for candidate in candidates:
        try:
            if candidate.resolve().name.startswith("video"):
                return str(candidate)
        except OSError:
            continue

    for index in range(10):
        device = Path(f"/dev/video{index}")
        if device.exists():
            return str(device)

    return "/dev/video0"


def generate_launch_description():
    video_device = LaunchConfiguration("video_device")
    camera_frame_id = LaunchConfiguration("camera_frame_id")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    pixel_format = LaunchConfiguration("pixel_format")
    output_encoding = LaunchConfiguration("output_encoding")
    image_size = LaunchConfiguration("image_size")
    camera_log_level = LaunchConfiguration("camera_log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "video_device",
                default_value=_default_video_device(),
                description=(
                    "V4L2 camera device. Prefer /dev/v4l/by-id/... for stable "
                    "multi-camera deployments."
                ),
            ),
            DeclareLaunchArgument(
                "camera_frame_id",
                default_value="camera_optical_frame",
                description="Frame id inserted into published image headers.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/image_raw",
                description="Output image topic consumed by zbar_ros.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera_info",
                description="Output camera info topic from the camera driver.",
            ),
            DeclareLaunchArgument(
                "pixel_format",
                default_value="YUYV",
                description=(
                    "V4L2 input pixel format. YUYV is the compatibility-first "
                    "default; avoid MJPG unless the driver path is verified."
                ),
            ),
            DeclareLaunchArgument(
                "output_encoding",
                default_value="mono8",
                description=(
                    "ROS image encoding published by v4l2_camera. mono8 is the "
                    "fastest path for zbar_ros; use rgb8/bgr8 when another "
                    "consumer requires color images."
                ),
            ),
            DeclareLaunchArgument(
                "image_size",
                default_value="[640, 480]",
                description="Camera image size as [width, height].",
            ),
            DeclareLaunchArgument(
                "camera_log_level",
                default_value="warn",
                description="ROS log level for the camera node.",
            ),
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="usb_camera",
                output="screen",
                ros_arguments=["--log-level", camera_log_level],
                parameters=[
                    {
                        "video_device": video_device,
                        "camera_frame_id": camera_frame_id,
                        "pixel_format": pixel_format,
                        "output_encoding": output_encoding,
                        "image_size": ParameterValue(image_size, value_type=List[int]),
                    }
                ],
                remappings=[
                    ("image_raw", image_topic),
                    ("camera_info", camera_info_topic),
                ],
            ),
        ]
    )
