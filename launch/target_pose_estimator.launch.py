from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    detections_topic = LaunchConfiguration("detections_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    target_pose_topic = LaunchConfiguration("target_pose_topic")
    target_size_m = LaunchConfiguration("target_size_m")
    use_depth = LaunchConfiguration("use_depth")
    depth_scale = LaunchConfiguration("depth_scale")
    depth_window_radius = LaunchConfiguration("depth_window_radius")
    preferred_symbology = LaunchConfiguration("preferred_symbology")
    publish_frame_id = LaunchConfiguration("publish_frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detections_topic",
                default_value="/perception/barcodes",
                description="Barcode and AprilTag detections from zbar_ros.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
                description="Camera calibration matching the image stream.",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
                description="Optional depth image aligned to the color camera.",
            ),
            DeclareLaunchArgument(
                "target_pose_topic",
                default_value="/perception/target_pose",
                description="Estimated target pose output.",
            ),
            DeclareLaunchArgument(
                "target_size_m",
                default_value="0.16",
                description="Physical side length of the square target in meters.",
            ),
            DeclareLaunchArgument(
                "use_depth",
                default_value="false",
                description="Use aligned depth to correct PnP translation distance.",
            ),
            DeclareLaunchArgument(
                "depth_scale",
                default_value="0.001",
                description="Scale for uint16 depth images, usually millimeters to meters.",
            ),
            DeclareLaunchArgument(
                "depth_window_radius",
                default_value="3",
                description="Median depth sample radius around the target center in pixels.",
            ),
            DeclareLaunchArgument(
                "preferred_symbology",
                default_value="QR-Code",
                description="Only estimate pose for this symbology; empty accepts any 4-corner detection.",
            ),
            DeclareLaunchArgument(
                "publish_frame_id",
                default_value="",
                description="Override output frame_id; empty preserves the detection frame_id.",
            ),
            Node(
                package="zbar_ros",
                executable="target_pose_estimator",
                name="target_pose_estimator",
                output="screen",
                parameters=[
                    {
                        "target_size_m": ParameterValue(target_size_m, value_type=float),
                        "use_depth": ParameterValue(use_depth, value_type=bool),
                        "depth_scale": ParameterValue(depth_scale, value_type=float),
                        "depth_window_radius": ParameterValue(
                            depth_window_radius, value_type=int
                        ),
                        "preferred_symbology": preferred_symbology,
                        "publish_frame_id": publish_frame_id,
                    }
                ],
                remappings=[
                    ("detections", detections_topic),
                    ("camera_info", camera_info_topic),
                    ("depth", depth_topic),
                    ("target_pose", target_pose_topic),
                ],
            ),
        ]
    )
