from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    dataset_path = LaunchConfiguration("dataset_path")
    image_topic = LaunchConfiguration("image_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    frame_id = LaunchConfiguration("frame_id")
    publish_interval_seconds = LaunchConfiguration("publish_interval_seconds")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    publish_empty_detections = LaunchConfiguration("publish_empty_detections")
    qrcode_only = LaunchConfiguration("qrcode_only")
    scanner_x_density = LaunchConfiguration("scanner_x_density")
    scanner_y_density = LaunchConfiguration("scanner_y_density")
    try_inverted = LaunchConfiguration("try_inverted")
    equalize_histogram = LaunchConfiguration("equalize_histogram")
    scan_scale = LaunchConfiguration("scan_scale")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("zbar_ros"), "data", "dataset"]
                ),
                description="Directory containing dataset images for offline testing.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/perception/test/image_raw",
                description="Image topic shared between publisher and barcode reader.",
            ),
            DeclareLaunchArgument(
                "detections_topic",
                default_value="/perception/barcodes",
                description="Structured detection output topic.",
            ),
            DeclareLaunchArgument(
                "debug_image_topic",
                default_value="/perception/debug/barcodes",
                description="Annotated debug image topic.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="dataset_camera_optical_frame",
                description="Frame id stamped into the published dataset images.",
            ),
            DeclareLaunchArgument(
                "publish_interval_seconds",
                default_value="1.0",
                description="Delay between dataset image publishes in seconds.",
            ),
            DeclareLaunchArgument(
                "publish_debug_image",
                default_value="true",
                description="Whether to publish annotated debug images.",
            ),
            DeclareLaunchArgument(
                "publish_empty_detections",
                default_value="true",
                description="Publish empty detection messages for frames with no codes.",
            ),
            DeclareLaunchArgument(
                "qrcode_only",
                default_value="true",
                description="Restrict ZBar scanning to QR codes only.",
            ),
            DeclareLaunchArgument(
                "scanner_x_density",
                default_value="1",
                description="ZBar vertical scan-line density.",
            ),
            DeclareLaunchArgument(
                "scanner_y_density",
                default_value="1",
                description="ZBar horizontal scan-line density.",
            ),
            DeclareLaunchArgument(
                "try_inverted",
                default_value="false",
                description="Also test inverted light-on-dark codes when decoding fails.",
            ),
            DeclareLaunchArgument(
                "equalize_histogram",
                default_value="false",
                description="Apply mono8 histogram equalization before decoding.",
            ),
            DeclareLaunchArgument(
                "scan_scale",
                default_value="1.0",
                description="Scale applied before scanning.",
            ),
            Node(
                package="zbar_ros",
                executable="dataset_image_publisher",
                name="dataset_image_publisher",
                output="screen",
                parameters=[
                    {
                        "dataset_path": dataset_path,
                        "frame_id": frame_id,
                        "publish_interval_seconds": ParameterValue(
                            publish_interval_seconds, value_type=float
                        ),
                    }
                ],
                remappings=[("image", image_topic)],
            ),
            Node(
                package="zbar_ros",
                executable="qr_code_detector",
                name="qr_code_detector",
                output="screen",
                parameters=[
                    {
                        "publish_debug_image": ParameterValue(
                            publish_debug_image, value_type=bool
                        ),
                        "publish_empty_detections": ParameterValue(
                            publish_empty_detections, value_type=bool
                        ),
                        "qrcode_only": ParameterValue(qrcode_only, value_type=bool),
                        "scanner_x_density": ParameterValue(
                            scanner_x_density, value_type=int
                        ),
                        "scanner_y_density": ParameterValue(
                            scanner_y_density, value_type=int
                        ),
                        "try_inverted": ParameterValue(try_inverted, value_type=bool),
                        "equalize_histogram": ParameterValue(
                            equalize_histogram, value_type=bool
                        ),
                        "scan_scale": ParameterValue(scan_scale, value_type=float),
                    }
                ],
                remappings=[
                    ("image", image_topic),
                    ("detections", detections_topic),
                    ("debug_image", debug_image_topic),
                ],
            ),
        ]
    )
