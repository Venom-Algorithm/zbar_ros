from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    publish_empty_detections = LaunchConfiguration("publish_empty_detections")
    qrcode_only = LaunchConfiguration("qrcode_only")
    scanner_x_density = LaunchConfiguration("scanner_x_density")
    scanner_y_density = LaunchConfiguration("scanner_y_density")
    try_inverted = LaunchConfiguration("try_inverted")
    equalize_histogram = LaunchConfiguration("equalize_histogram")
    scan_scale = LaunchConfiguration("scan_scale")
    enable_apriltag = LaunchConfiguration("enable_apriltag")
    apriltag_family = LaunchConfiguration("apriltag_family")
    use_reliable_image_qos = LaunchConfiguration("use_reliable_image_qos")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/image_raw",
                description="Input image topic consumed by the detector.",
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
                description=(
                    "ZBar vertical scan-line density. Keep at 1 for maximum "
                    "one-dimensional barcode sensitivity."
                ),
            ),
            DeclareLaunchArgument(
                "scanner_y_density",
                default_value="1",
                description=(
                    "ZBar horizontal scan-line density. Keep at 1 for maximum "
                    "one-dimensional barcode sensitivity."
                ),
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
                description=(
                    "Scale applied before scanning. 1.0 keeps full quality; "
                    "lower values trade sensitivity for speed."
                ),
            ),
            DeclareLaunchArgument(
                "enable_apriltag",
                default_value="false",
                description="Enable AprilTag detection through OpenCV aruco.",
            ),
            DeclareLaunchArgument(
                "apriltag_family",
                default_value="tag36h11",
                description="AprilTag family: tag16h5, tag25h9, tag36h10, tag36h11.",
            ),
            DeclareLaunchArgument(
                "use_reliable_image_qos",
                default_value="false",
                description="Use reliable QoS for the image subscription.",
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
                        "enable_apriltag": ParameterValue(
                            enable_apriltag, value_type=bool
                        ),
                        "apriltag_family": apriltag_family,
                        "use_reliable_image_qos": ParameterValue(
                            use_reliable_image_qos, value_type=bool
                        ),
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
