from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_namespace = LaunchConfiguration("camera_namespace")
    camera_name = LaunchConfiguration("camera_name")
    serial_no = LaunchConfiguration("serial_no")
    usb_port_id = LaunchConfiguration("usb_port_id")
    color_profile = LaunchConfiguration("rgb_camera.color_profile")
    enable_auto_exposure = LaunchConfiguration("rgb_camera.enable_auto_exposure")
    publish_tf = LaunchConfiguration("publish_tf")
    log_level = LaunchConfiguration("log_level")
    output = LaunchConfiguration("output")

    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "camera_namespace": camera_namespace,
            "camera_name": camera_name,
            "serial_no": serial_no,
            "usb_port_id": usb_port_id,
            "log_level": log_level,
            "output": output,
            "enable_color": "true",
            "rgb_camera.color_profile": color_profile,
            "rgb_camera.color_format": "RGB8",
            "rgb_camera.enable_auto_exposure": enable_auto_exposure,
            "enable_depth": "false",
            "enable_infra": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            "enable_motion": "false",
            "enable_rgbd": "false",
            "enable_sync": "false",
            "pointcloud.enable": "false",
            "align_depth.enable": "false",
            "colorizer.enable": "false",
            "publish_tf": publish_tf,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_namespace",
                default_value="camera",
                description="ROS namespace passed to realsense2_camera.",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="camera",
                description="ROS node name passed to realsense2_camera.",
            ),
            DeclareLaunchArgument(
                "serial_no",
                default_value="''",
                description="Optional RealSense serial number filter.",
            ),
            DeclareLaunchArgument(
                "usb_port_id",
                default_value="''",
                description="Optional RealSense USB port filter such as 4-3.",
            ),
            DeclareLaunchArgument(
                "rgb_camera.color_profile",
                default_value="1280x720x30",
                description="RealSense RGB profile as widthxheightxfps.",
            ),
            DeclareLaunchArgument(
                "rgb_camera.enable_auto_exposure",
                default_value="true",
                description="Enable RealSense color auto exposure.",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="false",
                description="Disable TF by default because zbar_ros only needs 2D images.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="warn",
                description="ROS log level for the RealSense camera node.",
            ),
            DeclareLaunchArgument(
                "output",
                default_value="screen",
                description="Launch output target for the RealSense camera node.",
            ),
            rs_launch,
        ]
    )
