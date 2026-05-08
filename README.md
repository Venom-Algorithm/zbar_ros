# zbar_ros

`zbar_ros` is the Venom perception-layer barcode detector built on top of ZBar.
The repository is intentionally kept as a single ROS 2 package:

- input stays on `sensor_msgs/msg/Image`
- output is published as `zbar_ros/msg/BarcodeDetections`
- no TF is published
- output headers inherit the source image `stamp` and `frame_id`

This version is trimmed for two concrete workflows only:

1. offline verification on `data/dataset`
2. USB camera recognition on the NUC

AprilTag detection is also available now through OpenCV `aruco` when the host
already provides `opencv-contrib`.

For offline dataset playback, image transport is now forced to reliable QoS so
that QR, 1D barcode, and AprilTag frames are not dropped between the dataset
publisher and detector.

## Package Layout

The maintained structure is:

```text
zbar_ros/
├── CMakeLists.txt
├── package.xml
├── README.md
├── msg/
├── include/zbar_ros/
├── src/
├── launch/
└── data/dataset/
```

There is no separate `zbar_interfaces` package anymore. Messages are defined in
`zbar_ros/msg`.

## Runtime Contract

Default runtime interfaces:

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| subscribe | `/image_raw` | `sensor_msgs/msg/Image` | source image stream |
| publish | `/perception/barcodes` | `zbar_ros/msg/BarcodeDetections` | per-frame detections |
| publish | `/perception/debug/barcodes` | `sensor_msgs/msg/Image` | annotated debug image |

Each `BarcodeDetection` contains:

- `data`: decoded text
- `symbology`: ZBar type such as `QRCODE`, `EAN-13`, `CODE-128`
- `polygon`: 2D polygon in image pixel coordinates

`zbar_ros` does not publish TF and does not estimate 3D pose.

## Launch Files

Three launch files are kept:

| Launch File | Purpose |
| --- | --- |
| `dataset_barcode.launch.py` | offline dataset verification |
| `usb_camera.launch.py` | USB camera image source |
| `d435i_camera.launch.py` | Intel RealSense D435i color image source |
| `zbar_ros.launch.py` | detector node |

## Build

From `~/venom_ws`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select zbar_ros
source install/setup.bash
```

## Dataset Verification

Run the built-in sample dataset:

```bash
cd ~/venom_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zbar_ros dataset_barcode.launch.py \
  qrcode_only:=false \
  enable_apriltag:=true \
  apriltag_family:=tag36h11 \
  publish_empty_detections:=false
```

The default dataset path resolves to the installed copy of:

```text
perception/zbar_ros/data/dataset
```

The maintained dataset currently contains one sample for each type:

- QR code
- 1D barcode
- AprilTag

To use another directory:

```bash
ros2 launch zbar_ros dataset_barcode.launch.py \
  dataset_path:=/path/to/your/dataset \
  qrcode_only:=false \
  enable_apriltag:=true \
  apriltag_family:=tag36h11 \
  publish_empty_detections:=false \
  publish_interval_seconds:=0.5
```

In another shell:

```bash
cd ~/venom_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /perception/barcodes
```

For easier observation, slow the dataset publisher down:

```bash
ros2 launch zbar_ros dataset_barcode.launch.py \
  qrcode_only:=false \
  enable_apriltag:=true \
  apriltag_family:=tag36h11 \
  publish_empty_detections:=false \
  publish_interval_seconds:=2.0
```

Terminal 1 will now show which dataset image is being published, and the detector
will print the decoded result in the same window.

In dataset mode, `use_reliable_image_qos:=true` is enabled by default. Keep it
enabled unless you explicitly need to test sensor-data QoS behavior.

## USB Camera Recognition

`usb_camera.launch.py` is tuned for compatibility and stability first:

- prefer `/dev/v4l/by-id/*-video-index0`
- use `YUYV` by default
- publish `mono8` by default for the fastest path into ZBar

Start the camera:

```bash
cd ~/venom_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zbar_ros usb_camera.launch.py
```

Start the detector:

```bash
cd ~/venom_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zbar_ros zbar_ros.launch.py
```

`zbar_ros.launch.py` keeps `use_reliable_image_qos:=false` by default for live
camera topics, because most camera drivers publish with sensor-data QoS.

Useful detector settings:

- QR only, lower latency:

```bash
ros2 launch zbar_ros zbar_ros.launch.py \
  image_topic:=/image_raw \
  qrcode_only:=true \
  publish_empty_detections:=false \
  publish_debug_image:=false
```

- QR + 1D barcode, balanced for stability:
  
```bash
ros2 launch zbar_ros zbar_ros.launch.py \
  image_topic:=/image_raw \
  qrcode_only:=false \
  scanner_x_density:=1 \
  scanner_y_density:=1 \
  publish_empty_detections:=false \
  publish_debug_image:=false
```

- QR + 1D barcode, higher sensitivity on real hardware:

```bash
ros2 launch zbar_ros zbar_ros.launch.py \
  image_topic:=/image_raw \
  qrcode_only:=false \
  scanner_x_density:=1 \
  scanner_y_density:=1 \
  try_inverted:=true \
  equalize_histogram:=true \
  scan_scale:=1.0 \
  publish_empty_detections:=false \
  publish_debug_image:=false
```

If the camera is fixed on the NUC, passing a stable by-id device is recommended:

```bash
ros2 launch zbar_ros usb_camera.launch.py \
  video_device:=/dev/v4l/by-id/usb-your-camera-video-index0
```

## Practical NUC Settings

- QR only:
  - `image_size:="[640,480]"`
  - `qrcode_only:=true`
  - `output_encoding:=mono8`
- QR + barcode:
  - prefer the highest stable frame-rate mode first
  - try `pixel_format:=MJPG` with `image_size:="[1280,720]"` if the camera keeps 30 fps
  - fall back to `YUYV` if the MJPG-to-ROS image path is unstable
  - `qrcode_only:=false`
  - consider `try_inverted:=true` and `equalize_histogram:=true`

If barcode recognition is weak, increase image resolution first before changing
scanner density.

## Intel RealSense D435i Recognition

For D435i on the NUC, do not use `v4l2_camera`.
Use `realsense2_camera` and feed `zbar_ros` from the RealSense color topic:

- camera topic: `/camera/camera/color/image_raw`
- camera info: `/camera/camera/color/camera_info`

Start the D435i color stream:

```bash
cd ~/venom_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zbar_ros d435i_camera.launch.py
```

Recommended QR-only detector command:

```bash
cd ~/venom_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zbar_ros zbar_ros.launch.py \
  image_topic:=/camera/camera/color/image_raw \
  qrcode_only:=true \
  try_inverted:=true \
  equalize_histogram:=true \
  publish_empty_detections:=false \
  publish_debug_image:=false
```

Recommended QR + 1D barcode detector command:

```bash
cd ~/venom_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zbar_ros zbar_ros.launch.py \
  image_topic:=/camera/camera/color/image_raw \
  qrcode_only:=false \
  try_inverted:=true \
  equalize_histogram:=true \
  scan_scale:=1.0 \
  publish_empty_detections:=false \
  publish_debug_image:=false
```

Recommended AprilTag detector command on D435i:

```bash
cd ~/venom_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zbar_ros zbar_ros.launch.py \
  image_topic:=/camera/camera/color/image_raw \
  qrcode_only:=false \
  enable_apriltag:=true \
  apriltag_family:=tag36h11 \
  publish_empty_detections:=false \
  publish_debug_image:=false
```

The default D435i color profile is `1280x720x30`.
If latency matters more than range, keep that profile.
If the scene is bright and QR codes are close, `640x480x30` can reduce CPU load:

```bash
ros2 launch zbar_ros d435i_camera.launch.py \
  rgb_camera.color_profile:=640x480x30
```

If multiple RealSense devices exist, pin the USB port explicitly:

```bash
ros2 launch zbar_ros d435i_camera.launch.py usb_port_id:=4-3
```

## Runtime Notes

- The detector publishes one `BarcodeDetections` message per input frame, even when no code is present.
- Debug images preserve the incoming header and are safe to inspect in RViz or `rqt_image_view`.
- `qrcode_only=true` is the recommended default for the current Venom use case.
- The module does not subscribe to `/camera_info`, because the current scope is 2D decoding only.
- AprilTag support depends on OpenCV `aruco` from `opencv-contrib`, not on `apriltag_ros`.
