# ROS2 Real-Time Object Detection

A real-time object detection system using ROS2 and YOLOv4-tiny, with web streaming capabilities.

## Features

- Real-time object detection using YOLOv4-tiny
- Detection of humans and vehicles (cars, buses, trucks)
- Web-based video streaming
- FPS optimization
- CPU-efficient implementation

## Requirements

- ROS2 (Humble)
- Python 3.8+
- OpenCV
- Flask
- YOLOv4-tiny weights

## Installation

1. Create ROS2 workspace:
- mkdir -p ~/ros2_ws/src
- cd ~/ros2_ws/src

2. Clone the package:
- git clone https://github.com/YOUR_USERNAME/REPO_NAME.git

3. Install dependencies:
- pip3 install flask flask-cors opencv-python-headless

4. Download YOLOv4-tiny model:
- cd object_detection_pkg/models
- wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights
- wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg

5. Build the package:
- cd ~/ros2_ws
- colcon build --packages-select object_detection_pkg

## Usage

1. Start the object detection node:
- source ~/ros2_ws/install/setup.bash
- ros2 run object_detection_pkg detection_node

2. Launch the web server:
- source ~/ros2_ws/install/setup.bash
- ros2 run object_detection_pkg web_stream

3. View in web browser:
- http://localhost:5000

## Configuration
- You can modify these parameters in the code for better performance:
-- Frame processing rate
-- Input image size
-- Detection confidence threshold
-- Web stream quality

## Troubleshooting

### If the camera doesn't work:
- Check camera ID (default is 0 for built-in camera)
- Verify camera permissions

### Low FPS:
- Reduce input image size
- Increase frame skip rate
- Adjust detection confidence threshold
