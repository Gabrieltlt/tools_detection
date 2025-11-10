# Tools Detection Package

## Overview
This ROS2 package provides real-time object recognition using YOLOv8 computer vision model for the [FBOT@Work](https://fbotwork.vercel.app/) industrial robot (MICKY) in RoboCup@Work league. The package includes dynamic start/stop capabilities, integrated camera support, and RViz visualization.

## Features
- **Real-time object detection** using YOLOv8 (automatic class name extraction from model)
- **Dynamic control** via ROS2 services (start/stop recognition)
- **Configurable detection threshold** and model selection
- **Integrated camera support** with USB camera auto-launch
- **RViz visualization** with custom configuration
- **Clean shutdown handling** for graceful termination
- **Dual output**: visual feedback (image + bounding boxes) and programmatic access (text classes)

## Architecture

### Node: `yolov8_recognition`
ROS2 node with service-based control and configurable parameters.

### Topics

#### 📥 **Subscribed Topics**
- `/camera1/image_raw` (sensor_msgs/Image) - Input RGB camera images

#### 📤 **Published Topics**
- `/micky_vision/tools_recognition` (sensor_msgs/Image) - Output image with bounding boxes and class labels
- `/micky_vision/tools_classes` (std_msgs/String) - Detected classes and confidence percentages in text format

#### ⚙️ **Services**
- `/micky_vision/recognition_start` (std_srvs/Empty) - Start object recognition
- `/micky_vision/recognition_stop` (std_srvs/Empty) - Stop object recognition

## Dependencies

### ROS2 Dependencies
- `rclpy` - ROS2 Python client library
- `sensor_msgs` - Standard sensor message types
- `std_msgs` - Standard message types
- `std_srvs` - Standard service types
- `cv_bridge` - OpenCV-ROS bridge
- `usb_cam` - USB camera driver (for camera integration)
- `rviz2` - 3D visualization tool

### Python Dependencies
- `ultralytics` - YOLOv8 implementation
- `opencv-python` - Computer vision library
- `numpy` - Numerical computing
- `pillow` - Python Imaging Library

## Installation

### 1. Clone Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/Gabrieltlt/tools_recognition.git
```

### 2. Install System Dependencies
```bash
# Install ROS2 packages
sudo apt install ros-humble-usb-cam ros-humble-rviz2

# Install Python dependencies
cd tools_recognition
pip install -r requirements.txt
```

### 3. Build Package
```bash
cd ~/ros2_ws
colcon build --packages-select tools_recognition
source install/setup.bash
```

## Usage

### Quick Start (All-in-One)
```bash
# Launch camera + recognition + visualization
ros2 launch tools_recognition tools_recognition.launch.py
```

### Customized Launch Options

#### Without Camera (if already running separately)
```bash
ros2 launch tools_recognition tools_recognition.launch.py use_camera:=false
```

#### Without RViz (headless mode)
```bash
ros2 launch tools_recognition tools_recognition.launch.py use_rviz:=false
```

#### Manual Control (don't start recognition automatically)
```bash
ros2 launch tools_recognition tools_recognition.launch.py start_on_init:=false
```

#### Custom Parameters
```bash
ros2 launch tools_recognition tools_recognition.launch.py \
  threshold:=0.7 \
  model_file:=yolov8s.pt \
  image_topic:=/my_camera/image_raw
```

### Service Control

#### Start Recognition
```bash
ros2 service call /micky_vision/recognition_start std_srvs/srv/Empty
```

#### Stop Recognition
```bash
ros2 service call /micky_vision/recognition_stop std_srvs/srv/Empty
```

### Monitor Output

#### View Detected Classes (Text)
```bash
ros2 topic echo /micky_vision/tools_classes
```

#### View Detection Image (Visual)
```bash
ros2 run rqt_image_view rqt_image_view /micky_vision/tools_recognition
```

#### Monitor Topic Rates
```bash
ros2 topic hz /micky_vision/tools_recognition
ros2 topic hz /micky_vision/tools_classes
```

## Configuration

### Parameters File: `config/yolov8_object_recognition.yaml`

```yaml
yolov8_recognition:
  ros__parameters:
    # Algorithm settings
    threshold: 0.6              # Detection confidence threshold (0.0-1.0)
    model_file: yolov8n.pt      # YOLOv8 model file in weights/ directory
    start_on_init: true         # Auto-start recognition on node initialization
    
    # Topics configuration
    subscribers:
      image_rgb:
        topic: /camera1/image_raw
        qos_profile: 10
    publishers:
      image:
        topic: /micky_vision/tools_recognition
        qos_profile: 10
      debug:
        topic: /micky_vision/tools_classes
        qos_profile: 10
    
    # Services configuration
    services:
      recognition:
        start: /micky_vision/recognition_start
        stop: /micky_vision/recognition_stop
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_camera` | `true` | Launch USB camera automatically |
| `use_rviz` | `true` | Launch RViz visualization |
| `start_on_init` | `true` | Start recognition on initialization |
| `threshold` | `0.6` | Detection confidence threshold |
| `model_file` | `yolov8n.pt` | YOLOv8 model filename |
| `image_topic` | `/camera1/image_raw` | Input camera topic |
| `detection_topic` | `/micky_vision/tools_recognition` | Output image topic |
| `debug_topic` | `/micky_vision/tools_classes` | Output classes topic |

## RViz Configuration

The package includes a pre-configured RViz setup (`rviz/tools_recognition.rviz`) that displays:
- Camera input stream
- Detection results with bounding boxes

## Output Examples

### Text Output (`/micky_vision/tools_classes`)
```
person: 85.3% | tv: 72.1% | laptop: 68.9%
```

### Visual Output (`/micky_vision/tools_recognition`)
- Original camera image with green bounding boxes
- Class names and confidence percentages overlaid on detections

## Troubleshooting

### Common Issues

1. **Model not found**
   - Ensure model file exists in `weights/` directory
   - Check `model_file` parameter in configuration

2. **No camera input**
   - Verify camera is connected and accessible
   - Check camera topic with `ros2 topic list`
   - Test camera with `ros2 topic echo /camera1/image_raw`

3. **Import/dependency errors**
   - Install Python dependencies: `pip install -r requirements.txt`
   - Verify ROS2 packages: `ros2 pkg list | grep -E "(usb_cam|rviz)"` 

4. **Performance issues**
   - Try smaller YOLOv8 model (yolov8n.pt)
   - Increase detection threshold
   - Reduce camera resolution

### Debug Commands

```bash
# Check available topics
ros2 topic list | grep micky_vision

# Monitor topic types
ros2 topic list -t

# Check service availability
ros2 service list | grep recognition

# View node information
ros2 node info /yolov8_recognition

# Check parameter values
ros2 param list /yolov8_recognition
```

## Development

### Clean Shutdown
The package implements graceful shutdown handling:
- Ctrl+C displays clean termination message
- Model is properly unloaded from memory
- Services are cleanly destroyed

### Extending Functionality
- Custom models: Place `.pt` files in `weights/` directory
- Custom topics: Modify YAML configuration
- Additional outputs: Extend publisher configuration

## Maintainer
Gabriel Torres (gabrieltlt721@gmail.com)