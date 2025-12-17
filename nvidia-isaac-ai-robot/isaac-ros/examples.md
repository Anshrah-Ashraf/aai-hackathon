---
sidebar_position: 6
---

# Isaac ROS Examples

This section provides practical examples of using Isaac ROS for various robotics applications. These examples demonstrate how to implement hardware-accelerated perception and navigation using Isaac ROS packages.

## Example 1: Visual SLAM with Isaac ROS

This example demonstrates setting up visual SLAM using Isaac ROS packages.

### Setup Requirements

- Stereo camera or RGB-D sensor
- NVIDIA GPU with CUDA support
- ROS 2 environment with Isaac ROS packages

### Implementation Steps

1. **Camera Configuration**
   ```yaml
   # camera_params.yaml
   camera:
     ros__parameters:
       camera_info_url: "file:///path/to/camera_info.yaml"
       image_topic: "/camera/rgb/image_rect_color"
       camera_frame: "camera_link"
   ```

2. **Isaac ROS Visual SLAM Node**
   ```yaml
   # visual_slam_params.yaml
   visual_slam_node:
     ros__parameters:
       input_viz_0: "input_image"
       input_viz_1: "input_depth"
       map_frame: "map"
       tracking_frame: "base_link"
       publish_pose_graph: true
       enable_localization: true
   ```

3. **Launch Configuration**
   ```python
   # launch/visual_slam.launch.py
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='isaac_ros_visual_slam',
               executable='visual_slam_node',
               parameters=['visual_slam_params.yaml'],
               remappings=[
                   ('/visual_slam_node/input_image', '/camera/rgb/image_rect_color'),
                   ('/visual_slam_node/input_depth', '/camera/depth/image_rect'),
                   ('/visual_slam_node/odometry', '/visual_slam/odometry')
               ]
           )
       ])
   ```

### Expected Results

- Real-time pose estimation with GPU acceleration
- Map building and loop closure detection
- Significantly improved performance compared to CPU-only implementations

## Example 2: Multi-Sensor Fusion

This example shows how to fuse data from multiple sensors using Isaac ROS.

### Components Used

- RGB camera
- IMU sensor
- Optional LiDAR
- Isaac ROS sensor fusion packages

### Implementation

1. **Sensor Configuration**
   ```yaml
   # sensor_fusion_params.yaml
   sensor_fusion_node:
     ros__parameters:
       camera_topic: "/camera/rgb/image_rect_color"
       imu_topic: "/imu/data"
       lidar_topic: "/lidar/points"
       fusion_rate: 30.0
       enable_visual_inertial: true
       enable_camera_lidar: false
   ```

2. **Pipeline Setup**
   ```python
   # sensor_fusion_pipeline.py
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image, Imu, PointCloud2

   class SensorFusionNode(Node):
       def __init__(self):
           super().__init__('sensor_fusion_node')
           self.camera_sub = self.create_subscription(
               Image, 'camera/image', self.camera_callback, 10)
           self.imu_sub = self.create_subscription(
               Imu, 'imu/data', self.imu_callback, 10)
           self.fused_pub = self.create_publisher(
               # fused sensor message type
               'fused_sensor_data', 10)

       def camera_callback(self, msg):
           # Process camera data with GPU acceleration
           pass

       def imu_callback(self, msg):
           # Process IMU data
           pass
   ```

### Performance Benefits

- Real-time processing of multiple sensor streams
- Hardware-accelerated fusion algorithms
- Reduced latency in sensor data processing

## Example 3: Object Detection and Tracking

This example demonstrates GPU-accelerated object detection and tracking.

### Required Components

- Camera with sufficient resolution
- Isaac ROS detection packages
- Pre-trained detection model

### Configuration

1. **Detection Parameters**
   ```yaml
   # detection_params.yaml
   detection_node:
     ros__parameters:
       input_image_topic: "/camera/rgb/image_rect_color"
       output_detections_topic: "/detections"
       model_path: "/path/to/model.trt"
       confidence_threshold: 0.5
       max_batch_size: 1
   ```

2. **Tracking Configuration**
   ```yaml
   # tracking_params.yaml
   tracking_node:
     ros__parameters:
       input_detections_topic: "/detections"
       input_image_topic: "/camera/rgb/image_rect_color"
       output_tracks_topic: "/object_tracks"
       max_objects: 50
       tracking_window: 10
   ```

### Implementation Considerations

- Model optimization using TensorRT for best performance
- Proper camera calibration for accurate detection
- Synchronization of detection and tracking pipelines

## Example 4: Perception Pipeline for Navigation

This example shows how to set up a complete perception pipeline for robot navigation.

### Complete Pipeline

```
Camera Input → Isaac ROS Image Pipeline → Feature Detection →
Object Detection → Sensor Fusion → Navigation Planning
```

### Launch File

```python
# launch/perception_pipeline.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Image preprocessing pipeline
    image_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('isaac_ros_image_pipeline'),
                        'launch', 'image_pipeline.launch.py')
        ])
    )

    # Visual SLAM component
    visual_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('isaac_ros_visual_slam'),
                        'launch', 'visual_slam.launch.py')
        ])
    )

    # Object detection component
    object_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('isaac_ros_detectnet'),
                        'launch', 'detectnet.launch.py')
        ])
    )

    return LaunchDescription([
        image_pipeline,
        visual_slam,
        object_detection,
    ])
```

## Performance Benchmarks

### Typical Performance Improvements

- **Visual SLAM**: 5-10x speed improvement over CPU implementations
- **Object Detection**: 3-15x speed improvement depending on model
- **Image Processing**: 10-50x speed improvement for parallel operations
- **Multi-Sensor Fusion**: 5-20x speed improvement for complex algorithms

### Resource Utilization

- GPU utilization typically 60-85% for active processing
- Memory usage optimized through efficient GPU memory management
- Power consumption balanced between performance and efficiency

## Troubleshooting Common Issues

### Performance Issues

- **Low GPU Utilization**: Check for CPU bottlenecks or memory transfer issues
- **High Latency**: Verify proper pipeline synchronization
- **Inconsistent Performance**: Monitor thermal throttling and power limits

### Integration Issues

- **Topic Mismatches**: Verify message type compatibility
- **Timing Issues**: Check timestamp synchronization
- **Coordinate Frame Problems**: Validate tf2 transformations

## Best Practices from Examples

### Pipeline Design

- Use appropriate buffer sizes to avoid data loss
- Implement proper error handling and recovery
- Monitor resource utilization and performance metrics
- Validate results for accuracy and reliability

### Configuration Management

- Use parameter files for easy configuration changes
- Implement runtime parameter adjustment where possible
- Document parameter meanings and valid ranges
- Provide default configurations for common use cases

[Continue to Isaac ROS Assessment](./assessment.md)