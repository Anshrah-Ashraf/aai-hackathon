---
sidebar_position: 6
---

# Nav2 Navigation Examples for Humanoid Robots

This section provides practical examples of implementing Nav2 navigation for humanoid robots. These examples demonstrate how to configure and use Nav2 with the unique requirements of bipedal locomotion.

## Example 1: Basic Humanoid Navigation Setup

This example shows how to set up basic navigation for a humanoid robot using Nav2.

### Required Components

- Humanoid robot with bipedal locomotion
- Navigation-ready sensors (IMU, cameras, LiDAR)
- ROS 2 environment with Nav2 packages
- Isaac ROS perception packages (optional)

### Configuration Files

1. **Costmap Configuration**
   ```yaml
   # humanoid_local_costmap_params.yaml
   local_costmap:
     ros__parameters:
       update_frequency: 10.0
       publish_frequency: 10.0
       global_frame: odom
       robot_base_frame: base_link
       use_sim_time: true
       rolling_window: true
       width: 6
       height: 6
       resolution: 0.05
       footprint: [[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]
       plugins: ["obstacle_layer", "inflation_layer"]
       inflation_layer:
         inflation_radius: 0.5
         cost_scaling_factor: 3.0
       obstacle_layer:
         enabled: true
         observation_sources: scan
         scan:
           topic: /laser_scan
           max_obstacle_height: 2.0
           clearing: true
           marking: true
   ```

2. **Humanoid-Specific Parameters**
   ```yaml
   # humanoid_nav2_params.yaml
   bt_navigator:
     ros__parameters:
       use_sim_time: true
       global_frame: map
       robot_base_frame: base_link
       odom_topic: /odom
       bt_loop_duration: 10
       default_server_timeout: 20
       enable_groot_monitoring: True
       groot_zmq_publisher_port: 1666
       groot_zmq_server_port: 1667
       navigate_to_pose_goal_checker:
         plugin: "nav2_controller::SimpleGoalChecker"
         xy_goal_tolerance: 0.25
         yaw_goal_tolerance: 0.25
         stateful: True

   controller_server:
     ros__parameters:
       use_sim_time: true
       controller_frequency: 20.0
       min_x_velocity_threshold: 0.001
       min_y_velocity_threshold: 0.5
       min_theta_velocity_threshold: 0.001
       progress_checker_plugin: "progress_checker"
       goal_checker_plugin: "goal_checker"
       controller_plugins: ["FollowPath"]

       FollowPath:
         plugin: "nav2_mppi_controller::MPPIController"
         time_steps: 50
         model_dt: 0.05
         batch_size: 1000
         vx_std: 0.2
         vy_std: 0.2
         wz_std: 0.3
         vx_max: 0.3
         vx_min: -0.15
         vy_max: 0.1
         wz_max: 0.5
         xy_goal_tolerance: 0.25
         trans_stopped_velocity: 0.25
         shorted_path_angle_threshold: 1.5
         # Humanoid-specific parameters
         balance_constraint_weight: 10.0
         step_constraint_weight: 5.0
   ```

3. **Footstep Planner Integration**
   ```yaml
   # footstep_planner_params.yaml
   footstep_planner:
     ros__parameters:
       step_size_max: 0.3
       step_size_min: 0.05
       step_width_max: 0.4
       step_width_min: 0.1
       step_height_max: 0.15
       step_rotation_max: 0.5
       support_polygon_margin: 0.05
       traversability_threshold: 0.7
       step_timing: 0.8
       gait_type: "walking"
   ```

### Launch File

```python
# launch/humanoid_nav2.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'footstep_planner_server']

    return LaunchDescription([
        # Footstep planner server
        Node(
            package='humanoid_footstep_planner',
            executable='footstep_planner_server',
            name='footstep_planner_server',
            parameters=[params_file],
            remappings=[('robot/odometry', 'odom'),
                       ('robot/footprint', 'local_costmap/published_footprint')],
            output='screen'
        ),

        # Controller server with humanoid-specific controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[params_file],
            remappings=[('cmd_vel', 'cmd_vel_nav'),
                       ('odom', 'odom'),
                       ('footstep_sequence', 'footstep_planner/footsteps')],
            output='screen'
        ),

        # Standard Nav2 components
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[params_file],
            remappings=[('robot/odometry', 'odom')],
            output='screen'
        ),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[params_file],
            remappings=[('robot/odometry', 'odom')],
            output='screen'
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[params_file],
            remappings=[('robot/odometry', 'odom')],
            output='screen'
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]
        )
    ])
```

## Example 2: Isaac ROS Integration with Navigation

This example demonstrates integrating Isaac ROS perception with Nav2 navigation.

### Perception Pipeline Integration

1. **Isaac ROS Object Detection**
   ```yaml
   # perception_nav_integration.yaml
   detection_fusion_node:
     ros__parameters:
       input_detection_topic: "/isaac_ros/detections"
       input_map_topic: "/map"
       output_obstacle_map_topic: "/local_costmap/obstacle_layer/voxel_grid"
       detection_confidence_threshold: 0.7
       max_detection_range: 5.0
       obstacle_inflation_radius: 0.3
   ```

2. **Launch Configuration**
   ```python
   # launch/perception_nav_integration.launch.py
   from launch import LaunchDescription
   from launch_ros.actions import Node, ComposableNodeContainer
   from launch_ros.descriptions import ComposableNode

   def generate_launch_description():
       perception_container = ComposableNodeContainer(
           name='perception_container',
           namespace='',
           package='rclcpp_components',
           executable='component_container_mt',
           composable_node_descriptions=[
               ComposableNode(
                   package='isaac_ros_detectnet',
                   plugin='nvidia::isaac_ros::detection::DetectNetNode',
                   name='detectnet',
                   parameters=[{
                       'model_name': 'ssd_mobilenet_v2_coco',
                       'input_image_width': 640,
                       'input_image_height': 480,
                       'confidence_threshold': 0.5
                   }],
                   remappings=[
                       ('image', '/camera/rgb/image_rect_color'),
                       ('detections', '/isaac_ros/detections')
                   ]
               ),
               ComposableNode(
                   package='isaac_ros_image_pipeline',
                   plugin='nvidia::isaac_ros::image_pipeline::RectifyNode',
                   name='rectify_node',
                   remappings=[
                       ('image_raw', '/camera/rgb/image_raw'),
                       ('image_rect', '/camera/rgb/image_rect_color')
                   ]
               )
           ]
       )

       return LaunchDescription([
           perception_container,
           # Nav2 nodes with perception integration
           # (as shown in Example 1)
       ])
   ```

## Example 3: Complex Navigation Scenario

This example shows a complex navigation scenario with multiple waypoints and dynamic obstacle avoidance.

### Waypoint Navigation

```python
# scripts/humanoid_waypoint_navigation.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
import time

class HumanoidWaypointNavigator(Node):
    def __init__(self):
        super().__init__('humanoid_waypoint_navigator')
        self._action_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')

    def send_waypoints(self, waypoints):
        """Send a sequence of waypoints to navigate through"""
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = waypoints
        goal_msg.behavior_tree_id = ""  # Use default BT

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        return future

def main():
    rclpy.init()
    navigator = HumanoidWaypointNavigator()

    # Define waypoints (example coordinates)
    waypoints = []

    # Waypoint 1
    pose1 = PoseStamped()
    pose1.header.frame_id = 'map'
    pose1.pose.position.x = 2.0
    pose1.pose.position.y = 1.0
    pose1.pose.orientation.w = 1.0
    waypoints.append(pose1)

    # Waypoint 2
    pose2 = PoseStamped()
    pose2.header.frame_id = 'map'
    pose2.pose.position.x = 4.0
    pose2.pose.position.y = 3.0
    pose2.pose.orientation.w = 1.0
    waypoints.append(pose2)

    # Waypoint 3
    pose3 = PoseStamped()
    pose3.header.frame_id = 'map'
    pose3.pose.position.x = 1.0
    pose3.pose.position.y = 5.0
    pose3.pose.orientation.w = 1.0
    waypoints.append(pose3)

    future = navigator.send_waypoints(waypoints)
    rclpy.spin_until_future_complete(navigator, future)

    result = future.result()
    navigator.get_logger().info(f'Navigation result: {result}')

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Dynamic Obstacle Avoidance Configuration

```yaml
# dynamic_avoidance_params.yaml
local_costmap:
  ros__parameters:
    update_frequency: 20.0
    publish_frequency: 20.0
    global_frame: odom
    robot_base_frame: base_link
    rolling_window: true
    width: 8
    height: 8
    resolution: 0.05
    footprint: [[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]
    plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]

    obstacle_layer:
      enabled: true
      observation_sources: laser_scan camera_detections
      laser_scan:
        topic: /laser_scan
        max_obstacle_height: 2.0
        clearing: true
        marking: true
      camera_detections:
        topic: /isaac_ros/detections
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: "PointCloud2"
        sensor_frame: "camera_link"

    voxel_layer:
      enabled: true
      publish_voxel_map: true
      origin_z: 0.0
      z_resolution: 0.2
      z_voxels: 10
      max_obstacle_height: 2.0
      mark_threshold: 0
      observation_sources: point_cloud

    inflation_layer:
      enabled: true
      cost_scaling_factor: 5.0
      inflation_radius: 0.7
```

## Performance Considerations

### Computational Requirements

- **Real-time Processing**: Navigation pipeline typically runs at 10-20 Hz
- **GPU Utilization**: Isaac ROS components utilize GPU for perception
- **Memory Management**: Efficient memory usage for costmap and path planning
- **Communication Bandwidth**: Sufficient bandwidth for sensor data

### Tuning Parameters

#### Navigation Performance
- **Controller Frequency**: Balance between responsiveness and computation load
- **Costmap Resolution**: Trade-off between accuracy and performance
- **Path Planning Frequency**: Update rate for global path replanning
- **Footstep Planning**: Timing and density of footstep generation

#### Safety Parameters
- **Inflation Radius**: Safety margin around obstacles
- **Velocity Limits**: Conservative limits for stability
- **Goal Tolerance**: Appropriate tolerance for humanoid navigation
- **Recovery Behaviors**: Safe fallback strategies

## Troubleshooting Common Issues

### Navigation Failures

- **Path Planning Failures**: Check costmap configuration and inflation parameters
- **Oscillation**: Adjust controller parameters and goal checker settings
- **Local Minima**: Implement more sophisticated recovery behaviors
- **Footstep Planning Issues**: Verify terrain traversability and step constraints

### Performance Issues

- **High CPU Usage**: Optimize costmap resolution and update frequency
- **Latency**: Check sensor processing pipeline and communication delays
- **Unstable Walking**: Verify balance control parameters and footstep timing
- **Inconsistent Behavior**: Validate sensor calibration and coordinate frames

## Best Practices

### Configuration Management

- Use YAML parameter files for easy configuration changes
- Implement proper logging for debugging navigation issues
- Validate configurations in simulation before real-world testing
- Document parameter meanings and typical ranges

### Safety Implementation

- Implement multiple safety layers and fallback strategies
- Test navigation in controlled environments first
- Monitor robot state continuously during navigation
- Implement emergency stop capabilities

[Continue to Nav2 Assessment](./assessment.md)