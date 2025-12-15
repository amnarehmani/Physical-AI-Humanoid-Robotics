import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Define the Visual SLAM Node
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        remappings=[
            ('stereo_camera/left/image', '/camera/left/image_rect'),
            ('stereo_camera/right/image', '/camera/right/image_rect'),
            ('visual_slam/odom', '/odom')
        ],
        parameters=[{
            'enable_rectified_pose': True,
            'denoise_input_images': False,
            'rectified_images': True,
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.000244,
            'accel_noise_density': 0.001862
        }]
    )

    # Run it in a Container (Standard for Isaac ROS)
    container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )

    return LaunchDescription([container])
