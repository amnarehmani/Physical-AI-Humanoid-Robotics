import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the ros_gz_sim package
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # 2. Define the Gazebo launch command
    # We launch 'gz_sim.launch.py' and pass arguments to load an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Define the Bridge
    # This Node forwards messages between ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Bridge the /scan topic (LiDAR)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Bridge the /cmd_vel topic (Movement commands)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Bridge the /camera/image_raw topic (Camera)
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # Bridge the /imu/data topic (IMU)
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    # 4. Return the LaunchDescription
    return LaunchDescription([
        gazebo,
        bridge
    ])