from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Launch the QR reader nodes and camera drivers."""
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    return LaunchDescription([
        # Launch Realsense camera for qr_srv
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path)
        ),

        # Launch USB camera for qr_pub
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen'
        ),

        # Launch qr_srv node, remapping its input image topic to the realsense camera
        Node(
            package='ros_qr_reader',
            executable='qr_srv',
            name='qr_service_node',
            output='screen',
            remappings=[
                ('/image_raw', '/camera/color/image_raw')
            ]
        ),

        # Launch qr_pub node, remapping its input image topic to the usb_cam
        Node(
            package='ros_qr_reader',
            executable='qr_pub',
            name='qr_publisher_node',
            output='screen',
            remappings=[
                ('/camera/color/image_raw', '/image_raw')
            ]
        ),
    ])
