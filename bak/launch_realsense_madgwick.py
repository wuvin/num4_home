import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):

    rs_config = LaunchConfiguration('rs_config').perform(context)

    debug_config = LogInfo(msg=[
        f'Using RealSense config file: {rs_config}'
    ])
    
    bringup_share_dir = get_package_share_directory('realsense2_bringup')
    camera_share_dir = get_package_share_directory('realsense2_camera')
    rs_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(camera_share_dir, 'launch'),
        '/rs_launch.py'
    ]),
    launch_arguments={'config_file': os.path.join(bringup_share_dir, 'config', rs_config)}.items())

    imu_filter = Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/camera/imu'),
                        ('/imu/data', '/rs_imu/data')])

    return [
        debug_config,
        rs_launch,
        imu_filter
    ]

def generate_launch_description():

    rs_config_arg = DeclareLaunchArgument(
        'rs_config',
        default_value='/local/config_realsense_madgwick.yaml',
        description='Path to config file')
    
    opaque_launch = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        rs_config_arg,
        opaque_launch
    ])
