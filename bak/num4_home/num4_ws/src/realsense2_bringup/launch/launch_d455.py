import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):

    config = LaunchConfiguration('config').perform(context)
    debug  = LogInfo(msg=f"RealSense config file: {config}")
    filter = LaunchConfiguration('use_imu_filter').perform(context)
    imulog = LogInfo(msg=f"IMU filter: {filter}")

    bringup_share_dir = get_package_share_directory('realsense2_bringup')
    camera_share_dir  = get_package_share_directory('realsense2_camera' )

    launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_share_dir, 'launch/rs_launch.py')
        ),
        launch_arguments={
            'config_file': os.path.join(bringup_share_dir, 'config', config)
        }.items()
    )

    nodes = [debug, imulog, launch]

    if filter.lower() == 'true':
        imu_filter = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='rs_imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False
            }],
            remappings=[
                ('/imu/data_raw', '/camera/camera/imu'),
                ('/imu/data',     '/madgwick/d455/imu')
            ]
        )

        nodes.append(imu_filter)

    return nodes

def generate_launch_description():

    launch_arg = DeclareLaunchArgument(
        'config',
        default_value='config.yaml',
        description='File path to config yaml'
    )

    filter_arg = DeclareLaunchArgument(
        'use_imu_filter',
        default_value='true',
        description='Toggle Madgwick filter'
    )

    return LaunchDescription([
        launch_arg,
        filter_arg,
        OpaqueFunction(function=launch_setup)
    ])
