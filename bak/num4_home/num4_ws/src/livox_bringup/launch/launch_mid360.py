import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def init_launch_node(context, *args, **kwargs):

    MULTI_TOPIC     = 0 # 0-All LiDARs same topic, 1-One LiDAR one topic
    DATA_SRC        = 0 # 0-lidar, others-Invalid data src
    OUTPUT_TYPE     = 0
    FRAME_ID        = 'livox_frame'
    LVX_FILE_PATH   = '/home/livox/livox_test.lvx'
    CMDLINE_BD_CODE = 'livox0000000001'

    config_file = LaunchConfiguration('config').perform(context)
    config_xfer = LaunchConfiguration('xfer_format').perform(context)
    debug_xfer  = LogInfo(msg=f"Point cloud format: {config_xfer}")
    config_freq = LaunchConfiguration('publish_freq').perform(context)
    debug_freq  = LogInfo(msg=f"Publish frequency: {config_freq} Hz")
    use_rviz    = LaunchConfiguration('use_rviz').perform(context)
    filter      = LaunchConfiguration('use_imu_filter').perform(context)
    imulog      = LogInfo(msg=f"IMU filter: {filter}")

    bringup_share_dir = get_package_share_directory('livox_bringup')
    driver_share_dir  = get_package_share_directory('livox_ros_driver2')

    bringup_config_file = os.path.join(bringup_share_dir, 'config', config_file)
    driver_config_file  = os.path.join(driver_share_dir, 'config', config_file)
    if os.path.isfile(bringup_config_file):
        config_path = bringup_config_file
        debug_file  = LogInfo(msg=f"Livox config file (bringup): {config_file}")
    elif os.path.isfile(driver_config_file):
        config_path = driver_config_file
        debug_file  = LogInfo(msg=f"Livox config file (driver): {config_file}")
    else:
        config_path = config_file
        debug_file  = LogInfo(msg=f"Livox config file: {config_file}")

    livox_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            "xfer_format"           : int(config_xfer),
            "multi_topic"           : MULTI_TOPIC,
            "data_src"              : DATA_SRC,
            "publish_freq"          : float(config_freq),
            "output_data_type"      : OUTPUT_TYPE,
            "frame_id"              : FRAME_ID,
            "lvx_file_path"         : LVX_FILE_PATH,
            "user_config_path"      : config_path,
            "cmdline_input_bd_code" : CMDLINE_BD_CODE
        }]
    )

    nodes = [debug_file, debug_xfer, debug_freq, imulog, livox_node]

    if use_rviz.lower() == 'true':
        rviz_path = os.path.join(driver_share_dir, 'config', 
                                 'display_point_cloud_ROS2.rviz')
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_path]
        )
        nodes.append(rviz_node)

    if filter.lower() == 'true':
        imu_filter = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='livox_imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False
            }],
            remappings=[
                ('/imu/data_raw', '/livox/imu'),
                ('/imu/data',     '/madgwick/mid360/imu')
            ]
        )

        nodes.append(imu_filter)

    return nodes

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch in RViz'
        ),
        DeclareLaunchArgument(
            'config',
            default_value='MID360_config.json',
            description='File path for Livox configuration JSON'
        ),
        DeclareLaunchArgument(
            'xfer_format',
            default_value='1',
            description='Transfer format: 0-PointCloud2, 1-CustomMsg'
        ),
        DeclareLaunchArgument(
            'publish_freq',
            default_value='10.0',
            description='Publisher rate in Hz (5.0, 10.0, 20.0, 50.0, etc.)'
        ),
        DeclareLaunchArgument(
            'use_imu_filter',
            default_value='true',
            description='Toggle Madgwick filter'
        )
    ]

    launch_args += [OpaqueFunction(function=init_launch_node)]

    return LaunchDescription(launch_args)