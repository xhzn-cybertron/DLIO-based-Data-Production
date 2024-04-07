#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    
    # #-----------------------------get the package of vanjee_lidar's rviz file path--------------------#
    # bringup_dir = get_package_share_directory('vanjee_lidar')#获取包的路径
    # print(bringup_dir)
    # strer = bringup_dir.split('/install')#字符串分割
    # rvizpath = str(strer[0]) + "/src/vanjee_lidar/rviz/Vanjeelidar.rviz" # 字符串拼接
    # parampath = str(strer[0]) + "/src/vanjee_lidar/" # 字符串拼接

    # config = os.path.join(parampath,'param','wlr_720.yaml')
    # print(config)

    # #-----------------------------get the package of vanjee_driver's path----------------------------------#
    # bringup_dir = get_package_share_directory('vanjee_driver')#获取包的路径
    # strer = bringup_dir.split('/install')#字符串分割
    # driver_path = str(strer[0]) + "/src/vanjee_driver/launch" # 字符串拼接

    # #-----------------------------set the package of vanjee_lidar's setting---------------------------#
    # driver_node = LifecycleNode(namespace='',
    #                             package='vanjee_lidar',
    #                             executable='wlr720_pointcloud_node',
    #                             name='wlr720_pointcloud_node',
    #                             output='screen',
    #                             parameters = [config]
    #                             )

    # param_dir = LaunchConfiguration('params_file')

    # DeclareLaunchArgument('params_file',default_value=config,description='Full path to param file to load')
   

    #-----------------------------start the rviz node ,show vanjee_lidar pointcloud2-----------------#
    openzen_node = Node(
        package='openzen_driver',
        executable='openzen_node',
        name='openzen',
        output='screen',
        arguments=['sensor_name', "devicefile:/dev/ttyUSB0"]
        )
    # Get the launch directory
    lidar_driver_dir = get_package_share_directory('lslidar_driver')
    launch_dir = os.path.join(lidar_driver_dir, 'launch')
    ls16_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'lslidar_c16_launch.py')),
        )

     # Get the launch directory
    dlio_dir = get_package_share_directory('direct_lidar_inertial_odometry')
    launch_dir = os.path.join(dlio_dir, 'launch')
    dlio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dlio.launch.py')),
        launch_arguments={'rviz': 'true',
                          'pointcloud_topic': '/c16/lslidar_point_cloud',
                          'imu_topic': '/data'}.items())

    kobuki_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch', 'base_teleop-launch.py')
        )
    )
    return LaunchDescription([
        openzen_node,
        ls16_node,
        # kobuki_node,
        dlio_node
    ])
