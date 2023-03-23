import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
  rviz_launch_dir=os.path.join(get_package_share_directory('tortoisebotpromax_description'), 'launch')
  ydlidar_launch_dir=os.path.join(get_package_share_directory('ydlidar'), 'launch')
  # realsense_launch_dir=os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
  use_sim_time=LaunchConfiguration('use_sim_time')
  
  rviz_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'rviz.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time}.items())


  ydlidar_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_launch_dir, 'x4_ydlidar_launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_sim_time])),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  # camera_launch_cmd=IncludeLaunchDescription(
  #       PythonLaunchDescriptionSource(
  #           os.path.join(realsense_launch_dir, 'rs_launch.py')),
  #           condition=IfCondition(PythonExpression(['not ', use_sim_time])),
  #           parameters=[
  #                   {'pointcloud.enable': True},
  #                   {'node_names': ['map_server']}]),
  #           launch_arguments={'use_sim_time':use_sim_time}.items())
    #imu
  return LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
    Node(
        package='tortoisebotpromax_firmware',
        executable='differential_publisher',
    ),
    # Node(
    #     package='tortoisebotpromax_firmware',
    #     executable='odom_to_ticks',
    # ),

    # Node(
    #         package='realsense2_camera', executable='realsense2_camera_node',
    #         condition=IfCondition(PythonExpression(['not ', use_sim_time])),
    #         parameters=[{
    #           {'enable_pointcloud': True},
    #           {'align_depth':True},
    #           {'enable_sync':True},
    #         }],
    #     ),
    Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        remappings=[('cloud_in','/camera/depth/color/points'),
                    ('/scan','/scan')],
        parameters=[{
            'target_frame': 'camera_link',
            'transform_tolerance': 0.01,
            'min_height': 0.0, #height of target frame is considered height 0.0m nad realsense tolerance is 0.05m
            'max_height': 1.0,
            'angle_min': -1.5708,  # -M_PI/2
            'angle_max': 1.5708,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.3,
            'range_max': 4.0,
            'use_inf': True,
            'inf_epsilon': 1.0

        }],
        name='pointcloud_to_laserscan'
    ),
    # rviz_launch_cmd, 
    # ydlidar_launch_cmd,

  ]
)
