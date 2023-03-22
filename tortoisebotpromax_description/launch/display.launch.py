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
  gazebo_launch_dir=os.path.join(get_package_share_directory('tortoisebotpromax_gazebo'), 'launch')
  rvizconfig=os.path.join(get_package_share_directory('tortoisebotpromax_description'), 'rviz/sensors_display.rviz')

  use_sim_time=LaunchConfiguration('use_sim_time')
  rviz_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'rviz.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time, 'rvizconfig':rvizconfig}.items())

  gazebo_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
            condition=IfCondition(use_sim_time),
            launch_arguments={'use_sim_time':use_sim_time}.items())


  return LaunchDescription([

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),


    rviz_launch_cmd,
    gazebo_launch_cmd,



  ]
)
