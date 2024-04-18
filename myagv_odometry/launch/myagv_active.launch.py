import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 파라미터 파일 경로 설정
    urdf_file_name = 'myAGV.urdf'
   # myagv_odometry_share_dir = FindPackageShare('myagv_odometry')
    param_file_path = os.path.join('myagv_odometry', 'urdf', 'myAGV.urdf')

    urdf = os.path.join(
	 get_package_share_directory('urdf'),
	 urdf_file_name)
    #param_file_path = os.path.join(
        #FindPackageShare('myagv_odometry'), 'urdf', 'myAGV.urdf')

    # Launch 설명 생성
    return LaunchDescription([
        # myagv_odometry_node 노드 실행
        Node(
            package='myagv_odometry',
            executable='myagv_odometry_node',
            name='myagv_odometry_node',
            output='screen',
           # parameters=[{'robot_description': param_file_path}]
        ),
        # joint_state_publisher 노드 실행
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),
        # robot_state_publisher 노드 실행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
           # parameters=[{'robot_description': param_file_path}]
        ),

        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
           # parameters=[],
        )
    ])
