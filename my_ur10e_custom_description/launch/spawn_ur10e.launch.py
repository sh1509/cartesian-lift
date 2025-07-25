from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
# import os


def generate_launch_description():
    # set gazebo model path
    # os.environ["GAZEBO_MODEL_PATH"] = os.environ.get("GAZEBO_MODEL_PATH", "") + ':/workspace/install/ur_description/share'
    pkg_name = 'my_ur10e_custom_description'
    urdf_file = 'ur10e_with_prismatic.urdf.xacro'

    robot_name = 'ur10e'
    ur_type = 'ur10e'

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare(pkg_name),
            'urdf',
            urdf_file
        ]),
        ' name:=', robot_name,
        ' ur_type:=', ur_type
    ])

    return LaunchDescription([
        # Start Gazebo first
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Then launch robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'robot_description': robot_description}],
        ),

        # Delay spawn_entity to wait for Gazebo services to be available
        TimerAction(
            period=5.0,  # seconds
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-topic', 'robot_description',
                        '-entity', robot_name,
                        '-x', '0', '-y', '0', '-z', '0.1'
                    ],
                    output='screen'
                )
            ]
        )
    ])
