from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package=pkg_name,
            executable='wall_marker_publisher',
            name='wall_marker_publisher',
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'robot_description': robot_description}],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[
                        PathJoinSubstitution([
                            FindPackageShare(pkg_name),
                            "config",
                            "ur10e_controllers.yaml"
                        ]),
                    ],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_trajectory_controller'],
                    output='screen'
                )
            ]
        ),

        # Delay spawn_entity to wait for Gazebo services to be available
        TimerAction(
            period=6.0,  # seconds
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-topic', 'robot_description',
                        '-entity', robot_name,
                        '-x', '0', '-y', '0', '-z', '0.0'
                    ],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-file',
                        PathJoinSubstitution([
                            FindPackageShare(pkg_name),
                            'models',
                            'wall',
                            'wall.sdf'
                        ]),
                        '-entity', 'wall',
                        '-x', '0.3', '-y', '0.0', '-z', '0.0'
                    ],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', PathJoinSubstitution([
                        FindPackageShare(pkg_name),
                        'config',
                        'ur10e.rviz'
                    ])],
                    parameters=[{'robot_description': robot_description}]
                )
            ]
        )

    ])
