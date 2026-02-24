from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ---------------------------------------------------------
    # Launch argument
    # ---------------------------------------------------------

    side_arg = DeclareLaunchArgument(
        'side',
        default_value='left',
        description='Hand side: left or right'
    )

    side = LaunchConfiguration('side')

    # ---------------------------------------------------------
    # Build prefix correctly (side + "_")
    # ---------------------------------------------------------

    prefix_value = PythonExpression([
        "'", side, "' + '_'"
    ])

    # ---------------------------------------------------------
    # Paths
    # ---------------------------------------------------------

    xacro_path = PathJoinSubstitution([
        FindPackageShare('rh56dfx_description'),
        'urdf',
        'rh56dfx.xacro'
    ])

    # RViz config depends on side
    rviz_config_file = PythonExpression([
        "'rh56dfx_' + '", side, "' + '.rviz'"
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('rh56dfx_description'),
        'rviz',
        rviz_config_file
    ])

    # ---------------------------------------------------------
    # Robot description (xacro call)
    # ---------------------------------------------------------

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_path,
            ' ',
            'side:=', side,
            ' ',
            'prefix:=', prefix_value,
            ' ',
            'use_world:=false'
        ]),
        value_type=str
    )

    # ---------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------

    return LaunchDescription([

        side_arg,

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])
