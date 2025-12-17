import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('hros5_gazebo')
    description_share = get_package_share_directory('hros5_description')

    default_world = os.path.join(pkg_share, 'worlds', 'empty.world')
    dae_meshes = os.path.join(
        description_share, 'urdf', 'hros5_visuals_collisions_endoskeleton.xacro')
    stl_meshes = os.path.join(
        description_share, 'urdf', 'hros5_visuals_collisions_endoskeleton_stl.xacro')
    xacro_file = os.path.join(pkg_share, 'urdf', 'hros5_gazebo.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'hros5_controllers.yaml')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    controller_config = LaunchConfiguration('controller_config')
    use_dae_meshes = LaunchConfiguration('use_dae_meshes')
    meshes_xacro = LaunchConfiguration('meshes_xacro')
    controller_manager = LaunchConfiguration('controller_manager')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    gui_config = LaunchConfiguration('gui_config')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' controller_config:=', controller_config,
            ' meshes_xacro_filename:=', meshes_xacro
        ]),
        value_type=str
    )

    # Gazebo resolves model:// paths by concatenating the resource entries with the URI.
    # Use the share roots (no package name suffix) so model://hros5_description/... resolves.
    pkg_share_root = os.path.join(get_package_prefix('hros5_gazebo'), 'share')
    desc_share_root = os.path.join(get_package_prefix('hros5_description'), 'share')
    resource_path = os.pathsep.join([pkg_share_root, desc_share_root, pkg_share, description_share])
    plugin_path = os.pathsep.join([
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        os.environ.get('GZ_SYSTEM_PLUGIN_PATH', ''),
        '/opt/ros/jazzy/lib'
    ])

    # Delay spawns slightly to let gz sim start cleanly.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_hros5',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }],
        arguments=[
            '-name', robot_name,
            '-param', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-R', spawn_roll,
            '-P', spawn_pitch,
            '-Y', spawn_yaw
        ]
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', controller_manager
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    spawn_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'whole_body_controller',
            '--controller-manager', controller_manager
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Path to world SDF file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'robot_name',
            default_value='hros5',
            description='Name for the spawned Gazebo entity'),
        DeclareLaunchArgument(
            'controller_config',
            default_value=controllers_file,
            description='ros2_control parameters file'),
        DeclareLaunchArgument(
            'controller_manager',
            default_value='/controller_manager',
            description='Controller manager namespace exposed by gz_ros2_control'),
        DeclareLaunchArgument(
            'use_dae_meshes',
            default_value='false',
            description='Use DAE visuals (better in RViz, can crash Gazebo if meshes are incompatible)'),
        DeclareLaunchArgument(
            'meshes_xacro',
            default_value=PythonExpression([
                "'", dae_meshes, "' if '", use_dae_meshes,
                "' == 'true' else '", stl_meshes, "'"
            ]),
            description='Mesh bundle xacro used when building the robot_description'),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='0.0',
            description='Initial spawn X position in Gazebo'),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0',
            description='Initial spawn Y position in Gazebo'),
        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.5',
            description='Initial spawn Z position (raise robot above ground)'),
        DeclareLaunchArgument(
            'spawn_roll',
            default_value='0.0',
            description='Initial roll (radians)'),
        DeclareLaunchArgument(
            'spawn_pitch',
            default_value='0.0',
            description='Initial pitch (radians)'),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='3.1416',
            description='Initial yaw (radians)'),
        DeclareLaunchArgument(
            'gui_config',
            default_value='',
            description='Optional path to a saved Gazebo GUI config to enforce view/zoom'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('GZ_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),
        SetEnvironmentVariable('GZ_SYSTEM_PLUGIN_PATH', plugin_path),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugin_path),
        SetEnvironmentVariable(
            'GZ_GUI_CONFIG_PATH',
            gui_config,
            condition=IfCondition(PythonExpression(["'", gui_config, "' != ''"]))
        ),

        ExecuteProcess(
            condition=IfCondition(PythonExpression(["'", gui_config, "' != ''"])),
            cmd=['gz', 'sim', '-r', '--gui-config', gui_config, world],
            output='screen'
        ),
        ExecuteProcess(
            condition=UnlessCondition(PythonExpression(["'", gui_config, "' != ''"])),
            cmd=['gz', 'sim', '-r', world],
            output='screen'
        ),

        # Bridge Gazebo sim time into ROS clock so controller_manager sees /clock
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_clock_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        TimerAction(period=2.0, actions=[spawn_entity]),
        TimerAction(period=4.0, actions=[spawn_joint_state_broadcaster]),
        TimerAction(period=4.5, actions=[spawn_trajectory_controller]),
    ])
