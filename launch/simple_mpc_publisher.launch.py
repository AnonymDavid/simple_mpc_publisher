from launch import LaunchDescription, LaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    trajectory_params = os.path.join(
        get_package_share_directory('simple_mpc_publisher'),
        'config',
        'trajectory_params.yaml'
    )
    
    simple_mpc_pub = Node(
        package='simple_mpc_publisher',
        executable='simple_mpc_publisher',
        name='simple_mpc_publisher',
        output='screen',
        parameters=[trajectory_params]
    )

    ### MPC ###

    vehicle_info_param = os.path.join(
        get_package_share_directory('simple_mpc_publisher'),
        'config',
        'vehicle_info.param.yaml'
    )

    nearest_search_param = os.path.join(
        get_package_share_directory('simple_mpc_publisher'),
        'config',
        'nearest_search.param.yaml'
    )

    trajectory_follower_node_param = os.path.join(
        get_package_share_directory('simple_mpc_publisher'),
        'config',
        'trajectory_follower_node.param.yaml'
    )

    lat_controller_param = os.path.join(
        get_package_share_directory('simple_mpc_publisher'),
        'config',
        'mpc.param.yaml'
    )

    lon_controller_param = os.path.join(
        get_package_share_directory('simple_mpc_publisher'),
        'config',
        'pid.param.yaml'
    )

    raw_vehicle_converter = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('raw_vehicle_cmd_converter'),
                'launch',
                'raw_vehicle_converter.launch.xml'
            )
        )
    )

    pacmod_interface = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pacmod_interface'),
                'launch',
                'pacmod_interface.launch.xml'
            ),
        ),
        launch_arguments={
            "vehicle_model": "lexus3_vehicle",
        }.items()
    )

    mpc = Node(
        package="trajectory_follower_node",
        executable="controller_node_exe",
        name="controller_node_exe",
        namespace="trajectory_follower",
        remappings=[
            ("~/input/reference_trajectory", "/planning/scenario_planning/trajectory"),
            ("~/input/current_odometry", "/localization/kinematic_state"),
            ("~/input/current_steering", "/vehicle/status/steering_status"),
            ("~/input/current_accel", "/localization/acceleration"),
            ("~/input/current_operation_mode", "/system/operation_mode/state"),
            ("~/output/predicted_trajectory", "lateral/predicted_trajectory"),
            ("~/output/lateral_diagnostic", "lateral/diagnostic"),
            ("~/output/slope_angle", "longitudinal/slope_angle"),
            ("~/output/longitudinal_diagnostic", "longitudinal/diagnostic"),
            ("~/output/control_cmd", "/control/command/control_cmd"),
        ],
        parameters=[
            {
                "lateral_controller_mode": "mpc",
                "longitudinal_controller_mode": "pid",
            },
            nearest_search_param,
            trajectory_follower_node_param,
            lon_controller_param,
            lat_controller_param,
            vehicle_info_param,
        ],
        output="screen",
    )
    

    return LaunchDescription([
        raw_vehicle_converter,
        pacmod_interface,
        simple_mpc_pub,
        mpc
    ])