#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    EmitEvent,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare

import lifecycle_msgs.msg


def generate_launch_description():

    # =========================================================================
    # Package paths
    # =========================================================================

    pkg_share = FindPackageShare(package="rover").find("rover")

    slam_params_path = os.path.join(
        pkg_share,
        "config",
        "slam_localization_nav.yaml",
    )

    nav2_params_path = os.path.join(
        pkg_share,
        "config",
        "nav2_params.yaml",
    )

    ekf_params_path = os.path.join(
        pkg_share,
        "config",
        "custom_ekf.yaml",
    )

    # =========================================================================
    # Launch arguments
    # =========================================================================

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true",
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        description=(
            "Full path to the serialized slam_toolbox pose-graph "
            "base name without .posegraph/.data"
        ),
    )

    # =========================================================================
    # Common TF remappings
    # =========================================================================

    tf_remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    # =========================================================================
    # Robot TF
    # =========================================================================

    # base_link -> laser
    base_to_laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_broadcaster",
        arguments=[
            "0.24",
            "-0.04",
            "0.45",
            "0.0",
            "0.0",
            "0.0",
            "base_link",
            "laser",
        ],
        output="screen",
    )

    # base_link -> imu_link
    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_broadcaster",
        arguments=[
            "--x",
            "0.1397",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--yaw",
            "0.0",
            "--pitch",
            "0.0",
            "--roll",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "imu_link",
        ],
        output="screen",
    )

    # =========================================================================
    # Robot I/O
    # =========================================================================

    arduino_bridge_node = Node(
        package="rover",
        executable="arduino_bridge",
        name="arduino_bridge",
        output="screen",
    )

    sensor_parser_node = Node(
        package="rover",
        executable="sensor_parser",
        name="sensor_parser",
        output="screen",
    )

    custom_ekf_node = Node(
        package="rover",
        executable="custom_ekf",
        name="custom_ekf",
        output="screen",
        parameters=[
            ekf_params_path,
        ],
    )

    # =========================================================================
    # LiDAR
    # =========================================================================

    sllidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 115200,
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Sensitivity",
                "scan_frequency": 10.0,
            }
        ],
    )

    # Start LiDAR after the rest of the localization stack has had time
    # to initialize.
    delayed_lidar_node = TimerAction(
        period=5.0,
        actions=[
            sllidar_node,
        ],
    )

    # =========================================================================
    # SLAM Toolbox - Localization
    #
    # map -> odom is provided by SLAM Toolbox.
    # odom -> base_link is provided by custom EKF.
    # =========================================================================

    slam_localization_node = LifecycleNode(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            slam_params_path,
            {
                "use_sim_time": use_sim_time,
                "map_file_name": map_file,
                "map_start_pose": [0.0, 0.0, 0.0],
                "use_lifecycle_manager": False,
            },
        ],
        remappings=tf_remappings,
    )

    # Configure SLAM Toolbox.
    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(
                slam_localization_node
            ),
            transition_id=(
                lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
            ),
        )
    )

    # Activate SLAM Toolbox after successful configuration.
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_localization_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(
                            slam_localization_node
                        ),
                        transition_id=(
                            lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                        ),
                    )
                )
            ],
        )
    )

    # =========================================================================
    # Nav2 Controller
    #
    # Output:
    #   /cmd_vel_nav
    # =========================================================================

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            nav2_params_path,
        ],
        remappings=tf_remappings + [
            ("cmd_vel", "cmd_vel_nav"),
        ],
    )

    # =========================================================================
    # Nav2 Smoother Server
    # =========================================================================

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            nav2_params_path,
        ],
        remappings=tf_remappings,
    )

    # =========================================================================
    # Nav2 Planner
    # =========================================================================

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            nav2_params_path,
        ],
        remappings=tf_remappings,
    )

    # =========================================================================
    # Nav2 Behavior Server
    #
    # Spin / BackUp / Wait output:
    #   /cmd_vel_nav
    #
    # This is important because recovery behaviors must enter the same
    # velocity-safety pipeline as normal navigation.
    # =========================================================================

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            nav2_params_path,
        ],
        remappings=tf_remappings + [
            ("cmd_vel", "cmd_vel_nav"),
        ],
    )

    # =========================================================================
    # BT Navigator
    # =========================================================================

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            nav2_params_path,
        ],
        remappings=tf_remappings,
    )

    # =========================================================================
    # Waypoint Follower
    # =========================================================================

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[
            nav2_params_path,
        ],
        remappings=tf_remappings,
    )

    # =========================================================================
    # Velocity Smoother
    #
    # INPUT:
    #   /cmd_vel_nav
    #
    # OUTPUT:
    #   /cmd_vel_smoothed
    #
    # IMPORTANT:
    # Only the INPUT is remapped.
    #
    # The Nav2 velocity_smoother publishes its normal output on
    # /cmd_vel_smoothed.
    # =========================================================================

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[
            nav2_params_path,
        ],
        remappings=tf_remappings + [
            ("cmd_vel", "cmd_vel_nav"),
        ],
    )

    # =========================================================================
    # Collision Monitor
    #
    # INPUT:
    #   /cmd_vel_smoothed
    #
    # OUTPUT:
    #   /cmd_vel
    #
    # The params file defines:
    #
    #   cmd_vel_in_topic:  cmd_vel_smoothed
    #   cmd_vel_out_topic: cmd_vel
    #
    # Therefore NO cmd_vel remapping is required here.
    # =========================================================================

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            nav2_params_path,
        ],
        remappings=tf_remappings,
    )

    # =========================================================================
    # Nav2 Lifecycle Manager
    #
    # These are the lifecycle-managed Nav2 nodes.
    #
    # SLAM Toolbox is intentionally NOT included because it has its own
    # explicit lifecycle configuration above.
    # =========================================================================

    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                    "collision_monitor",
                ],
            }
        ],
        remappings=tf_remappings,
    )

    # =========================================================================
    # Launch description
    # =========================================================================

    return LaunchDescription(
        [
            # -----------------------------------------------------------------
            # Arguments
            # -----------------------------------------------------------------

            use_sim_time_arg,
            map_file_arg,

            # -----------------------------------------------------------------
            # Robot TF
            # -----------------------------------------------------------------

            base_to_laser_tf,
            imu_tf,

            # -----------------------------------------------------------------
            # Robot I/O
            # -----------------------------------------------------------------

            arduino_bridge_node,
            sensor_parser_node,
            custom_ekf_node,

            # -----------------------------------------------------------------
            # SLAM localization
            # -----------------------------------------------------------------

            slam_localization_node,
            configure_slam,
            activate_slam,

            # -----------------------------------------------------------------
            # LiDAR
            # -----------------------------------------------------------------

            delayed_lidar_node,

            # -----------------------------------------------------------------
            # Nav2
            # -----------------------------------------------------------------

            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,

            # -----------------------------------------------------------------
            # Velocity safety pipeline
            # -----------------------------------------------------------------

            velocity_smoother,
            collision_monitor,

            # -----------------------------------------------------------------
            # Lifecycle manager
            # -----------------------------------------------------------------

            lifecycle_manager_navigation,
        ]
    )
