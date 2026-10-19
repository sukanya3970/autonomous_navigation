import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg

def generate_launch_description():
    pkg_share = FindPackageShare(package='rover').find('rover')
    
    # 1. Static Transform Publisher: base_link -> laser_frame
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_broadcaster',
        arguments=['0.1524', '-0.03', '0.30', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    imu_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_imu_broadcaster',
    arguments=[
        '--x', '0.1397',
        '--y', '0.0',
        '--z', '0.0',
        '--yaw', '0.0',
        '--pitch', '0.0',
        '--roll', '0.0',
        '--frame-id', 'base_link',
        '--child-frame-id', 'imu_link'
    ]
)

    # 2. Arduino Bridge Node
    arduino_bridge_node = Node(
        package='rover',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen'
    )

    # 3. Sensor Parser Node
    sensor_parser_node = Node(
        package='rover',
        executable='sensor_parser',
        name='sensor_parser',
        output='screen'
    )

    # 4. EKF Odometry Node
    custom_ekf_node = Node(
	    package='rover',
	    executable='custom_ekf',
	    name='custom_ekf',
	    output='screen',
	    parameters=[
	        os.path.join(pkg_share, 'config', 'custom_ekf.yaml')]
)

    # 5. Async SLAM Toolbox as a LifecycleNode with parameters loaded
    slam_params_path = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    
    async_slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        namespace='',
        output='screen',
        parameters=[
            slam_params_path,
            {'use_sim_time': False}
        ]
    )

    # Auto-configure slam_toolbox once spawned
    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(async_slam_toolbox_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        )
    )

    # Auto-activate slam_toolbox once configured
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=async_slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(async_slam_toolbox_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    # 6. RPLIDAR Node (Using serial port /dev/ttyUSB0 or your current mapping)
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
		    'channel_type': 'serial',
		    'serial_port': '/dev/ttyUSB0',
		    'serial_baudrate': 115200,
		    'frame_id': 'laser',
		    'inverted': False,
		    'angle_compensate': True,
		    'scan_mode': 'Sensitivity',
		    'scan_frequency': 10.0,
}]
    )

    # Optional safety timer delay for LIDAR stream synchronization
    delayed_lidar_node = TimerAction(
        period=5.0,
        actions=[sllidar_node]
    )

    return LaunchDescription([
        base_to_laser_tf,
        imu_tf,
        arduino_bridge_node,
        sensor_parser_node,
        custom_ekf_node,
        async_slam_toolbox_node,
        configure_slam,
        activate_slam,
        delayed_lidar_node
    ])
