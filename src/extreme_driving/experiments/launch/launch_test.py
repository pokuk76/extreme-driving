

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ## exteme_driving project
    experiment_config = os.path.join(
        get_package_share_directory('experiments'),
        'config',
        'experiment.yaml'
    )

    ## f1tenth_system project
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'sensors.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )

    experiment_la = DeclareLaunchArgument(
        'experiment_config',
        default_value=experiment_config,
        description='experiment node launch argument'
    )
    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Descriptions for sensor configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')

    ## ExecuteProcess is being used to automatically start rosbag collection
    # TODO: Test QoS overrides (hoping to use it to collect data for a set duration)
    ld = LaunchDescription([experiment_la, joy_la, vesc_la, sensors_la, mux_la,
                            # Delete old test_bags/
                            ExecuteProcess(
                                cmd=['rm', '-rf', 'test_bags/'],
                                output='screen'
                            ),
                            ExecuteProcess(
                                cmd=['ros2', 'bag', 'record', '-o', 'test_bags', '/odom', '/sensors/imu', '/sensors/imu/raw', '/odometry/filtered'],
                                # cmd=['ros2', 'bag', 'record', '--qos-profile-overrides-path', '/home/f1tenth2/f1tenth_ws/src/extreme_driving/experiments/config/qos_profile.yaml', '-o', 'test_bags', '/odom' '/sensors/imu' '/sensors/imu/raw'],
                                output='screen'
                            ),
                            Node(
                                package='robot_localization',
                                executable='ekf_node',
                                name='ekf_filter_node',
                                output='screen',
                                parameters=[os.path.join(get_package_share_directory("experiments"), 'config', 'ekf.yaml')],
                            ),])


    experiment_node = Node(
        package='experiments',
        executable='experiment',
        name='experiments',
        parameters=[LaunchConfiguration('experiment_config')]
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('sensors_config')]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    # finalize
    ld.add_action(experiment_node)
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(throttle_interpolator_node)
    # ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_node)

    return ld