from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='segway_rmp_ros2',
            executable='segway_rmp_node_ros2',
            name='segway_rmp_node_ros2',
            output='screen',
            parameters=[
                {'interface_type': 'serial'},
                {'serial_port': '/dev/ttyUSB0'},
                {'motor_timeout': 1.0},
                {'frame_id': 'base_footprint'},
                {'invert_linear_vel_cmds': False},
                {'invert_angular_vel_cmds': False},
                {'broadcast_tf': True},
                {'rmp_type': '200/400'},
                # Uncomment the following parameters as needed
                {'linear_pos_accel_limit': 1.0},
                {'linear_neg_accel_limit': 1.0},
                {'angular_pos_accel_limit': 45.0},
                {'angular_neg_accel_limit': 45.0},
                {'max_linear_vel': 0.75},
                {'max_angular_vel': 0.5}, # 0.5
                {'linear_odom_scale': 1.6},
                {'angular_odom_scale': 1.8}, #1.6
                # {'reset_odometry': True},
                # {'odometry_reset_duration': 1.0}
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),  # Remap cmd_vel
            ],
        ),
    ])
