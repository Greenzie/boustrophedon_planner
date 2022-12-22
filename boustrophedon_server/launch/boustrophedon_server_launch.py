from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='boustrophedon_server',
            executable='boustrophedon_server',
            name='boustrophedon_server',
            parameters=[{
                "repeat_boundary": False,
                "outline_clockwise": True,
                "skip_outlines": True,
                "stripe_separation": 1.0,
                "outline_layer_count": 0,
                # "intermediary_separation": 0.0, --> default
                "stripe_angle": 0.0,
                # Note: both stripe_angle_from_robot_orientation and stripe_angle_from_boundary_orientation should not be enabled at the same time
                "stripe_angle_from_robot_orientation": False,
                "stripe_angle_from_boundary_orientation": True,
                "travel_along_boundary": True,
                "allow_points_outside_boundary": False,
                # Note: if enabling half-y turns, must have an outline_layer_count >= 1
                "enable_half_y_turns": False,
                "points_per_turn": 15,
                "turn_start_offset": 0.0,
                "publish_polygons": True,
                "publish_path_points": True
            }]
        ),
    ])

