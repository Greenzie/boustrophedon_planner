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
                "outline_layer_count": 0,
                "stripe_separation": 1.0,
                "intermediary_separation": 0.0,
                "stripe_angle": 0.0,
                "enable_stripe_angle_orientation": True,
                "travel_along_boundary": True,
                "allow_points_outside_boundary": False,
                # Note: if enabling half-y turns, must have an outline_layer_count >= 1
                "enable_half_y_turns": False,
                "points_per_turn": 15,
                "turn_start_offset": 0.5,
                "publish_polygons": True,
                "publish_path_points": True
            }]
        ),
    ])

