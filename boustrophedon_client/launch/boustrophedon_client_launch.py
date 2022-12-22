from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='boustrophedon_client',
            executable='boustrophedon_client',
            name='boustrophedon_client',
            parameters=[{
                "boundary": 'boundary.txt',
            }]
        ),
    ])

