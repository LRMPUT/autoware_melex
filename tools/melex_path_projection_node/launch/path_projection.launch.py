from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    path_projection = Node(
        package='melex_path_projection_node',
        executable='melex_path_projection_node.py'
    )

    image = Node(
        package='image_view',
        executable='image_view',
        parameters=[{'image': '/melex/path_projection/image',
                     'autosize': True,
                     'window_name': 'Paths comparison'}],
        remappings=[
           ('image', '/melex/path_projection/image')
        ]
    )

    return LaunchDescription([
        image,
        path_projection,
    ])