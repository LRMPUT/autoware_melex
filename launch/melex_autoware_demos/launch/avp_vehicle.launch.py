# Copyright 2020, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    avp_demo_pkg_prefix = get_package_share_directory('autoware_auto_avp_demo')
    melex_demo_pkg_prefix = get_package_share_directory('melex_autoware_demos')
    ouster_pkg_prefix = get_package_share_directory('melex_ros2_ouster')
    map_publisher_param_file = os.path.join(
        melex_demo_pkg_prefix, 'param/avp/map_publisher_vehicle.param.yaml')
    ndt_localizer_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/ndt_localizer_vehicle.param.yaml')
    mpc_param_file = os.path.join(
        melex_demo_pkg_prefix, 'param/avp/mpc_vehicle.param.yaml')

    pc_filter_transform_param_file = os.path.join(
        melex_demo_pkg_prefix, 'param/avp/pc_filter_transform.param.yaml')

    vehicle_characteristics_param_file = os.path.join(
        melex_demo_pkg_prefix, 'param/vehicle_characteristics.param.yaml')

    urdf_pkg_prefix = get_package_share_directory('melex_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/melex.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()

    # Arguments

    with_lidars_param = DeclareLaunchArgument(
        'with_lidars',
        default_value='True',
        description='Launch lidar drivers in addition to other nodes'
    )
    map_publisher_param = DeclareLaunchArgument(
        'map_publisher_param_file',
        default_value=map_publisher_param_file,
        description='Path to config file for Map Publisher'
    )
    ndt_localizer_param = DeclareLaunchArgument(
        'ndt_localizer_param_file',
        default_value=ndt_localizer_param_file,
        description='Path to config file for ndt localizer'
    )
    mpc_param = DeclareLaunchArgument(
        'mpc_param_file',
        default_value=mpc_param_file,
        description='Path to config file for MPC'
    )

    pc_filter_transform_param = DeclareLaunchArgument(
        'pc_filter_transform_param_file',
        default_value=pc_filter_transform_param_file,
        description='Path to config file for Point Cloud Filter/Transform Nodes'
    )

    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=vehicle_characteristics_param_file,
        description='Path to config file for vehicle characteristics'
    )

    # Nodes
    filter_transform_os1_128 = Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_os1_128',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        remappings=[("points_in", "points")]
    )
    map_publisher = Node(
        package='ndt_nodes',
        executable='ndt_map_publisher_exe',
        namespace='localization',
        parameters=[LaunchConfiguration('map_publisher_param_file')]
    )
    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )
    ndt_localizer = Node(
        package='ndt_nodes',
        executable='p2d_ndt_localizer_exe',
        namespace='localization',
        name='p2d_ndt_localizer_node',
        parameters=[LaunchConfiguration('ndt_localizer_param_file')],
        remappings=[
            ("points_in", "/points_downsampled")
        ]
    )
    mpc = Node(
        package='mpc_controller_nodes',
        executable='mpc_controller_node_exe',
        name='mpc_controller',
        namespace='control',
        parameters=[
            LaunchConfiguration('mpc_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
    )

    # TODO(nikolai.morin): Hack, to be resolved in #626
    odom_bl_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )

    map_map_pcd_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "-0.53249995478", "0", "0", "map", "map_pcd"]
    )

    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([melex_demo_pkg_prefix, '/launch/avp_core.launch.py']),
        launch_arguments={}.items()
    )
    ouster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ouster_pkg_prefix, '/launch/os1_launch.py']),
        launch_arguments={}.items()
    )



    return LaunchDescription([
        with_lidars_param,
        map_publisher_param,
        ndt_localizer_param,
        mpc_param,
        pc_filter_transform_param,
        vehicle_characteristics_param,
        ouster_launch,
        filter_transform_os1_128,
        urdf_publisher,
        map_publisher,
        ndt_localizer,
        mpc,
        odom_bl_publisher,
        map_map_pcd_publisher,
        core_launch
    ])
