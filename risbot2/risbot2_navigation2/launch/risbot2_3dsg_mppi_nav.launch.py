#!/usr/bin/env python3
"""
RISBOT2 3D Scene Graph + MPPI Local Planner Navigation Launch File
==================================================================
Launches the full navigation stack for RISBOT2 using 3D Scene Graph + AMCL + MPPI Local Planner:

  Step 1: dsg_mesh_to_2d_map_node  → Extracts 2D horizontal mesh slice from Hydra DSG → publishes /map
  Step 2: nav2_amcl & lifecycle   → Dynamic AMCL localization (map -> odom TF) using 2D slice map & /scan
  Step 3: heracles_publisher_node → Reads 3DSG from Neo4j/file → publishes /hydra/backend/dsg
  Step 4: omniplanner_node        → 3D Global Planner (GotoPoints plugin)
  Step 5: Nav2 Controller Server  → MPPI Local Planner (Predictive control & trajectory sampling)
  Step 6: omniplanner_nav2_bridge → Forwards 3D Omniplanner Path to Nav2 FollowPath action
  Step 7: Hydra Visualizer & RViz → Visualizes 3DSG, 2D slice map, MPPI trajectories, and robot navigation

Prerequisites:
  1. Package ros-jazzy-nav2-mppi-controller installed (sudo apt install ros-jazzy-nav2-mppi-controller).
  2. Hydra has created dsg_with_mesh.json.
  3. Robot bringup (risbot2_bringup/robot_wzed.launch.py) is running to provide odom -> base_link TF and /scan.

Usage:
    ros2 launch risbot2_navigation2 risbot2_3dsg_mppi_nav.launch.py use_amcl:=true z_cut:=0.5
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ===== Launch Configurations =====
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_name = LaunchConfiguration('robot_name', default='risbot2')
    use_amcl = LaunchConfiguration('use_amcl', default='true')
    publish_static_tf = LaunchConfiguration('publish_static_tf', default='false')
    z_cut = LaunchConfiguration('z_cut', default='0.5')
    launch_hydra_visualizer = LaunchConfiguration('launch_hydra_visualizer', default='true')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')

    # Heracles / Neo4j connection parameters
    heracles_ip = LaunchConfiguration(
        'heracles_ip',
        default=os.environ.get('HERACLES_IP', 'localhost'))

    heracles_port = LaunchConfiguration(
        'heracles_port',
        default=os.environ.get('HERACLES_PORT', '7687'))

    heracles_user = LaunchConfiguration(
        'heracles_user',
        default=os.environ.get('HERACLES_NEO4J_USERNAME', 'neo4j'))

    heracles_pass = LaunchConfiguration(
        'heracles_pass',
        default=os.environ.get('HERACLES_NEO4J_PASSWORD', 'neo4j_pw'))

    # Config paths
    omniplanner_config = LaunchConfiguration(
        'omniplanner_config',
        default=os.path.join(
            get_package_share_directory('risbot2_navigation2'),
            'param',
            'risbot2_omniplanner.yaml'))

    dsg_filepath = LaunchConfiguration(
        'dsg_filepath',
        default='/home/tht/hydra_ws/src/planner/heracles/heracles/examples/scene_graphs/backend/dsg_with_mesh.json')

    # Default to MPPI config YAML
    nav2_config = LaunchConfiguration(
        'nav2_config',
        default=os.path.join(
            get_package_share_directory('risbot2_navigation2'),
            'param',
            'risbot2_mppi.yaml'))

    ekf_config = LaunchConfiguration(
        'ekf_config',
        default=os.path.join(
            get_package_share_directory('risbot2_navigation2'),
            'param',
            'ekf_zed.yaml'))

    return LaunchDescription([
        # ===== Declare Arguments =====
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),

        DeclareLaunchArgument(
            'robot_name', default_value='risbot2',
            description='Robot name matching Omniplanner config'),

        DeclareLaunchArgument(
            'use_amcl', default_value='true',
            description='Enable AMCL dynamic localization (map -> odom TF) using Hydra 2D mesh slice'),

        DeclareLaunchArgument(
            'publish_static_tf', default_value='false',
            description='Publish static odom->base_link TF (for offline testing without robot)'),

        DeclareLaunchArgument(
            'z_cut', default_value='0.5',
            description='Height Z (meters) to slice Hydra 3D mesh into 2D Occupancy Grid map'),

        DeclareLaunchArgument(
            'launch_hydra_visualizer', default_value='true',
            description='Launch Hydra visualizer node'),

        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Launch RViz2 with Heracles configuration'),

        DeclareLaunchArgument(
            'heracles_ip', default_value=os.environ.get('HERACLES_IP', 'localhost'),
            description='Neo4j server IP address'),

        DeclareLaunchArgument(
            'heracles_port', default_value=os.environ.get('HERACLES_PORT', '7687'),
            description='Neo4j bolt port'),

        DeclareLaunchArgument(
            'heracles_user', default_value=os.environ.get('HERACLES_NEO4J_USERNAME', 'neo4j'),
            description='Neo4j username'),

        DeclareLaunchArgument(
            'heracles_pass', default_value=os.environ.get('HERACLES_NEO4J_PASSWORD', 'neo4j_pw'),
            description='Neo4j password'),

        DeclareLaunchArgument(
            'dsg_filepath',
            default_value='/home/tht/hydra_ws/src/planner/heracles/heracles/examples/scene_graphs/backend/dsg_with_mesh.json',
            description='Fallback path to 3DSG JSON file'),

        DeclareLaunchArgument(
            'omniplanner_config',
            default_value=os.path.join(
                get_package_share_directory('risbot2_navigation2'),
                'param', 'risbot2_omniplanner.yaml'),
            description='Path to Omniplanner plugin config YAML'),

        DeclareLaunchArgument(
            'nav2_config',
            default_value=os.path.join(
                get_package_share_directory('risbot2_navigation2'),
                'param', 'risbot2_mppi.yaml'),
            description='Path to Nav2 MPPI controller & AMCL config YAML'),

        DeclareLaunchArgument(
            'ekf_config',
            default_value=os.path.join(
                get_package_share_directory('risbot2_navigation2'),
                'param', 'ekf_zed.yaml'),
            description='Path to EKF config YAML for ZED2 visual odometry'),

        # ===========================================================
        # Step 0: EKF Node (Dynamic odom -> base_footprint TF)
        # Fuses ZED2 visual odometry to provide dynamic odometry TF.
        # Without this, AMCL sees the robot as stationary and never updates.
        # ===========================================================
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),

        # ===========================================================
        # Step 1: Hydra 2D Mesh Slice Map Generator Node
        # ===========================================================
        Node(
            package='risbot2_node',
            executable='dsg_mesh_to_2d_map_node.py',
            name='dsg_mesh_to_2d_map_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'dsg_filepath': dsg_filepath,
                'z_cut': 0.5,
                'resolution': 0.05,
                'padding': 1.0,
                'frame_id': 'map',
            }],
        ),

        # ===========================================================
        # Step 2: AMCL Localization Node (Dynamic map -> odom TF)
        # ===========================================================
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            condition=IfCondition(use_amcl),
            parameters=[nav2_config],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            condition=IfCondition(use_amcl),
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['amcl'],
            }],
        ),

        # Static TF: map -> odom (ONLY used when AMCL is disabled for offline testing)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            condition=UnlessCondition(use_amcl),
        ),

        # Static TF: odom -> base_link (only for offline testing when no robot odometry)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom_baselink',
            arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
            condition=IfCondition(publish_static_tf),
        ),

        # ===========================================================
        # Step 3: Visualization (Hydra Visualizer & RViz2)
        # ===========================================================
        Node(
            package='hydra_visualizer',
            executable='hydra_visualizer_node',
            name='hydra_visualizer',
            output='screen',
            condition=IfCondition(launch_hydra_visualizer),
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('~/dsg', '/hydra/backend/dsg'),
            ],
            arguments=[
                '--config-utilities-file',
                PathJoinSubstitution([
                    FindPackageShare('heracles_ros'),
                    'config',
                    'hydra_visualizer_config.yaml'
                ]),

                '--config-utilities-file',
                PathJoinSubstitution([
                    FindPackageShare('hydra_visualizer'),
                    'config',
                    'visualizer_plugins.yaml'
                ]),

                '--config-utilities-file',
                PathJoinSubstitution([
                    FindPackageShare('hydra_visualizer'),
                    'config',
                    'external_plugins.yaml'
                ]),

                '--config-utilities-yaml',
                '{glog_level: 1, glog_verbosity: 0}',
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            output='screen',
            condition=IfCondition(launch_rviz),
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('heracles_ros'),
                    'rviz',
                    'example.rviz'
                ]),
            ],
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),

        # ===========================================================
        # Step 4: Heracles Publisher Node
        # ===========================================================
        Node(
            package='heracles_ros',
            executable='heracles_publisher_node',
            name='heracles_publisher_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'heracles_ip': heracles_ip,
                'heracles_port': heracles_port,
                'heracles_neo4j_user': heracles_user,
                'heracles_neo4j_pass': heracles_pass,
                'object_labelspace': 'ade20k_mit_label_space.yaml',
                'region_labelspace': 'b45_label_space.yaml',
                'dsg_filepath': dsg_filepath,
            }],
            remappings=[
                ('~/dsg_out', '/hydra/backend/dsg'),
            ],
        ),

        # ===========================================================
        # Step 5: Omniplanner Node (3D Global Planner)
        # ===========================================================
        Node(
            package='omniplanner_ros',
            executable='omniplanner_node',
            name='omniplanner_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'plugin_config_path': omniplanner_config,
            }],
            remappings=[
                ('~/dsg_in', '/hydra/backend/dsg'),
            ],
        ),

        # ===========================================================
        # Step 6: Nav2 Controller Server with MPPI Local Planner
        # ===========================================================
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_config, {'use_sim_time': use_sim_time}],
        ),

        # Nav2 Lifecycle Manager for Navigation Controllers
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['controller_server'],
            }],
        ),

        # Bridge: Omniplanner Path → Nav2 FollowPath action
        Node(
            package='risbot2_node',
            executable='omniplanner_nav2_bridge.py',
            name='omniplanner_nav2_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_name': robot_name,
            }],
        ),
    ])
