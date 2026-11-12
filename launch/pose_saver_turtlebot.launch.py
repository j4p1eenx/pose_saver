import json
import math
import os
import urllib.request

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _parse_record(rec):
    try:
        x, y = float(rec['x']), float(rec['y'])
        yaw = _yaw_from_quat(float(rec.get('qx', 0.0)), float(rec.get('qy', 0.0)),
                             float(rec.get('qz', 0.0)), float(rec.get('qw', 1.0)))
        stamp = float(rec.get('stamp_unix', 0.0))
        if all(math.isfinite(v) for v in (x, y, yaw, stamp)):
            return x, y, yaw, stamp
    except (KeyError, TypeError, ValueError, AttributeError):
        pass
    return None


def get_last_saved_pose(params_path):
    """Newest of cloud record and local cache; same /poses/<robot_id> layout as the node."""
    cfg = {}
    try:
        with open(params_path) as f:
            cfg = (yaml.safe_load(f) or {}).get('pose_saver_node', {}).get('ros__parameters', {})
    except Exception as e:
        print(f'[launch] could not read {params_path}: {e}')
    robot_id = cfg.get('robot_id', 'turtlebot3_burger_01')
    db_url = os.getenv('POSE_DB_URL') or cfg.get('db_url', '')
    token = os.getenv('POSE_DB_TOKEN') or cfg.get('auth_token', '')
    cache_dir = cfg.get('cache_dir') or os.path.join(os.path.expanduser('~'), '.ros', 'pose_saver')

    found = []  # (x, y, yaw, stamp, source)
    if db_url:
        url = f"{db_url.rstrip('/')}/poses/{robot_id}.json" + (f'?auth={token}' if token else '')
        try:
            with urllib.request.urlopen(url, timeout=2.5) as resp:
                parsed = _parse_record(json.loads(resp.read().decode()))
                if parsed:
                    found.append(parsed + ('cloud',))
        except Exception as e:
            print(f'[launch] cloud pose fetch failed: {e}')
    try:
        with open(os.path.join(cache_dir, f'{robot_id}.json')) as f:
            parsed = _parse_record(json.load(f))
            if parsed:
                found.append(parsed + ('local cache',))
    except Exception:
        pass

    if not found:
        print('[launch] no saved pose found, spawning at the default (-2.0, -0.5, yaw 0)')
        return '-2.0', '-0.5', '0.0'
    x, y, yaw, _, src = max(found, key=lambda c: c[3])
    print(f'[launch] spawning at saved pose from {src}: x={x:.3f} y={y:.3f} yaw={yaw:.3f}')
    return f'{x:.3f}', f'{y:.3f}', f'{yaw:.3f}'


def generate_launch_description():
    pkg = get_package_share_directory('pose_saver_pkg')
    tb3_gz = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    nav2 = get_package_share_directory('nav2_bringup')
    tb3_nav2 = get_package_share_directory('turtlebot3_navigation2')

    model = 'burger'
    model_sdf = os.path.join(tb3_gz, 'models', f'turtlebot3_{model}', 'model.sdf')
    bridge_params = os.path.join(tb3_gz, 'params', f'turtlebot3_{model}_bridge.yaml')
    world = os.path.join(tb3_gz, 'worlds', 'turtlebot3_world.world')
    gz_launch = os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params = os.path.join(nav2, 'params', 'nav2_params.yaml')
    rviz_cfg = os.path.join(nav2, 'rviz', 'nav2_default_view.rviz')

    default_x, default_y, default_yaw = get_last_saved_pose(
        os.path.join(pkg, 'config', 'params.yaml'))

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', model),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3_gz, 'models')),

        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(tb3_nav2, 'map', 'map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'params.yaml')),
        DeclareLaunchArgument('navigation', default_value='false',
                              description='Also start the Nav2 navigation stack'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value=default_x,
                              description='Gazebo spawn x (default: last saved pose)'),
        DeclareLaunchArgument('y_pose', default_value=default_y,
                              description='Gazebo spawn y (default: last saved pose)'),
        DeclareLaunchArgument('yaw_pose', default_value=default_yaw,
                              description='Gazebo spawn yaw in rad (default: last saved pose)'),

        # --- Gazebo server + GUI (same as turtlebot3_world.launch.py) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': f'-r -s -v2 {world}', 'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gz, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        # --- Spawner with yaw (-Y), which the stock spawn launch file does not offer ---
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', model, '-file', model_sdf,
                        '-x', LaunchConfiguration('x_pose'),
                        '-y', LaunchConfiguration('y_pose'),
                        '-z', '0.01',
                        '-Y', LaunchConfiguration('yaw_pose')]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['--ros-args', '-p', f'config_file:={bridge_params}']),

        # --- Nav2 localization (+ optional navigation) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'localization_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'map': LaunchConfiguration('map'),
                              'params_file': nav2_params,
                              'autostart': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params_file': nav2_params,
                              'autostart': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('navigation'))),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', rviz_cfg, '--ros-args', '--log-level', 'warn'],
             parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),

        Node(package='pose_saver_pkg', executable='pose_saver_node',
             name='pose_saver_node', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'use_sim_time': use_sim_time}]),
    ])
