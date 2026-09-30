"""Launch Insta360 Link, discovering its capture node if the udev alias is absent."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_device(context):
    requested = LaunchConfiguration('device').perform(context)
    if requested != 'auto':
        if not Path(requested).exists():
            raise RuntimeError(f'Camera device does not exist: {requested}')
        return requested
    alias = Path('/dev/video-insta360')
    if alias.exists():
        return str(alias)
    candidates = []
    for entry in sorted(Path('/sys/class/video4linux').glob('video*')):
        try:
            name = (entry / 'name').read_text().strip()
            index = (entry / 'index').read_text().strip()
        except OSError:
            continue
        device = Path('/dev') / entry.name
        if name.startswith('Insta360 Link') and index == '0' and device.exists():
            candidates.append(str(device))
    if len(candidates) != 1:
        raise RuntimeError('Expected one Insta360 Link capture node (index 0); found: '
                           + (', '.join(candidates) if candidates else 'none'))
    return candidates[0]


def launch_camera(context):
    share = Path(get_package_share_directory('insta360_usb_cam'))
    config = share / 'config/camera_config.yaml'
    info = share / 'config/camera_info_config.yaml'
    params = yaml.safe_load(config.read_text())['/**']['ros__parameters']
    name = f"pan_tilt_camera_node_{params['camera_name']}"
    device = resolve_device(context)
    return [Node(
        package='insta360_usb_cam', executable='pan_tilt_camera_node',
        name=name, output='screen', parameters=[str(config), str(info), {'device': device}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='auto',
                              description='auto uses /dev/video-insta360 or finds the Insta360 Link index-0 device'),
        OpaqueFunction(function=launch_camera),
    ])
