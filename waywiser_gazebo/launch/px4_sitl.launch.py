import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    gazebo_dir = get_package_share_directory('waywiser_gazebo')

    launch_setup_action = OpaqueFunction(function=launch_setup)

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(gazebo_dir, 'worlds', 'car_world.sdf'),
            description='Full path to the Gazebo world SDF file PX4 should load.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'gazebo_bridge',
            default_value=os.path.join(gazebo_dir, 'config', 'drone_gazebo_bridges.yaml'),
            description='Full path to the model-specific ROS-Gazebo bridge config file.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'default_gazebo_bridge',
            default_value=os.path.join(gazebo_dir, 'config', 'default_gazebo_bridges.yaml'),
            description='Full path to the default ROS-Gazebo bridge config file.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'drone_config',
            default_value=os.path.join(gazebo_dir, 'config', 'drone.yaml'),
            description='Full path to the drone config YAML.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'drone_name',
            default_value='drone',
            description='Existing Gazebo model name PX4 should attach to.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'px4_sys_autostart',
            default_value='4001',
            description='PX4 SYS_AUTOSTART airframe id (4001 is Gazebo x500).',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'px4_start_delay',
            default_value='2.0',
            description='Delay in seconds before starting PX4 after Gazebo starts.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use Gazebo simulation time.',
        )
    )
    ld.add_action(launch_setup_action)
    return ld


def launch_setup(context):
    gazebo_dir = get_package_share_directory('waywiser_gazebo')
    world_path = Path(LaunchConfiguration('world').perform(context)).resolve()
    drone_config_path = Path(LaunchConfiguration('drone_config').perform(context)).resolve()
    px4_dir = resolve_px4_dir(Path(gazebo_dir))
    drone_name = LaunchConfiguration('drone_name').perform(context)
    px4_sys_autostart = LaunchConfiguration('px4_sys_autostart').perform(context)
    px4_start_delay = float(LaunchConfiguration('px4_start_delay').perform(context))
    bridge_install_prefix = get_vendored_harmonic_bridge_install_prefix()
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_value = LaunchConfiguration('use_sim_time').perform(context)
    gz_world_name = read_world_name(world_path)
    gazebo_world_path = (
        create_harmonic_compatible_sdf(world_path) if is_ignition_sdf(world_path) else world_path
    )

    px4_models_dir = px4_dir / 'Tools' / 'simulation' / 'gz' / 'models'
    px4_worlds_dir = px4_dir / 'Tools' / 'simulation' / 'gz' / 'worlds'
    waywiser_description_dir = Path(get_package_share_directory('waywiser_description'))

    enuref = read_enuref(drone_config_path)
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_entries = [entry for entry in existing_gz_resource_path.split(':') if entry]
    gz_resource_entries.extend(
        [
            str(px4_models_dir),
            str(px4_worlds_dir),
            str(world_path.parent),
            str(gazebo_world_path.parent),
            str(waywiser_description_dir / 'sdf'),
            str(waywiser_description_dir.parent),
        ]
    )

    deduped_resource_entries = []
    for entry in gz_resource_entries:
        if entry not in deduped_resource_entries:
            deduped_resource_entries.append(entry)

    px4_build_dir = px4_dir / 'build' / 'px4_sitl_zenoh'
    px4_rootfs_dir = px4_build_dir / 'rootfs'
    px4_binary = px4_build_dir / 'bin' / 'px4'

    if not px4_binary.is_file():
        raise RuntimeError(
            f"PX4 binary not found at '{px4_binary}'. Build waywiser_gazebo first so "
            'PX4 SITL Zenoh artifacts are generated.'
        )

    validate_gazebo_compatibility(px4_binary, px4_dir, bridge_install_prefix)

    refresh_px4_zenoh_runtime_config(px4_rootfs_dir, px4_sys_autostart)

    px4_env = {
        'PX4_GZ_WORLDS': str(world_path.parent),
        'PX4_GZ_WORLD': gz_world_name,
        'PX4_GZ_MODELS': str(px4_models_dir),
        'GZ_SIM_RESOURCE_PATH': ':'.join(deduped_resource_entries),
        'PX4_HOME_LAT': str(enuref[0]),
        'PX4_HOME_LON': str(enuref[1]),
        'PX4_HOME_ALT': str(enuref[2]),
        'PX4_SYS_AUTOSTART': str(px4_sys_autostart),
        'PX4_GZ_STANDALONE': '1',
        'PX4_PARAM_ZENOH_ENABLE': '1',
        'PX4_PARAM_ZENOH_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0'),
        # This launch is intended to run headless with Zenoh offboard control, so
        # PX4 must be allowed to arm without a QGroundControl/MAVLink GCS heartbeat.
        'PX4_PARAM_NAV_DLL_ACT': '0',
        'PX4_PARAM_COM_DLL_EXCEPT': '4',
        'PX4_PARAM_COM_RCL_EXCEPT': '4',
        'PX4_PARAM_COM_ARM_WO_GPS': '1',
        # Waywiser spawns and owns the Gazebo model. The model is not a stock PX4
        # airframe with ESC telemetry or a simulated power module, so disable the
        # checks that would otherwise trigger termination immediately after arming.
        'PX4_PARAM_COM_ARM_CHK_ESCS': '0',
        'PX4_PARAM_FD_ESCS_EN': '0',
        'PX4_PARAM_SYS_FAILURE_EN': '0',
        'PX4_PARAM_CBRK_FLIGHTTERM': '121212',
        'PX4_PARAM_CBRK_SUPPLY_CHK': '894281',
        'PX4_PARAM_COM_DISARM_PRFLT': '-1',
        'PX4_PARAM_COM_DISARM_LAND': '-1',
        'PX4_PARAM_COM_LOW_BAT_ACT': '0',
        # PX4's Gazebo bridge drives the model through the SIM_GZ_EC output
        # group. Keep these explicit so a persisted parameter cache from a
        # non-Gazebo airframe cannot leave the simulated motors unassigned.
        'PX4_PARAM_SIM_GZ_EN': '1',
        'PX4_PARAM_SIM_GZ_EC_FUNC1': '101',
        'PX4_PARAM_SIM_GZ_EC_FUNC2': '102',
        'PX4_PARAM_SIM_GZ_EC_FUNC3': '103',
        'PX4_PARAM_SIM_GZ_EC_FUNC4': '104',
        'PX4_PARAM_SIM_GZ_EC_MIN1': '150',
        'PX4_PARAM_SIM_GZ_EC_MIN2': '150',
        'PX4_PARAM_SIM_GZ_EC_MIN3': '150',
        'PX4_PARAM_SIM_GZ_EC_MIN4': '150',
        'PX4_PARAM_SIM_GZ_EC_MAX1': '1000',
        'PX4_PARAM_SIM_GZ_EC_MAX2': '1000',
        'PX4_PARAM_SIM_GZ_EC_MAX3': '1000',
        'PX4_PARAM_SIM_GZ_EC_MAX4': '1000',
    }

    # Waywiser owns model spawning; PX4 attaches to that existing Gazebo model.
    px4_env['PX4_GZ_MODEL_NAME'] = drone_name

    default_bridge_config = create_runtime_bridge_config(
        LaunchConfiguration('default_gazebo_bridge').perform(context), gz_world_name
    )
    model_bridge_config = create_runtime_bridge_config(
        LaunchConfiguration('gazebo_bridge').perform(context), gz_world_name
    )
    default_bridge_node = create_bridge_action(
        default_bridge_config,
        use_sim_time_value,
        bridge_install_prefix,
    )

    model_bridge_node = create_bridge_action(
        model_bridge_config,
        use_sim_time_value,
        bridge_install_prefix,
    )

    map_frame_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x',
            '0',
            '--y',
            '0',
            '--z',
            '0',
            '--roll',
            '0',
            '--pitch',
            '0',
            '--yaw',
            '0',
            '--frame-id',
            gz_world_name,
            '--child-frame-id',
            'map',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    gazebo_cmd = ['gz', 'sim', '-r', str(gazebo_world_path)]
    gazebo_process = ExecuteProcess(
        cmd=gazebo_cmd,
        output='screen',
        emulate_tty=True,
    )

    px4_sitl_process = ExecuteProcess(
        cmd=[str(px4_binary)],
        cwd=str(px4_rootfs_dir),
        additional_env=px4_env,
        output='screen',
        emulate_tty=True,
    )
    delayed_px4_sitl_process = TimerAction(period=px4_start_delay, actions=[px4_sitl_process])

    gazebo_resource_path_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(deduped_resource_entries),
    )
    ign_resource_path_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=':'.join(deduped_resource_entries),
    )

    actions = [
        gazebo_resource_path_env,
        ign_resource_path_env,
        gazebo_process,
        map_frame_transform,
        default_bridge_node,
        model_bridge_node,
    ]
    actions.append(delayed_px4_sitl_process)
    return actions


def refresh_px4_zenoh_runtime_config(px4_rootfs_dir: Path, px4_sys_autostart: str):
    zenoh_dir = px4_rootfs_dir / 'zenoh'
    for csv_name in ('pub.csv', 'sub.csv'):
        csv_path = zenoh_dir / csv_name
        if csv_path.is_file():
            csv_path.unlink()

    # PX4 processes PX4_PARAM_* env vars (via `param set`) BEFORE the airframe startup
    # script runs. The airframe script for SYS_AUTOSTART 4001 (gz_x500) contains
    # `param set-default NAV_DLL_ACT 2`, which calls user_config.refresh() and resets
    # the value back to 2 even after the env var set it to 0. The rcS script sources
    # `$autostart_file.post` immediately after the airframe script, giving us a reliable
    # hook to re-apply overrides. Write such a .post file for the selected airframe so
    # that NAV_DLL_ACT=0 (no GCS required) is always enforced for headless SITL runs.
    airframes_dir = px4_rootfs_dir / 'etc' / 'init.d-posix' / 'airframes'
    if airframes_dir.is_dir():
        # Find the airframe file matching SYS_AUTOSTART (files are named <id>_<model>)
        matching = list(airframes_dir.glob(f'{px4_sys_autostart}_*'))
        if matching:
            post_file = Path(str(matching[0]) + '.post')
            post_file.write_text(
                '# Auto-generated by px4_sitl.launch.py — do not edit manually.\n'
                '# Re-apply headless SITL parameter overrides after the airframe script,\n'
                '# which may use param set-default to raise NAV_DLL_ACT above 0.\n'
                'param set NAV_DLL_ACT 0\n'
            )


def get_vendored_harmonic_bridge_install_prefix():
    candidates = []

    waywiser_ws = os.environ.get('WAYWISER_WS')
    if waywiser_ws:
        candidates.append(Path(waywiser_ws) / 'install' / 'ros_gz_harmonic')
        candidates.append(
            Path(waywiser_ws)
            / 'src'
            / 'WayWiseR'
            / 'waywiser_gazebo'
            / 'external'
            / 'ros_gz_harmonic'
            / 'install'
        )

    gazebo_dir = Path(get_package_share_directory('waywiser_gazebo')).resolve()
    install_dir = find_waywiser_install_dir(gazebo_dir)
    if install_dir:
        candidates.append(install_dir / 'ros_gz_harmonic')

    candidates.append(gazebo_dir / 'external' / 'ros_gz_harmonic' / 'install')

    source_dir = find_waywiser_source_dir(gazebo_dir)
    if source_dir:
        candidates.append(
            source_dir / 'waywiser_gazebo' / 'external' / 'ros_gz_harmonic' / 'install'
        )

    for bridge_prefix in candidates:
        bridge_executable = bridge_prefix / 'lib' / 'ros_gz_bridge' / 'parameter_bridge'
        if bridge_executable.is_file():
            return str(bridge_prefix)

    return ''


def find_waywiser_install_dir(start_path: Path):
    for parent in [start_path, *start_path.parents]:
        if (parent / 'waywiser_gazebo' / 'share' / 'waywiser_gazebo').is_dir():
            return parent
    return None


def find_waywiser_source_dir(start_path: Path):
    for parent in [start_path, *start_path.parents]:
        candidate = parent / 'waywiser_gazebo' / 'external' / 'ros_gz_harmonic'
        if candidate.is_dir():
            return parent
    return None


def resolve_px4_dir(gazebo_dir: Path):
    candidates = []

    waywiser_ws = os.environ.get('WAYWISER_WS')
    if waywiser_ws:
        candidates.append(
            Path(waywiser_ws)
            / 'src'
            / 'WayWiseR'
            / 'waywiser_gazebo'
            / 'external'
            / 'PX4-Autopilot'
        )

    source_dir = find_waywiser_source_dir(gazebo_dir)
    if source_dir:
        candidates.append(source_dir / 'waywiser_gazebo' / 'external' / 'PX4-Autopilot')

    candidates.append(gazebo_dir / 'external' / 'PX4-Autopilot')

    deduped_candidates = []
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved not in deduped_candidates:
            deduped_candidates.append(resolved)

    for candidate in deduped_candidates:
        if (candidate / 'build' / 'px4_sitl_zenoh' / 'bin' / 'px4').is_file():
            return candidate

    for candidate in deduped_candidates:
        if candidate.is_dir():
            return candidate

    return (gazebo_dir / 'external' / 'PX4-Autopilot').resolve()


def create_bridge_action(config_file, use_sim_time, bridge_install_prefix=''):
    bridge_binary = resolve_bridge_executable(bridge_install_prefix)

    if not bridge_binary:
        return Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=[
                '--ros-args',
                '-p',
                ['config_file:=', config_file],
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )

    bridge_env = {}
    if bridge_install_prefix:
        bridge_env = {
            'LD_LIBRARY_PATH': prepend_env_path(
                os.environ.get('LD_LIBRARY_PATH', ''), str(Path(bridge_install_prefix) / 'lib')
            ),
            'AMENT_PREFIX_PATH': prepend_env_path(
                os.environ.get('AMENT_PREFIX_PATH', ''), bridge_install_prefix
            ),
        }

    return ExecuteProcess(
        cmd=[
            str(bridge_binary),
            '--ros-args',
            '-p',
            f'config_file:={config_file}',
            '-p',
            f'use_sim_time:={use_sim_time}',
        ],
        additional_env=bridge_env,
        output='screen',
    )


def prepend_env_path(current_value, new_value):
    if not new_value:
        return current_value
    entries = [new_value]
    entries.extend([entry for entry in current_value.split(':') if entry and entry != new_value])
    return ':'.join(entries)


def validate_gazebo_compatibility(px4_binary: Path, px4_dir: Path, bridge_install_prefix=''):
    px4_min_gz_version = read_px4_min_gz_version(px4_dir)
    px4_transport = detect_linked_gazebo_transport(px4_binary)
    bridge_transport = detect_ros_gz_bridge_transport(bridge_install_prefix)

    if not px4_min_gz_version or not px4_transport or not bridge_transport:
        return

    px4_requires_harmonic = major_version(px4_min_gz_version) >= 8
    bridge_is_fortress = bridge_transport.startswith('ignition-transport')

    if px4_requires_harmonic and bridge_is_fortress:
        raise RuntimeError(
            f'Gazebo ABI mismatch: PX4 requires Harmonic (MIN_GZ_VERSION={px4_min_gz_version}, '
            f'links {px4_transport}), but ros_gz_bridge links {bridge_transport} '
            '(Fortress/Ignition). '
            'Fix: run `make setup` to build the ros_gz_harmonic submodule.'
        )


def create_harmonic_compatible_sdf(sdf_path: Path):
    try:
        tree = ET.parse(sdf_path)
    except ET.ParseError as exc:
        raise RuntimeError(f"Failed to parse SDF file '{sdf_path}': {exc}") from exc

    root = tree.getroot()
    changed = False
    world = root.find('world')

    for plugin in root.iter('plugin'):
        filename = plugin.get('filename')
        if filename:
            converted_filename = convert_harmonic_plugin_filename(filename)
            if converted_filename != filename:
                plugin.set('filename', converted_filename)
                changed = True

        name = plugin.get('name')
        if name:
            converted_name = convert_harmonic_plugin_name(name)
            if converted_name != name:
                plugin.set('name', converted_name)
                changed = True

    if world is not None:
        changed = (
            add_world_plugin_if_missing(
                world,
                'gz::sim::systems::AirPressure',
                'gz-sim-air-pressure-system',
            )
            or changed
        )
        changed = (
            add_world_plugin_if_missing(
                world,
                'gz::sim::systems::Magnetometer',
                'gz-sim-magnetometer-system',
            )
            or changed
        )

    if not changed:
        return sdf_path

    temp_sdf = tempfile.NamedTemporaryFile(
        mode='wb',
        prefix=f'waywiser_harmonic_{sdf_path.stem}_',
        suffix='.sdf',
        delete=False,
    )
    with temp_sdf:
        tree.write(temp_sdf, encoding='utf-8', xml_declaration=True)

    return Path(temp_sdf.name)


def add_world_plugin_if_missing(world, plugin_name, plugin_filename):
    for plugin in world.findall('plugin'):
        if plugin.get('name') == plugin_name or plugin.get('filename') == plugin_filename:
            return False

    plugin = ET.Element('plugin')
    plugin.set('filename', plugin_filename)
    plugin.set('name', plugin_name)
    world.insert(0, plugin)
    return True


def convert_harmonic_plugin_filename(filename: str):
    if filename.startswith('ignition-gazebo-'):
        return filename.replace('ignition-gazebo-', 'gz-sim-', 1)

    if filename.startswith('libignition-gazebo-') and filename.endswith('.so'):
        bare_filename = filename.removeprefix('lib').removesuffix('.so')
        return bare_filename.replace('ignition-gazebo-', 'gz-sim-', 1)

    return filename


def convert_harmonic_plugin_name(name: str):
    return name.replace('ignition::gazebo::', 'gz::sim::')


def read_px4_min_gz_version(px4_dir: Path):
    gzsim_script = px4_dir / 'ROMFS' / 'px4fmu_common' / 'init.d-posix' / 'px4-rc.gzsim'
    if not gzsim_script.is_file():
        return ''

    match = re.search(r'MIN_GZ_VERSION="([^"]+)"', gzsim_script.read_text(encoding='utf-8'))
    return match.group(1) if match else ''


def detect_linked_gazebo_transport(binary: Path, library_prefix=''):
    output = run_ldd(binary, library_prefix)
    if 'libgz-transport' in output:
        match = re.search(r'lib(gz-transport\d*)\.so', output)
        return match.group(1) if match else 'gz-transport'
    if 'libignition-transport' in output:
        match = re.search(r'lib(ignition-transport\d*)\.so', output)
        return match.group(1) if match else 'ignition-transport'
    return ''


def detect_ros_gz_bridge_transport(bridge_install_prefix=''):
    bridge_binary = resolve_bridge_executable(bridge_install_prefix)
    if not bridge_binary:
        bridge_executable = shutil.which('parameter_bridge')
        bridge_binary = (
            Path(bridge_executable)
            if bridge_executable
            else Path('/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge')
        )
    if not bridge_binary.is_file():
        return ''
    return detect_linked_gazebo_transport(bridge_binary, bridge_install_prefix)


def resolve_bridge_executable(bridge_install_prefix=''):
    if bridge_install_prefix:
        return Path(bridge_install_prefix) / 'lib' / 'ros_gz_bridge' / 'parameter_bridge'
    return None


def run_ldd(binary: Path, library_prefix=''):
    env = os.environ.copy()
    if library_prefix:
        env['LD_LIBRARY_PATH'] = prepend_env_path(
            env.get('LD_LIBRARY_PATH', ''), str(Path(library_prefix) / 'lib')
        )

    try:
        return subprocess.check_output(
            ['ldd', str(binary)],
            text=True,
            stderr=subprocess.STDOUT,
            env=env,
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ''


def major_version(version: str):
    match = re.match(r'(\d+)', version)
    return int(match.group(1)) if match else 0


def create_runtime_bridge_config(bridge_config_path, world_name):
    with open(bridge_config_path, 'r', encoding='utf-8') as bridge_file:
        config = yaml.safe_load(bridge_file) or []

    if not isinstance(config, list):
        raise RuntimeError(f'Gazebo bridge config must be a list: {bridge_config_path}')

    for entry in config:
        gz_topic_name = entry.get('gz_topic_name')
        if isinstance(gz_topic_name, str):
            entry['gz_topic_name'] = gz_topic_name.format(world_name=world_name)

    temp_config = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='waywiser_px4_default_bridge_',
        suffix='.yaml',
        delete=False,
    )
    with temp_config:
        yaml.safe_dump(config, temp_config)

    return temp_config.name


def read_enuref(drone_config_path: Path):
    default_enuref = [57.713805, 12.890088, 203.59]

    if not drone_config_path.is_file():
        return default_enuref

    with drone_config_path.open('r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}

    ros_params = config.get('/**', {}).get('ros__parameters', {})
    enuref = ros_params.get('enuref', default_enuref)
    if isinstance(enuref, list) and len(enuref) >= 3:
        return enuref[:3]
    return default_enuref


def read_world_name(world_path: Path):
    if world_path.is_file():
        try:
            tree = ET.parse(world_path)
            world_element = tree.getroot().find('world')
            if world_element is not None and world_element.get('name'):
                return str(world_element.get('name'))
        except ET.ParseError:
            pass
    return world_path.stem


def is_ignition_sdf(sdf_path: Path):
    try:
        content = sdf_path.read_text(encoding='utf-8')
        return 'ignition-gazebo' in content or 'ignition::gazebo' in content
    except Exception:
        return False


# ── PX4-specific SDF transforms ───────────────────────────────────────────────
# These are applied to drone model SDFs before spawning into Gazebo.
# Called from gazebo_drone_px4.launch.py's spawn pre-processing step.


def create_model_sdf_with_harmonic_plugins(sdf_path):
    """Convert Ignition/Fortress plugin names to Gazebo Harmonic names in a model SDF."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    changed = False

    for plugin in root.iter('plugin'):
        filename = plugin.get('filename')
        if filename:
            converted = filename.replace('ignition-gazebo-', 'gz-sim-')
            if converted != filename:
                plugin.set('filename', converted)
                changed = True
        name = plugin.get('name')
        if name:
            converted = name.replace('ignition::gazebo::', 'gz::sim::')
            if converted != name:
                plugin.set('name', converted)
                changed = True

    if not changed:
        return sdf_path

    temp_sdf = tempfile.NamedTemporaryFile(
        mode='wb', prefix='waywiser_harmonic_model_', suffix='.sdf', delete=False
    )
    with temp_sdf:
        tree.write(temp_sdf, encoding='utf-8', xml_declaration=True)
    return temp_sdf.name


def create_sdf_without_multicopter_velocity_control(sdf_path):
    """Remove the MulticopterVelocityControl plugin from a model SDF (PX4 handles control)."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    changed = False

    for parent in root.iter():
        for plugin in list(parent.findall('plugin')):
            name = plugin.get('name', '')
            filename = plugin.get('filename', '')
            if (
                name.endswith('::MulticopterVelocityControl')
                or 'multicopter-control-system' in filename
            ):
                parent.remove(plugin)
                changed = True

    if not changed:
        return sdf_path

    temp_sdf = tempfile.NamedTemporaryFile(
        mode='wb', prefix='waywiser_px4_spawn_', suffix='.sdf', delete=False
    )
    with temp_sdf:
        tree.write(temp_sdf, encoding='utf-8', xml_declaration=True)
    return temp_sdf.name


def create_sdf_with_px4_sim_sensors(sdf_path):
    """Inject PX4-compatible sensors into a model SDF."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find('.//model')
    if model is None:
        raise RuntimeError(f'Cannot add PX4 sim sensors without a <model> tag in: {sdf_path}')
    base_link = root.find(".//model/link[@name='base_link']")
    if base_link is None:
        raise RuntimeError(f'Cannot add PX4 sim sensors without a base_link in: {sdf_path}')

    existing_sensor_names = {
        sensor.get('name') for sensor in base_link.findall('sensor') if sensor.get('name')
    }
    existing_link_names = {link.get('name') for link in model.findall('link') if link.get('name')}
    existing_joint_names = {
        joint.get('name') for joint in model.findall('joint') if joint.get('name')
    }
    changed = False

    sensor_xml_strings = [
        """
        <sensor name="air_pressure_sensor" type="air_pressure">
          <gz_frame_id>base_link</gz_frame_id>
          <always_on>1</always_on>
          <update_rate>50</update_rate>
          <air_pressure>
            <pressure>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>3</stddev>
              </noise>
            </pressure>
          </air_pressure>
        </sensor>
        """,
        """
        <sensor name="magnetometer_sensor" type="magnetometer">
          <gz_frame_id>base_link</gz_frame_id>
          <always_on>1</always_on>
          <update_rate>100</update_rate>
          <magnetometer>
            <x><noise type="gaussian"><stddev>0.0001</stddev></noise></x>
            <y><noise type="gaussian"><stddev>0.0001</stddev></noise></y>
            <z><noise type="gaussian"><stddev>0.0001</stddev></noise></z>
          </magnetometer>
        </sensor>
        """,
        """
        <sensor name="imu_sensor" type="imu">
          <gz_frame_id>base_link</gz_frame_id>
          <always_on>1</always_on>
          <update_rate>250</update_rate>
          <imu>
            <angular_velocity>
              <x><noise type="gaussian"><mean>0.0</mean><stddev>0.0008726646</stddev></noise></x>
              <y><noise type="gaussian"><mean>0.0</mean><stddev>0.0008726646</stddev></noise></y>
              <z><noise type="gaussian"><mean>0.0</mean><stddev>0.0008726646</stddev></noise></z>
            </angular_velocity>
            <linear_acceleration>
              <x><noise type="gaussian"><mean>0.0</mean><stddev>0.00637</stddev></noise></x>
              <y><noise type="gaussian"><mean>0.0</mean><stddev>0.00637</stddev></noise></y>
              <z><noise type="gaussian"><mean>0.0</mean><stddev>0.00686</stddev></noise></z>
            </linear_acceleration>
          </imu>
        </sensor>
        """,
        """
        <sensor name="navsat_sensor" type="navsat">
          <gz_frame_id>base_link</gz_frame_id>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
        </sensor>
        """,
    ]

    model_xml_strings = [
        (
            'joint',
            'lidar_sensor_joint',
            """
            <joint name="lidar_sensor_joint" type="fixed">
              <parent>base_link</parent>
              <child>lidar_sensor_link</child>
            </joint>
            """,
        ),
        (
            'link',
            'lidar_sensor_link',
            """
            <link name="lidar_sensor_link">
              <pose relative_to="base_link">0 0 -0.05 0 1.57 0</pose>
              <inertial>
                <mass>0.001</mass>
                <inertia>
                  <ixx>0.00001</ixx>
                  <iyy>0.00001</iyy>
                  <izz>0.00001</izz>
                  <ixy>0.0</ixy>
                  <ixz>0.0</ixz>
                  <iyz>0.0</iyz>
                </inertia>
              </inertial>
              <sensor name="lidar" type="gpu_lidar">
                <gz_frame_id>lidar_sensor_link</gz_frame_id>
                <pose>0 0 0 3.14 0 0</pose>
                <update_rate>50</update_rate>
                <ray>
                  <scan>
                    <horizontal>
                      <samples>1</samples>
                      <resolution>1</resolution>
                      <min_angle>0</min_angle>
                      <max_angle>0</max_angle>
                    </horizontal>
                    <vertical>
                      <samples>1</samples>
                      <resolution>1</resolution>
                      <min_angle>0</min_angle>
                      <max_angle>0</max_angle>
                    </vertical>
                  </scan>
                  <range>
                    <min>0.1</min>
                    <max>100.0</max>
                    <resolution>0.01</resolution>
                  </range>
                </ray>
                <always_on>1</always_on>
                <visualize>false</visualize>
              </sensor>
            </link>
            """,
        ),
    ]

    for sensor_xml in sensor_xml_strings:
        sensor = ET.fromstring(sensor_xml)
        if sensor.get('name') not in existing_sensor_names:
            base_link.append(sensor)
            changed = True

    for element_type, element_name, element_xml in model_xml_strings:
        if element_type == 'joint' and element_name in existing_joint_names:
            continue
        if element_type == 'link' and element_name in existing_link_names:
            continue
        model.append(ET.fromstring(element_xml))
        changed = True

    if not changed:
        return sdf_path

    temp_sdf = tempfile.NamedTemporaryFile(
        mode='wb', prefix='waywiser_px4_sensors_', suffix='.sdf', delete=False
    )
    with temp_sdf:
        tree.write(temp_sdf, encoding='utf-8', xml_declaration=True)
    return temp_sdf.name


def create_sdf_with_px4_motor_joint_names(sdf_path):
    # No longer removing prefixes to maintain consistency with prefixed URDF joints.
    return sdf_path
