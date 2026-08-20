"""Launch the complete click-plan-start-execute chain in one command."""

import os

from ament_index_python.packages import get_package_share_directory
from climbot_description.wall_frame import reference_grid_spacing
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Bring up simulation, planner, RViz, tracker and manager together."""
    gazebo_share = get_package_share_directory('climbot_gazebo')
    coverage_share = get_package_share_directory('climbot_coverage')
    control_share = get_package_share_directory('climbot_control')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'climbot_wall.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'headless': LaunchConfiguration('headless'),
            'gpu_backend': LaunchConfiguration('gpu_backend'),
            'wall_grid_spacing': LaunchConfiguration('wall_grid_spacing'),
        }.items(),
    )
    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                coverage_share, 'launch', 'coverage_planner.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'rviz': LaunchConfiguration('rviz'),
            'config_file': LaunchConfiguration('planner_config_file'),
            'input_mode': LaunchConfiguration('input_mode'),
            'region_type': LaunchConfiguration('region_type'),
            'sweep_direction': LaunchConfiguration('sweep_direction'),
            'wall_grid_spacing': LaunchConfiguration('wall_grid_spacing'),
        }.items(),
    )
    # The executor is what coverage_sim.launch.py leaves out: without it the
    # planned task is previewed but nothing can ever run it. Both parameter
    # files are passed explicitly and carry distinct argument names, because
    # one shared config_file would reach whichever include is expanded and
    # leave the other node running on built-in defaults.
    execution = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                control_share, 'launch', 'coverage_executor.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'control_config_file': LaunchConfiguration('control_config_file'),
            'tracking_mode': LaunchConfiguration('tracking_mode'),
        }.items(),
    )

    # This launch runs the RViz click workflow by default, where the shape is
    # a runtime choice, so it needs a config whose task_id does not name one.
    default_planner_config = os.path.join(
        coverage_share, 'config', 'coverage_interactive.yaml')
    default_control_config = os.path.join(
        control_share, 'config', 'control.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without its GUI; RViz still opens.',
        ),
        DeclareLaunchArgument(
            'gpu_backend',
            default_value='auto',
            description='Rendering backend: auto, wsl_d3d12, or native.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz, which provides the Publish Point tool.',
        ),
        DeclareLaunchArgument(
            'planner_config_file',
            default_value=default_planner_config,
            description='Planner parameter file; region corners are ignored '
                        'in rviz input mode.',
        ),
        DeclareLaunchArgument(
            'control_config_file',
            default_value=default_control_config,
            description='Line-tracker parameter file.',
        ),
        DeclareLaunchArgument(
            'input_mode',
            default_value='rviz',
            description='Region input: rviz clicks or configured parameters.',
        ),
        DeclareLaunchArgument('region_type', default_value='rectangle'),
        DeclareLaunchArgument('sweep_direction', default_value='horizontal'),
        # One word for both views. The wall launch paints the grid on the
        # wall face, the planner draws the same lines in RViz, and both take
        # their default from climbot_description/config/wall.yaml. Set it to 0
        # for a run that photographs the wall; untick the RViz display to hide
        # the overlay live without restarting anything.
        DeclareLaunchArgument(
            'wall_grid_spacing',
            default_value=repr(reference_grid_spacing()),
            description='Reference grid pitch in metres; 0 draws none.',
        ),
        DeclareLaunchArgument(
            'tracking_mode',
            default_value='time',
            description='Straight-line control: distance or time. The panel '
                        'can also switch it while no task is running.',
        ),
        simulation,
        planning,
        execution,
    ])
