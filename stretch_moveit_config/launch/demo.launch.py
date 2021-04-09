import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro
import argparse
import sys

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    parser = argparse.ArgumentParser()
    parser.add_argument("--use_fake_controller", default=False, type=eval, choices=[True, False])
    args, _ = parser.parse_known_args([arg for sys_arg in sys.argv[4:] for arg in ('--' + sys_arg).split(':=')])

    ld = LaunchDescription()
    # planning_context
    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory('stretch_moveit_config'),
                                                               'config',
                                                               'stretch.xacro'),
                                                  mappings={"use_fake_controller": str(args.use_fake_controller)})
    robot_description = {'robot_description' : robot_description_config.toxml()}

    robot_description_semantic_config = load_file('stretch_moveit_config', 'config/stretch_description.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml('stretch_moveit_config', 'config/kinematics.yaml')

    # Planning Functionality
    ompl_planning_pipeline_config = { 'move_group' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }
    ompl_planning_yaml = load_yaml('stretch_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    controllers_yaml = load_yaml('stretch_moveit_config', 'config/moveit_simple_controllers.yaml')
    moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml,
                           'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.01}

    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                                         "publish_geometry_updates": True,
                                         "publish_state_updates": True,
                                         "publish_transforms_updates": True}

    # Start the actual move_group node/action server
    run_move_group_node = Node(package='moveit_ros_move_group',
                               executable='move_group',
                               output='screen',
                               parameters=[robot_description,
                                           robot_description_semantic,
                                           kinematics_yaml,
                                           ompl_planning_pipeline_config,
                                           trajectory_execution,
                                           moveit_controllers,
                                           planning_scene_monitor_parameters])
    ld.add_action(run_move_group_node)

    # RViz
    rviz_config_file = get_package_share_directory('stretch_moveit_config') + "/launch/moveit.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic,
                                 ompl_planning_pipeline_config,
                                 kinematics_yaml])
    ld.add_action(rviz_node)

    if args.use_fake_controller:
        # Publish TF
        robot_state_publisher = Node(package='robot_state_publisher',
                                     executable='robot_state_publisher',
                                     name='robot_state_publisher',
                                     output='both',
                                     parameters=[robot_description])
        ld.add_action(robot_state_publisher)

        # Fake joint driver
        fake_joint_driver_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description,  os.path.join(get_package_share_directory("stretch_moveit_config"), "config", "ros_controllers.yaml")],
        )
        ld.add_action(fake_joint_driver_node)

        for controller in ["stretch_arm_controller", "gripper_controller", "joint_state_controller"]:
            ld.add_action(
                ExecuteProcess(
                    cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                    shell=True,
                    output="screen",
                )
            )

    return ld
