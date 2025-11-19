import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import yaml
from os import path



def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')
    
    
    
    
    moveit_config = (
            MoveItConfigsBuilder("avu", package_name="avu_moveit")
            .robot_description(file_path=os.path.join(
                get_package_share_directory("avu_description"),
                "urdf",
                "avu.urdf.xacro"
                )
            )
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .robot_description_semantic(file_path="config/avu.srdf")
            .joint_limits(file_path="config/joint_limits.yaml")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .planning_pipelines(
                pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
                default_planning_pipeline="ompl"
            )
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
            .to_moveit_configs()
        )
    
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    start_move_group_node_cmd = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time},
                {'start_state': {'content': "config/initial_positions.yaml"}},
                move_group_capabilities,
            ],
        )
    
    
    start_rviz_node_cmd = Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {'use_sim_time': use_sim_time}
            ],
        )
    

    


    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            declare_use_rviz_cmd,
            start_move_group_node_cmd,
            start_rviz_node_cmd,
        ]
    )