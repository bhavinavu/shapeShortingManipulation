import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_description = get_package_share_directory("avu_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        robot_description, "urdf", "avu.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(robot_description).parent.resolve())
            ]
        )
    
    red_cube_path = os.path.join(
        get_package_share_directory('avu_description'),
        'urdf', 'red_cube.sdf'
    )

    blue_cube_path = os.path.join(
        get_package_share_directory('avu_description'),
        'urdf', 'blue_cube.sdf'
    )

    red_sphere_path = os.path.join(
        get_package_share_directory('avu_description'),
        'urdf', 'red_sphere.sdf'
    )

    blue_sphere_path = os.path.join(
        get_package_share_directory('avu_description'),
        'urdf', 'blue_sphere.sdf'
    )

    cup_path = os.path.join(
        get_package_share_directory('avu_description'),
        'urdf', 'cup.sdf'
    )
    

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4 -r empty.sdf "]
                    )
                ]
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "avu"],
    )

    #red_cube_spawn = ExecuteProcess(
    #    cmd=['ros2', 'run', 'ros_gz_sim', 'create',
    # #        '-file', red_cube_path,
    #         '-name', 'red_cube', '-x', '0.160', '-y', '0.131', '-z', '0.1'],
    #    output='screen'
    #)

    blue_cube_spawn = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-file', blue_cube_path,
             '-name', 'blue_cube', '-x', '-0.163', '-y', '-0.127', '-z', '0.1'],
        output='screen'
    )

    red_sphere_spawn = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-file', red_sphere_path,
             '-name', 'red_sphere', '-x', '0.128', '-y', '0.072', '-z', '0.1'],
        output='screen'
    )

    blue_sphere_spawn = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-file', blue_sphere_path,
             '-name', 'blue_sphere', '-x', '-0.136', '-y', '0.169', '-z', '0.1'],
        output='screen'
    )

    cup_spawn = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-file', cup_path,
             '-name', 'cup', '-x', '0.123', '-y', '-0.131', '-z', '0.0'],
        output='screen'
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera_wrist/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_wrist/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera_front/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_front/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera_top/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_top/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera_top/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
        ]
    )



    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        #red_cube_spawn,
        blue_cube_spawn,
        red_sphere_spawn,
        blue_sphere_spawn,
        cup_spawn,
        gz_ros2_bridge
    ])