# Copyright 2023 Husarion
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all topics and tfs",
    )

    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description=(
            "Whether to use mecanum drive controller (otherwise diff drive controller is used)"
        ),
    )

    use_multirobot_system = LaunchConfiguration("use_multirobot_system")
    declare_use_multirobot_system_arg = DeclareLaunchArgument(
        "use_multirobot_system",
        default_value="false",
        description="Enable correct Ignition Gazebo configuration in URDF",
    )

    use_gpu = LaunchConfiguration("use_gpu")
    declare_use_gpu_arg = DeclareLaunchArgument(
        "use_gpu",
        default_value="True",
        description="Whether GPU acceleration is used",
    )

    robot_name = PythonExpression(
        ["'rosbot'", " if '", namespace, "' == '' ", "else ", "'", namespace, "'"]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            robot_name,
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x", default="0.00"),
            "-y",
            LaunchConfiguration("y", default="0.00"),
            "-z",
            LaunchConfiguration("z", default="0.00"),
            "-R",
            LaunchConfiguration("roll", default="0.00"),
            "-P",
            LaunchConfiguration("pitch", default="0.00"),
            "-Y",
            LaunchConfiguration("yaw", default="0.00"),
        ],
        output="screen",
        namespace=namespace,
    )

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            # an IR sensor is not implemented yet https://github.com/gazebosim/gz-sensors/issues/19
            "/range/fl@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/range/fr@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/range/rl@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/range/rr@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        ],
        remappings=[
            ("/camera/camera_info", "camera/color/camera_info"),
            ("/camera/image", "camera/color/image_raw"),
            ("/camera/depth_image", "camera/depth/image_raw"),
            ("/camera/points", "camera/depth/points"),
            ("/scan", "scan"),
            ("/range/fl", "range/fl"),
            ("/range/fr", "range/fr"),
            ("/range/rl", "range/rl"),
            ("/range/rr", "range/rr"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        output="screen",
        namespace=namespace,
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_bringup"),
                    "launch",
                    "bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "mecanum": mecanum,
            "use_sim": "True",
            "use_gpu": use_gpu,
            "simulation_engine": "ignition-gazebo",
            "namespace": namespace,
            "use_multirobot_system": use_multirobot_system,
        }.items(),
    )

    # The frame of the pointcloud from ignition gazebo 6 isn't provided by <frame_id>.
    # See https://github.com/gazebosim/gz-sensors/issues/239
    depth_cam_frame_fixer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depth_to_camera",
        output="log",
        arguments=[
            "0.0",
            "0.0",
            "0.0",
            "1.57",
            "-1.57",
            "0.0",
            # Here should be namespace
            "camera_depth_optical_frame",
            "rosbot/base_link/camera_orbbec_astra_camera",
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        namespace=namespace,
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_mecanum_arg,
            declare_use_multirobot_system_arg,
            declare_use_gpu_arg,
            # Sets use_sim_time for all nodes started below
            # (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            ign_bridge,
            gz_spawn_entity,
            bringup_launch,
            depth_cam_frame_fixer,
        ]
    )
