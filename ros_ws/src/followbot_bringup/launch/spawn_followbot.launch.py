"""1 台の FollowBot を Gazebo へ spawn し、ros2_control を起動する (spec v2.0 §9-§12)。

Leader / Follower の双方からこの launch を include して使う。

実装上の注意 (DEVLOG L-1):
    引数は OpaqueFunction 内で perform() して **素の文字列**へ確定させてから Node を作る。
    event handler (OnProcessExit) の中で LaunchConfiguration を評価すると、include が
    作ったスコープではなく発火時のコンテキストで解決されてしまい、2 台目の spawner が
    1 台目の namespace を叩く。

実装上の注意 (DEVLOG L-2):
    URDF の link 名は xacro の prefix 引数で既に "<ns>/" 付きになっている。
    robot_state_publisher に frame_prefix を与えると prefix が二重になるため与えない。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from followbot_bringup.launch_args import parse_bool

ARGS = [
    DeclareLaunchArgument("name", description="Gazebo 上のモデル名 (例: leader)"),
    DeclareLaunchArgument("namespace", description="ROS namespace (例: leader)"),
    DeclareLaunchArgument("color", default_value="0.1 0.2 1.0 1.0",
                          description="車体色 RGBA"),
    DeclareLaunchArgument("x", default_value="0.0"),
    DeclareLaunchArgument("y", default_value="0.0"),
    DeclareLaunchArgument("yaw", default_value="0.0"),
    DeclareLaunchArgument("use_sim_time", default_value="true"),
]


def _build(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)

    name = arg("name")
    ns = arg("namespace")
    color = arg("color")
    use_sim_time = parse_bool(arg("use_sim_time"), "use_sim_time")

    description_pkg = FindPackageShare("followbot_description").perform(context)
    xacro_file = f"{description_pkg}/urdf/followbot.urdf.xacro"
    controllers_file = f"{description_pkg}/config/followbot_controllers.yaml"

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            f" prefix:={ns}/",
            f" namespace:={ns}",
            f" color:='{color}'",
            f" controllers_file:={controllers_file}",
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{ns}",
        output="screen",
        arguments=[
            "-name", name,
            "-topic", f"/{ns}/robot_description",
            "-x", arg("x"),
            "-y", arg("y"),
            "-z", "0.02",
            "-Y", arg("yaw"),
        ],
    )

    def spawner(controller, extra_args=()):
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace=ns,
            name=f"spawner_{ns}_{controller}",
            output="screen",
            arguments=[controller,
                       "--controller-manager", "controller_manager",
                       "--controller-manager-timeout", "120",
                       *extra_args],
        )

    jsb_spawner = spawner("joint_state_broadcaster")

    # spec §11 が要求する /<ns>/cmd_vel, /<ns>/odom へリマップする
    # (既定では /<ns>/diff_drive_controller/cmd_vel, .../odom)
    diff_drive_spawner = spawner("diff_drive_controller", [
        "--controller-ros-args",
        f"-r /{ns}/diff_drive_controller/cmd_vel:=/{ns}/cmd_vel",
        "--controller-ros-args",
        f"-r /{ns}/diff_drive_controller/odom:=/{ns}/odom",
    ])

    # spawn 完了後に controller を順に起動する (spec §53 の依存順)
    return [
        robot_state_publisher,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[jsb_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner,
                                           on_exit=[diff_drive_spawner])),
    ]


def generate_launch_description():
    return LaunchDescription(ARGS + [OpaqueFunction(function=_build)])
