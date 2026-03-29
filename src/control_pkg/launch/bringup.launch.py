from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro

def generate_launch_description():

    descrip_path = os.path.join(get_package_share_directory('descrip_pino'))
    mujoco_xml_path = os.path.join(descrip_path, 'urdf', 'Pino_robot.xml')
    control_pkg_share = os.path.join(get_package_share_directory('control_pkg'))
    controller_config_file = os.path.join(control_pkg_share, 'controller_config', 'config.yaml')
    declared_arguments = []
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("control_pkg"), "rviz_config", "rviz.rviz"]
    )


    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "robot_controller",
    #         default_value="pos_controller",
    #     )
    # )

    xacro_file = os.path.join(descrip_path,'urdf','total.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    # robot_description = {'robot_description': robot_description_xml}

    ros2_control_params = [
        robot_description,
        controller_config_file,
        {'mujoco_model_path': os.path.join(descrip_path, 'urdf', 'Pino_robot.xml')}, # 明确指定 MuJoCo 文件路径
    ]


    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path':mujoco_xml_path}
        ]
    )

    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_control_params],
        output="screen",
        # remappings=[
        #     ("~/robot_description", "/robot_description"),
        # ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pos_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # start_node = 

    nodes = [
        # control_node,
        # # node_mujoco_ros2_control,
        # robot_state_pub_node,
        # rviz_node,
        # joint_state_broadcaster_spawner,
        # pos_controller_spawner,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[pos_controller_spawner],
            )
        ),
        node_mujoco_ros2_control,   
        robot_state_pub_node,
    ]

    return LaunchDescription(nodes)
