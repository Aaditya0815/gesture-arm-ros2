from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os
import glob

def generate_launch_description():

    # ── Auto-detect ROS 2 distro (works on any machine) ──────────────────────
    # Search for the ur.urdf.xacro file across all installed ROS 2 distros
    xacro_candidates = glob.glob(
        '/opt/ros/*/share/ur_description/urdf/ur.urdf.xacro'
    )

    if not xacro_candidates:
        raise FileNotFoundError(
            "\n\n[ERROR] ur_description not found!\n"
            "Please install it with:\n"
            "  sudo apt install ros-$(. /opt/ros/*/setup.bash && echo $ROS_DISTRO)-ur-description\n"
        )

    # Use the first (and usually only) match found
    xacro_file = xacro_candidates[0]
    print(f"[gesture_arm] Using URDF from: {xacro_file}")

    robot_desc = ParameterValue(
        Command(['xacro ', xacro_file, ' name:=ur', ' ur_type:=ur5e']),
        value_type=str
    )

    return LaunchDescription([

        # ── Node 1: robot_state_publisher ──────────────────────────────────
        # Reads the URDF and broadcasts TF transforms for every robot link
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),

        # ── Node 2: Gesture arm controller (IK + Scene + AUTO mode) ────────
        # Core node: reads UDP hand data, solves IK, publishes /joint_states
        Node(
            package='gesture_arm',
            executable='gesture_publisher',
            name='gesture_arm_node',
            output='screen'
        ),

        # ── Node 3: RViz2 (3D visualiser) ──────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
