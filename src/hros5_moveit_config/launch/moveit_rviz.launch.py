from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("hros5", package_name="hros5_moveit_config").to_moveit_configs()
    # Keep RViz lightweight: let move_group own kinematics/limits to avoid parameter type clashes
    moveit_config.robot_description_kinematics = {}
    moveit_config.joint_limits = {}
    return generate_moveit_rviz_launch(moveit_config)
