from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("hros5", package_name="hros5_moveit_config").to_moveit_configs()
    # Pass only the planning pipeline params to RViz; kinematics are owned by move_group and
    # can cause type clashes when RViz reloads them locally.
    moveit_config.robot_description_kinematics = {}
    return generate_moveit_rviz_launch(moveit_config)
