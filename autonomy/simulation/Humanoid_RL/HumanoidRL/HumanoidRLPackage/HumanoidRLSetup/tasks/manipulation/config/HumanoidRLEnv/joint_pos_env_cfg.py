import math
from isaaclab.utils import configclass
import HumanoidRLPackage.HumanoidRLSetup.tasks.manipulation.mdp as mdp
from HumanoidRLPackage.HumanoidRLSetup.tasks.manipulation.reach_env_cfg import ReachEnvCfg
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.universal_robots import UR10_CFG
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid import HAND_CFG


@configclass
class HumanoidHandReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # self.scene.robot = UR10_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot = HAND_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        marker_scale = 0.02

        self.commands.ee_pose.goal_pose_visualizer_cfg.markers["frame"].scale = (
            marker_scale, marker_scale, marker_scale)
        self.commands.ee_pose.current_pose_visualizer_cfg.markers["frame"].scale = (
            marker_scale, marker_scale, marker_scale)

        self.commands.ee_pose_2.goal_pose_visualizer_cfg.markers["frame"].scale = (
            marker_scale, marker_scale, marker_scale)
        self.commands.ee_pose_2.current_pose_visualizer_cfg.markers["frame"].scale = (
            marker_scale, marker_scale, marker_scale)

        self.commands.ee_pose_3.goal_pose_visualizer_cfg.markers["frame"].scale = (
            marker_scale, marker_scale, marker_scale)
        self.commands.ee_pose_3.current_pose_visualizer_cfg.markers["frame"].scale = (
            marker_scale, marker_scale, marker_scale)

        self.commands.ee_pose_4.goal_pose_visualizer_cfg.markers["frame"].scale = (
            marker_scale, marker_scale, marker_scale)
        self.commands.ee_pose_4.current_pose_visualizer_cfg.markers["frame"].scale = (
            marker_scale, marker_scale, marker_scale)

        self.commands.ee_pose_5.goal_pose_visualizer_cfg.markers["frame"].scale = (
            marker_scale, marker_scale, marker_scale)
        self.commands.ee_pose_5.current_pose_visualizer_cfg.markers["frame"].scale = (
            marker_scale, marker_scale, marker_scale)

        # override events
        self.events.reset_robot_joints.params["position_range"] = (0.75, 1.25)
        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = [
            "TIP_B_1"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = [
            "TIP_B_1"]
        # self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["TIP_B_1"]

        self.rewards.end_effector_2_position_tracking.params["asset_cfg"].body_names = [
            "TIP_B_2"]
        self.rewards.end_effector_2_position_tracking_fine_grained.params["asset_cfg"].body_names = [
            "TIP_B_2"]
        # self.rewards.end_effector_2_orientation_tracking.params["asset_cfg"].body_names = ["TIP_B_2"]

        self.rewards.end_effector_3_position_tracking.params["asset_cfg"].body_names = [
            "TIP_B_3"]
        self.rewards.end_effector_3_position_tracking_fine_grained.params["asset_cfg"].body_names = [
            "TIP_B_3"]
        # self.rewards.end_effector_3_orientation_tracking.params["asset_cfg"].body_names = ["TIP_B_3"]

        self.rewards.end_effector_4_position_tracking.params["asset_cfg"].body_names = [
            "TIP_B_4"]
        self.rewards.end_effector_4_position_tracking_fine_grained.params["asset_cfg"].body_names = [
            "TIP_B_4"]
        # self.rewards.end_effector_4_orientation_tracking.params["asset_cfg"].body_names = ["TIP_B_4"]

        self.rewards.end_effector_5_position_tracking.params["asset_cfg"].body_names = [
            "TIP_B_5"]
        self.rewards.end_effector_5_position_tracking_fine_grained.params["asset_cfg"].body_names = [
            "TIP_B_5"]
        # self.rewards.end_effector_5_orientation_tracking.params["asset_cfg"].body_names = ["TIP_B_5"]

        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True)
        # override command generator body
        # end-effector is along x-direction
        self.commands.ee_pose.body_name = "TIP_B_1"
        # self.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2) #
        # this rotate the pose

        self.commands.ee_pose_2.body_name = "TIP_B_2"
        # self.commands.ee_pose_2.ranges.pitch = (math.pi / 2, math.pi / 2)

        self.commands.ee_pose_3.body_name = "TIP_B_3"
        # self.commands.ee_pose_3.ranges.pitch = (math.pi / 2, math.pi / 2)

        self.commands.ee_pose_4.body_name = "TIP_B_4"
        # self.commands.ee_pose_4.ranges.pitch = (math.pi / 2, math.pi / 2)

        self.commands.ee_pose_5.body_name = "TIP_B_5"
        # self.commands.ee_pose_5.ranges.pitch = (math.pi / 2, math.pi / 2)

        # self.commands.ee_pose.ranges.yaw = (-math.pi / 2, -math.pi / 2)
        # self.commands.ee_pose_2.ranges.yaw = (-math.pi / 2, -math.pi / 2)


@configclass
class HumanoidHandReachEnvCfg_PLAY(HumanoidHandReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False

# (env_isaaclab) hy@hy-LOQ-15IRX9:~/Downloads/Humanoid_Wato/HumanoidRL$ PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/train.py --task=Isaac-Reach-Humanoid-Hand-v0 --headless

# (env_isaaclab) hy@hy-LOQ-15IRX9:~/Downloads/Humanoid_Wato/HumanoidRL$ PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py --task=Isaac-Reach-Humanoid-Hand-v0 --num_envs=1
