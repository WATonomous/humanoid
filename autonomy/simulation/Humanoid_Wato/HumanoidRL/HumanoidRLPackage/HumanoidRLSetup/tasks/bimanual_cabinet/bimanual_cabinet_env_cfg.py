# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import FrameTransformerCfg, ContactSensorCfg
from isaaclab.sensors.frame_transformer import OffsetCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from . import mdp
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.bimanual_arm import BIMANUAL_ARM_CFG

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip


FRAME_MARKER_SMALL_CFG = FRAME_MARKER_CFG.copy()
FRAME_MARKER_SMALL_CFG.markers["frame"].scale = (0.10, 0.10, 0.10)


##
# Scene definition
##


@configclass
class CabinetSceneCfg(InteractiveSceneCfg):
    """Configuration for the cabinet scene with a robot and a cabinet.

    This is the abstract base implementation, the exact scene is defined in the derived classes
    which need to set the robot and end-effector frames
    """

    robot = BIMANUAL_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    cabinet = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Cabinet",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd",
            activate_contact_sensors=True,  # needed so the handle reports contact forces
            scale=(1.5, 1.5, 1.5),  # Scaled up from 1.15 — bigger handle bar for easier hooking
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.85, 0.0, 0.55),  # Pushed back +X slightly to keep spacing with robot
            rot=(0.0, 0.0, 0.0, 1.0),
            joint_pos={
                "door_left_joint": 0.0,
                "door_right_joint": 0.0,
                "drawer_bottom_joint": 0.0,
                "drawer_top_joint": 0.0,
            },
        ),
        actuators={
            "drawers": ImplicitActuatorCfg(
                joint_names_expr=["drawer_top_joint", "drawer_bottom_joint"],
                effort_limit_sim=87.0,
                # Zero stiffness so the drawer is a passive sliding mechanism
                stiffness=0.0,
                damping=0.05,
            ),
            "doors": ImplicitActuatorCfg(
                joint_names_expr=["door_left_joint", "door_right_joint"],
                effort_limit_sim=87.0,
                stiffness=10.0,
                damping=2.5,
            ),
        },
    )

    # Frame definitions for the cabinet.
    cabinet_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Cabinet/sektion",
        debug_vis=True,
        visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/CabinetFrameTransformer"),
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Cabinet/drawer_handle_top",
                name="drawer_handle_top",
                offset=OffsetCfg(
                    pos=(0.454, 0.0, 0.013),  # Scaled from (0.3485, 0.0, 0.01) by 1.5/1.15 ≈ 1.304x
                    rot=(0.5, 0.5, -0.5, -0.5),  # align with end-effector frame
                ),
            ),
        ],
    )

    # Contact sensors on the two inner fingers, filtered to ONLY report contact with
    # the drawer handle. Used by the pull rewards to confirm the claw is truly gripping.
    contact_link7 = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/link7",
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Cabinet/drawer_handle_top"],
        history_length=0,
        track_air_time=False,
    )
    contact_link8 = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/link8",
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Cabinet/drawer_handle_top"],
        history_length=0,
        track_air_time=False,
    )

    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(),
        spawn=sim_utils.GroundPlaneCfg(),
        collision_group=-1,
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


##
# MDP settings
##


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        scale=0.8,
        use_default_offset=True,
    )
    gripper_action = mdp.BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["joint7", "joint8"],
        # OPEN = wide (~11.5cm gap) so the bar can pass between the fingers on approach.
        # CLOSE = fingers together (0.0) so the claw can actually CLAMP the bar and pull.
        # Previously both were identical (locked open) — the claw could never pinch.
        open_command_expr={"joint7": -0.14, "joint8": 0.14},
        close_command_expr={"joint7": 0.0, "joint8": 0.0},
    )
    # Dummy action to actively hold the left arm at its resting pose
    left_arm_hold = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["joint1L", "joint2l", "joint3l", "joint4l", "joint5l", "joint6l", "joint7l", "joint8l"],
        scale=0.0,  # Neural network output is multiplied by zero
        use_default_offset=True,  # Always targets the default resting position
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        cabinet_joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"])},
        )
        cabinet_joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"])},
        )
        rel_ee_drawer_distance = ObsTerm(func=mdp.rel_ee_drawer_distance)

        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    robot_physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.8, 1.25),
            "dynamic_friction_range": (0.8, 1.25),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 16,
        },
    )

    cabinet_physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("cabinet", body_names="drawer_handle_top"),
            # Lowered from (2.0, 2.5) — high friction let the robot drag the drawer
            # open by pressing down on TOP of the bar. Lower friction forces it to
            # mechanically HOOK a finger behind the bar to pull. Tune upward if the
            # grip slips too much once hooking is learned.
            # Higher friction so a clamped pinch grip can transfer pulling force.
            # The contact-based rewards block cheats, so high friction is safe again.
            "static_friction_range": (1.5, 2.0),
            "dynamic_friction_range": (1.2, 1.6),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 16,
        },
    )

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", 
                joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"]
            ),
            "position_range": (-0.1, 0.1),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # =========================================================================
    # CLEAN REWARD LADDER
    # Each behavior is rewarded EXACTLY ONCE. Weights escalate monotonically by
    # stage, and drawer-opening rewards are GATED behind a correct straddle grip
    # so the drawer cannot be opened by cheating (top-drape / friction drag).
    # Dynamic range is kept sane (0.5 .. 300) to avoid one reward swamping the rest.
    # =========================================================================

    # ── STAGE 1: Reach the handle (dense, small) ─────────────────────────────
    approach_ee_handle = RewTerm(func=mdp.approach_ee_handle, weight=2.0, params={"threshold": 0.2})
    align_ee_handle = RewTerm(func=mdp.align_ee_handle, weight=0.5)

    # ── STAGE 2: Position the OPEN claw near the handle (dense, medium) ───────
    # Breadcrumb: either finger getting near (OR logic) — keeps a gradient alive early.
    single_claw_proximity = RewTerm(
        func=mdp.single_claw_proximity,
        weight=5.0,
        params={"contact_radius": 0.06},
    )
    # Lightly encourage an OPEN claw ONLY during the far approach so the bar can pass
    # between the fingers. Weight cut from 100 → 15: it must NOT fight the claw CLOSING
    # to clamp the bar once in position (that conflict stalled pulling entirely).
    open_claw_approach = RewTerm(
        func=mdp.open_claw_approach_reward,
        weight=15.0,
        params={
            "gripper_cfg": SceneEntityCfg("robot", joint_names=["joint7", "joint8"]),
            "open_target": 0.1,
            "aperture_sigma": 0.04,
            "proximity_radius": 0.15,
        },
    )
    # Rotate the approach so the fingers move toward OPPOSITE sides of the bar.
    approach_angle_reward = RewTerm(
        func=mdp.approach_angle_reward,
        weight=10.0,
        params={"proximity_radius": 0.15},
    )

    # ── STAGE 3: Bring BOTH fingers in together (milestone, medium-high) ──────
    # Stepping stone between single-finger proximity and full straddle.
    dual_approach_bonus = RewTerm(
        func=mdp.dual_approach_bonus,
        weight=150.0,  # Boosted from 80 — stronger gradient for the 5cm→2cm push
        params={"near_threshold": 0.06},
    )

    # ── STAGE 4: Get both fingers to opposite sides of the bar (positioning) ──
    # Guides the two fingers to straddle the bar (over/under or either side) so their
    # INNER faces can make contact. No behind/hook bias — any opposite-side config works.
    dual_claw_straddle = RewTerm(
        func=mdp.dual_claw_straddle,
        weight=150.0,
        params={"contact_radius": 0.08},
    )

    # ── STAGE 5: GOOD GRIP — both INNER edges touching the handle (contact sensor) ──
    # NOTE: Small weight — this is a GUIDING signal only, not a reward to be farmed.
    # The real payoff is in Stage 6 where grip quality multiplies the pull reward.
    # If this were large, the robot would learn to just hold the grip and never pull.
    inner_edge_grip = RewTerm(
        func=mdp.inner_edge_grip_reward,
        weight=10.0,  # Reduced 10× from 100 — guidance only, not farmable
        params={
            "force_threshold": 1.0,
            "both_bonus": 4.0,
            "center_sigma": 0.04,
        },
    )

    # ── STAGE 6: Pull the drawer (goal — ALL gated by a GOOD GRIP: both inner edges) ──
    # No pull reward fires unless BOTH inner edges are in contact (good_grip_gate).
    #
    # Dense + massively escalating pull reward.
    # Raw output at 0.01cm = 10 (before grip multiplier).
    # Raw output at full open = ~1,151,000 — ~115,000× the 0.01cm value.
    # weight=0.001 brings per-step reward to sensible range:
    #   0.01cm: 10 × 0.69 × 0.001 = 0.0069 per step
    #   full open: 1,151,000 × 0.69 × 0.001 = 794 per step → ~380k per episode
    pull_distance_reward = RewTerm(
        func=mdp.pull_distance_reward,
        weight=0.001,
        params={
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]),
            "max_open": 0.39,
            "force_threshold": 1.0,
        },
    )
    # Continuous pull velocity: good grip × drawer velocity. Rewards one smooth swing.
    continuous_pull_reward = RewTerm(
        func=mdp.continuous_pull_reward,
        weight=0.5,
        params={
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]),
            "force_threshold": 1.0,
        },
    )
    # Posture bonus: upright wrist while holding a good grip and pulling.
    upright_pull_bonus = RewTerm(
        func=mdp.upright_pull_bonus,
        weight=0.2,
        params={
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]),
            "threshold": 0.01,
            "force_threshold": 1.0,
        },
    )

    # ── PENALTIES (small, proportional) ──────────────────────────────────────
    # Cosmetic smoothness — scale down as the drawer opens so they never block pulling.
    action_rate_l2 = RewTerm(
        func=mdp.conditional_action_rate_l2,
        weight=-0.05,
        params={"asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"])},
    )
    joint_vel = RewTerm(
        func=mdp.conditional_joint_vel_l2,
        weight=-0.01,
        params={
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]),
            "robot_cfg": SceneEntityCfg("robot", joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"])
        },
    )

    # ── DEBUG (weight 0 — prints INNER vs OUTER edge contact counts per iteration) ──
    debug_inner_edge = RewTerm(func=mdp.debug_inner_edge, weight=0.0)


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    success = DoneTerm(
        func=mdp.joint_pos_out_of_manual_limit,
        params={"asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]), "bounds": (0.0, 0.39)}
    )

@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""
    pass


##
# Environment configuration
##


@configclass
class CabinetEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the cabinet environment."""

    # Scene settings
    scene: CabinetSceneCfg = CabinetSceneCfg(num_envs=4096, env_spacing=2.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 1
        self.episode_length_s = 8.0
        self.viewer.eye = (-2.5, 2.0, 2.0)
        self.viewer.lookat = (0.85, 0.0, 0.5)
        # simulation settings
        self.sim.dt = 1 / 60  # 60Hz
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.friction_correlation_distance = 0.00625
        self.scene.robot.init_state.pos = (-0.25, 0.0, 0.55)  # Pushed back and raised for larger cabinet
        # Enable contact reporting on the robot so the finger ContactSensors work
        self.scene.robot.spawn.activate_contact_sensors = True


# PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/train.py --task=Isaac-Open-Drawer-Humanoid-Arm-v0 --headless

# PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py --task=Isaac-Open-Drawer-Humanoid-Arm-v0 --num_envs=1
