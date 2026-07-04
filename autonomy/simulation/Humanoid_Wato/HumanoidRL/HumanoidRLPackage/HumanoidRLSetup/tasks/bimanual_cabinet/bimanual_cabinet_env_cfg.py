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
                # Very low stiffness + damping so even a weak hook grip can slide it open
                stiffness=1.0,
                damping=0.1,
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

    # ── STAGE 4: Hook a finger BEHIND + BELOW the bar (milestone, high) ───────
    # Dense 3D hook target inside the gap — the primary anti top-drape signal.
    finger_behind_handle = RewTerm(
        func=mdp.finger_behind_handle,
        weight=150.0,
        params={
            "behind_offset": 0.03,   # 3cm behind the bar (into the gap)
            "below_offset": 0.01,    # 1cm below the bar top edge
            "sigma": 0.02,           # tight target — pressure to actually get inside
            "proximity_radius": 0.12,
            "gap_sign": 1.0,         # flip to -1.0 if the gap is on the -X side
        },
    )
    # Shape the descent: reward being behind the bar AND at/below its top edge.
    descend_into_gap = RewTerm(
        func=mdp.descend_into_gap,
        weight=80.0,
        params={"proximity_radius": 0.10, "gap_sign": 1.0},
    )

    # ── STAGE 5: Straddle grip — both fingers close, opposite sides (gate) ────
    # FIX: contact_radius increased so avg-distance gate fires at 5-6cm approach.
    # Old product gate gave 0.003 at 5cm; new avg gate gives ~0.22 at 5cm.
    dual_claw_straddle = RewTerm(
        func=mdp.dual_claw_straddle,
        weight=150.0,
        params={"contact_radius": 0.08},  # wider radius — gradient fires from 8cm
    )

    # ── STAGE 5b: Inner-edge contact at the TOP/BOTTOM of the bar (vertical pinch) ──
    # Only rewards contact that is a graspable top/bottom grip — NOT a flat front-face
    # press and NOT the end cap. Big pinch bonus when one finger grips top, one bottom.
    edge_contact = RewTerm(
        func=mdp.edge_contact_reward,
        weight=100.0,
        params={
            "force_threshold": 1.0,
            "min_offset": 0.005,     # finger must be >5mm above/below bar center
            "on_bar_radius": 0.06,   # within 6cm of the bar center-line (rejects end cap)
            "pinch_bonus": 5.0,      # DOMINANT top+bottom simultaneous grip bonus
            "single_scale": 0.2,     # single-finger contact earns little — no farming
        },
    )

    # ── STAGE 6: Pull the drawer (goal — ALL gated by real handle CONTACT) ──────
    # A finger (link7 or link8) must be physically TOUCHING the handle for any to fire.
    #
    # SMALL flat bump when pulling starts (reduced from 800 → 150 so the robot can no
    # longer 'park just past 1cm' to farm a huge flat reward — it must keep opening).
    first_pull_bonus = RewTerm(
        func=mdp.first_pull_bonus,
        weight=150.0,
        params={
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]),
            "threshold": 0.01,
        },
    )
    # PRIMARY GOAL — SUPERLINEAR: opening FURTHER pays disproportionately more.
    # contact * (f + 3 f^2), f = open fraction. Fully open ≈ 120× a 1cm nudge.
    pull_distance_reward = RewTerm(
        func=mdp.pull_distance_reward,
        weight=2500.0,  # dominant term — the more it opens, the far bigger the reward
        params={
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]),
            "contact_radius": 0.05,
            "max_open": 0.39,
        },
    )
    # Open the drawer WITH a proper finger-behind-handle hook (main inner edge, not the
    # shallow side edge). = contact * open_fraction * behind_score. Big incentive to use
    # the correct hooking grip while pulling.
    hook_pull_reward = RewTerm(
        func=mdp.hook_pull_reward,
        weight=1500.0,
        params={
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]),
            "max_open": 0.39,
            "gap_sign": 1.0,
            "behind_scale": 0.03,
        },
    )
    # Early gradient: positive drawer velocity while a finger is touching the handle.
    drawer_vel_reward = RewTerm(
        func=mdp.drawer_vel_reward,
        weight=400.0,
        params={"asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"])},
    )
    # Posture bonus: upright wrist orientation WHILE touching the handle and pulling.
    upright_pull_bonus = RewTerm(
        func=mdp.upright_pull_bonus,
        weight=150.0,
        params={
            "asset_cfg": SceneEntityCfg("cabinet", joint_names=["drawer_top_joint"]),
            "threshold": 0.01,
        },
    )

    # ── PENALTIES (small, proportional) ──────────────────────────────────────
    # Discourage the single-finger local optimum.
    asymmetry_penalty = RewTerm(
        func=mdp.asymmetry_penalty,
        weight=-200.0,
        params={"contact_radius": 0.08, "penalty_threshold": 0.05},
    )
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

    # ── DEBUG (weight 0 — zero training effect; prints claw distances) ────────
    debug_link_distances = RewTerm(func=mdp.debug_link_distances, weight=0.0)


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
