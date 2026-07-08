"""Push-block environment: the arm pushes a block along the table, up a ramp,
onto the flat elevated interior floor of an open box.

Box geometry (from ``assets/box/box.obj``, corner-origin, Z-up):
254 x 254 x 31.75 mm shell, interior floor at z = 6.3 mm, walled on three
sides; the fourth side is open with a full-width ramp (table level up to the
floor over a ~29 mm run). The box is spawned with yaw -90 deg so its
up-the-ramp direction (box-local +y) is the env-frame ``+x`` axis.
"""

from dataclasses import MISSING
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from . import mdp

##
# Task geometry (env frame; robot base at the origin)
##

ASSETS_DIR = Path(__file__).resolve().parents[2] / "assets"

# box placed corner at (0.27, 0.127), yaw -90 deg: box-local +y (up the ramp) -> env +x
BOX_POS = (0.27, 0.127, 0.0)
BOX_QUAT = (0.70711, 0.0, 0.0, -0.70711)
PUSH_DIR = (1.0, 0.0)

RAMP_BASE_X = 0.279  # ramp meets the table (z=0)
RAMP_TOP_X = 0.308  # ramp meets the interior floor
FLOOR_Z = 0.0063  # interior floor height above the table (visual mesh)
# Effective COLLISION floor: the box USD's collision surface sits ~5 mm above the
# visual floor, so a settled 50.8 mm block rests at center z ~= 0.0369
# (= FLOOR_Z_COLLISION + BLOCK_HALF), not FLOOR_Z + BLOCK_HALF. Used only by the
# block_on_floor success check; FLOOR_Z stays the visual value for obs/scoop.
FLOOR_Z_COLLISION = 0.0115
FLOOR_X_MAX = 0.511  # interior floor end (back wall)
FLOOR_Y_HALF = 0.114  # interior floor half width

BLOCK_HALF = mdp.BLOCK_HALF_SIZE  # 50.8 mm cube, corner-origin USD
# block starts on the table in front of the ramp (center at ~(0.21, 0))
BLOCK_INIT_POS = (0.21 - BLOCK_HALF, -BLOCK_HALF, 0.0)
# target point on the interior floor, comfortably past the ramp top
FLOOR_TARGET = (0.37, 0.0)

##
# Spawn curriculum (performance-gated widening of the block spawn offsets)
##

# Default block center anchor (matches BLOCK_INIT_POS center = (0.21, 0.0)); the
# curriculum only grows the OFFSET magnitude around this fixed anchor.
FULL_YAW = (0.0, 6.2831853)
SPAWN_STAGES = [
    # stage 0: current moderate range (ramp-approach side only)
    {"x": (-0.06, 0.02), "y": (-0.06, 0.06), "yaw": FULL_YAW},
    # stage 1: wider approach, out to the edges of the ramp mouth
    {"x": (-0.10, 0.05), "y": (-0.12, 0.12), "yaw": FULL_YAW},
    # stage 2: reaches beside the box (|y| past the 0.127 wall) -> repositioning on
    {"x": (-0.12, 0.20), "y": (-0.20, 0.20), "yaw": FULL_YAW},
    # stage 3: full reachable table incl. beside + near-behind the box
    {"x": (-0.14, 0.30), "y": (-0.26, 0.26), "yaw": FULL_YAW},
]
# Curriculum stage (0-indexed) at which the block can spawn beside/behind the
# box and the repositioning reward switches on.
REPOSITION_START_STAGE = 2

# Box footprint to exclude from spawns (env frame). Open ramp (-x) side is left
# unmargined so the block can still spawn right at the ramp base; wall/back and
# lateral sides are inflated by the block half-extent so the body clears them.
BOX_EXCLUSION = {
    "x_min": RAMP_BASE_X,
    "x_max": 0.524 + BLOCK_HALF,
    "y_abs": 0.127 + BLOCK_HALF,
}


##
# Scene definition
##


@configclass
class PushBlockSceneCfg(InteractiveSceneCfg):
    """Scene with a robot, the block to push and the static ramp-box."""

    # robot and end-effector frame: populated by the agent env cfg
    robot: ArticulationCfg = MISSING
    ee_frame: FrameTransformerCfg = MISSING

    # dynamic block to push (corner-origin USD)
    object = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Object",
        init_state=RigidObjectCfg.InitialStateCfg(pos=BLOCK_INIT_POS, rot=[1, 0, 0, 0]),
        spawn=UsdFileCfg(
            usd_path=str(ASSETS_DIR / "block" / "block.usd"),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=1,
                max_angular_velocity=1000.0,
                max_linear_velocity=1000.0,
                max_depenetration_velocity=5.0,
                disable_gravity=False,
            ),
        ),
    )

    # static open box with ramp (no RigidBodyAPI in the USD -> fixed collider)
    box = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Box",
        init_state=AssetBaseCfg.InitialStateCfg(pos=BOX_POS, rot=BOX_QUAT),
        spawn=UsdFileCfg(usd_path=str(ASSETS_DIR / "box" / "box.usd")),
    )

    # Table
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0, 0], rot=[0.707, 0, 0, 0.707]),
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    )

    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -1.05]),
        spawn=GroundPlaneCfg(),
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
    """Action specifications for the MDP.

    Only the arm is actuated. The gripper joint is intentionally excluded from
    the action space: with no action term writing targets, the PD actuator
    holds it at its default (closed) position, so the policy physically cannot
    grasp the block - the closed gripper is the pushing tool.
    """

    arm_action: mdp.JointPositionActionCfg = MISSING


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        ee_position = ObsTerm(func=mdp.ee_position_in_robot_root_frame)
        block_position = ObsTerm(func=mdp.block_center_in_robot_root_frame)
        block_orientation = ObsTerm(func=mdp.block_orientation)
        block_lin_vel = ObsTerm(func=mdp.block_lin_vel)
        ramp_geometry = ObsTerm(
            func=mdp.ramp_geometry,
            params={
                "ramp_base_x": RAMP_BASE_X,
                "ramp_top_x": RAMP_TOP_X,
                "floor_z": FLOOR_Z,
                "target": FLOOR_TARGET,
            },
        )
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_object_position = EventTerm(
        func=mdp.reset_block_center_uniform,
        mode="reset",
        params={
            # Position + full yaw randomization of the block CENTER, as offsets
            # around the default center (0.21, 0.0). The spawn_curriculum term
            # overwrites this range in place, widening it stage by stage; the
            # value here is stage 0 (current moderate range) and is what a run
            # starts from. Box footprint is excluded via reject-and-resample so
            # widened offsets never spawn the block inside the walls/floor.
            "pose_range": dict(SPAWN_STAGES[0]),
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("object"),
            "box_exclusion": BOX_EXCLUSION,
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # reach + stay behind the block relative to the push direction
    ee_behind_block = RewTerm(
        func=mdp.ee_behind_block,
        params={"std": 0.06, "push_offset": 0.05},
        weight=2.0,
    )

    # dense progress along the up-the-ramp axis (signed velocity)
    push_progress = RewTerm(func=mdp.block_push_progress, params={"max_speed": 0.5}, weight=5.0)

    # extra asymmetric penalty for sliding back down the ramp
    backslide = RewTerm(func=mdp.block_backslide, weight=-8.0)

    # distance to the target point on the interior floor
    block_to_target = RewTerm(
        func=mdp.block_to_target,
        params={"std": 0.15, "target": FLOOR_TARGET},
        weight=8.0,
    )

    block_to_target_fine = RewTerm(
        func=mdp.block_to_target,
        params={"std": 0.05, "target": FLOOR_TARGET},
        weight=5.0,
    )

    # success: settled on the elevated interior floor
    block_on_floor = RewTerm(
        func=mdp.block_on_floor,
        params={
            "x_min": RAMP_TOP_X + BLOCK_HALF,
            "x_max": FLOOR_X_MAX - 0.01,
            "y_half": FLOOR_Y_HALF - 0.025,
            # use the effective collision floor: a settled block rests at center
            # z ~= FLOOR_Z_COLLISION + BLOCK_HALF = 0.0369, ~5 mm above the visual
            # floor, so keying on FLOOR_Z here made success unreachable.
            "floor_z": FLOOR_Z_COLLISION,
            "z_tol": 0.006,
            "rest_speed": 0.05,
        },
        weight=30.0,
    )

    # location-aware anti-scoop: no lifting in open table space
    # (~zero on the ramp and interior floor, where height gain is required)
    scoop_penalty = RewTerm(
        func=mdp.block_scoop_penalty,
        params={
            "ramp_base_x": RAMP_BASE_X,
            "ramp_top_x": RAMP_TOP_X,
            "floor_z": FLOOR_Z,
            "box_x_max": FLOOR_X_MAX,
            "box_y_half": FLOOR_Y_HALF,
            "height_tol": 0.01,
            "dist_std": 0.05,
        },
        weight=-40.0,
    )

    # sideways drift off the ramp approach corridor
    lateral_deviation = RewTerm(func=mdp.block_lateral_deviation, params={"y_tol": 0.06}, weight=-10.0)

    # repositioning shaping: route the EE behind an off-corridor block so it can
    # be pushed toward the ramp mouth. Starts inactive (weight 0); the spawn
    # curriculum turns it on only once spawns can land beside/behind the box
    # (stage >= REPOSITION_START_STAGE).
    ee_reposition = RewTerm(
        func=mdp.ee_reposition_behind_block,
        params={
            "std": 0.06,
            "push_offset": 0.05,
            "ramp_mouth": (RAMP_BASE_X, 0.0),
            "corridor_x_max": RAMP_BASE_X,
            "corridor_y": 0.06,
            "gate_std": 0.08,
        },
        weight=0.0,
    )

    # action penalties
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-4)

    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("object")}
    )

    block_off_course = DoneTerm(
        func=mdp.block_off_course,
        params={"x_min": 0.05, "x_max": 0.55, "y_limit": 0.20},
    )


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -1e-2, "num_steps": 10000}
    )

    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "joint_vel", "weight": -1e-2, "num_steps": 10000}
    )

    # Performance-gated widening of the block spawn offsets. Advances a stage
    # once the rolling success rate (block_on_floor) over the last `window`
    # completed episodes clears `threshold`; also switches on the ee_reposition
    # reward at REPOSITION_START_STAGE. Logs Curriculum/spawn/{stage,
    # success_rate, x_off_hi, reposition_active}.
    spawn = CurrTerm(
        func=mdp.spawn_offset_curriculum,
        params={
            "stages": SPAWN_STAGES,
            "event_term_name": "reset_object_position",
            "success_reward_term": "block_on_floor",
            "window": 100,
            "threshold": 0.70,
            # minimum training before a stage may advance. At num_steps_per_env=24
            # the env step counter grows 24/iteration, so 1200 steps ~= 50 PPO
            # iterations per stage. Without this the 100-episode window refills
            # many times per iteration and the curriculum cascades straight to the
            # last stage before the policy ever trains on the wider spawns.
            "min_stage_steps": 1200,
            "min_episode_len": 2,
            "reposition_reward_term": "ee_reposition",
            "reposition_weight": 1.5,
            "reposition_start_stage": REPOSITION_START_STAGE,
        },
    )


##
# Environment configuration
##


@configclass
class PushBlockEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the push-block environment."""

    # Scene settings
    scene: PushBlockSceneCfg = PushBlockSceneCfg(num_envs=4096, env_spacing=2.5)
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
        self.decimation = 2
        self.episode_length_s = 5.0
        self.viewer.eye = (2.5, 2.5, 1.5)
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = self.decimation

        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        # was 16 * 1024 (=16384): PhysX warned it needed ~16400 for 4096 envs and
        # dropped interactions. Give real headroom (8x) since randomized block
        # spawns/yaw create more varied contacts.
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 128 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
