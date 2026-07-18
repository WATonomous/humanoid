"""Isaac Lab Mimic wrapper for Isaac-PickPlace-BimanualLeft-v0.

Keeps the base task's action space exactly as-is (8-dim joint positions,
cuRobo-native -- see pick_place_env_cfg.py's ActionsCfg). Isaac Lab Mimic's
core API (target_eef_pose_to_action / action_to_target_eef_pose) is normally
paired with an IK-relative-pose action term; here it's bridged onto the
joint-position action space using Isaac Lab's own DifferentialIKController
plus the Jacobian utilities already used, for this exact
robot/gripper-tip convention, by
Task_space_controller/robot_arm_controllers/task_space_real.py. No cuRobo
code is touched anywhere in this file.

Nothing in pick_place_env_cfg.py's ActionsCfg/ObservationsCfg/TerminationsCfg
or EventCfg is modified -- generate_demos.py and existing recorded datasets
are unaffected. All Mimic-specific scene/termination/observation additions
(the "grasp" subtask signal, the success termination, and a placeholder
scene marker for "table" placement mode) live only in the subclasses below.

Usage (mirrors pick_place_env_cfg.make_env_cfg):
    from pick_place_bimanual_mimic_env import make_mimic_env_cfg
    env = gym.make("Isaac-PickPlace-BimanualLeft-Mimic-v0",
                    cfg=make_mimic_env_cfg(PickPlaceTaskParams.from_yaml(path)))
"""
from __future__ import annotations

from collections.abc import Sequence

import torch

import isaaclab.utils.math as PoseUtils
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.envs import ManagerBasedRLMimicEnv
from isaaclab.envs.mimic_env_cfg import MimicEnvCfg, SubTaskConfig
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from isaaclab.utils.math import subtract_frame_transforms

from . import mdp
from .pick_place_env_cfg import (
    ObservationsCfg,
    PickPlaceBimanualEnvCfg,
    TerminationsCfg,
    _cuboid_object_cfg,
    apply_task_params,
)
from .robot_cfg_shim import (
    LEFT_ARM_JOINTS,
    LEFT_EE_BODY,
    LEFT_FINGER_TIP_BODIES,
    LEFT_GRIPPER_JOINTS,
    PickPlaceTaskParams,
    compute_gripper_tip_pose_b,
    compute_tip_ik_jacobian,
    resolve_body_ids,
)
from .robot_cfg_shim import wato_constants as wc

_EEF_NAME = "left_arm"
# object_ref for the second (place) subtask, keyed by PlaceParams.mode. Every
# mode needs a live scene asset here: "stack" and "tray" already have one
# (place_object / tray); "table" gets a marker via _add_place_target_marker.
_PLACE_OBJECT_REF = {"stack": "place_object", "tray": "tray", "table": "place_target"}


# ---------------------------------------------------------------------------
# Mimic-only observation/termination additions (base ObservationsCfg /
# TerminationsCfg are untouched -- see module docstring).
# ---------------------------------------------------------------------------


@configclass
class MimicObservationsCfg(ObservationsCfg):
    @configclass
    class SubtaskTermsCfg(ObsGroup):
        grasp = ObsTerm(func=mdp.object_grasped, params={})  # params filled below

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    subtask_terms: SubtaskTermsCfg = SubtaskTermsCfg()


@configclass
class MimicTerminationsCfg(TerminationsCfg):
    success: DoneTerm = DoneTerm(func=mdp.object_placed_success, params={})  # params filled below


def _add_place_target_marker(cfg: "PickPlaceBimanualMimicEnvCfg", params: PickPlaceTaskParams) -> None:
    """"table" placement mode has no scene object at the place target -- it's
    an ad-hoc random point sampled by generate_demos.py's Python driver, not
    a scene asset. Isaac Lab Mimic needs a live asset there for both the
    success check and get_object_poses()'s per-subtask object_ref, so this
    adds a tiny, non-colliding, kinematic marker and folds it into the
    existing reset_objects_min_separation event (same min_separation already
    applied to `object`, matching generate_demos.py's sample_place_target).
    """
    mid = (
        (params.place.x_range[0] + params.place.x_range[1]) / 2,
        (params.place.y_range[0] + params.place.y_range[1]) / 2,
        wc.TABLE_TOP_Z,
    )
    marker = _cuboid_object_cfg("PlaceTarget", (0.01, 0.01, 0.002), 0.001, (0.15, 0.85, 0.15), mid)
    marker.spawn.collision_props = None
    marker.spawn.rigid_props.kinematic_enabled = True
    marker.spawn.rigid_props.disable_gravity = True
    cfg.scene.place_target = marker

    reset_params = cfg.events.reset_objects.params
    reset_params["asset_cfgs"] = list(reset_params["asset_cfgs"]) + [SceneEntityCfg("place_target")]
    reset_params["z_values"] = list(reset_params["z_values"]) + [wc.TABLE_TOP_Z]


def _wire_mimic_extras(cfg: "PickPlaceBimanualMimicEnvCfg", params: PickPlaceTaskParams) -> None:
    """Fills in everything apply_task_params() doesn't know about: the grasp
    observation params, the success termination params, and (table mode
    only) the place-target marker."""
    if params.place.mode == "table":
        _add_place_target_marker(cfg, params)

    cfg.observations.subtask_terms.grasp.params = {
        "lift_height_threshold": 0.5 * params.motion.lift_height,
    }

    success_params = {
        "xy_tolerance": params.place.xy_tolerance,
        "z_tolerance": params.place.z_tolerance,
        "max_lin_vel": params.success.max_lin_vel,
        "object_half_height": params.object.size[2] / 2,
    }
    if params.place.mode == "stack":
        success_params["target_cfg"] = SceneEntityCfg("place_object")
        success_params["target_support_offset"] = params.place.stack_object_size[2] / 2
    elif params.place.mode == "tray":
        s = params.place.tray_scale * params.place.tray_height_scale
        success_params["target_xy"] = tuple(params.place.tray_center)
        success_params["target_support_z"] = wc.TABLE_TOP_Z + s * wc.TRAY_FLOOR_LOCAL_Z
    else:  # table
        success_params["target_cfg"] = SceneEntityCfg("place_target")
        success_params["target_support_offset"] = 0.0
    cfg.terminations.success.params = success_params

    cfg.subtask_configs[_EEF_NAME][1].object_ref = _PLACE_OBJECT_REF[params.place.mode]


# ---------------------------------------------------------------------------
# Env cfg
# ---------------------------------------------------------------------------


@configclass
class PickPlaceBimanualMimicEnvCfg(PickPlaceBimanualEnvCfg, MimicEnvCfg):
    """Isaac Lab Mimic config for the wato_bimanual_arm pick-and-place task.

    __post_init__ self-configures with default (table-mode) PickPlaceTaskParams
    so the class is usable standalone via gym's registered entry point (as
    annotate_demos.py / generate_dataset.py expect when driven by --task
    name alone). For a specific YAML config (tray/stack/etc, matching what
    generate_demos.py's --task_params takes), build with make_mimic_env_cfg()
    instead -- same pattern as pick_place_env_cfg.make_env_cfg().
    """

    observations: MimicObservationsCfg = MimicObservationsCfg()
    terminations: MimicTerminationsCfg = MimicTerminationsCfg()

    def __post_init__(self):
        super().__post_init__()

        default_params = PickPlaceTaskParams()
        apply_task_params(self, default_params)

        # The Mimic tooling driven off this registered env (annotate_demos.py,
        # generate_dataset.py) is state-only: the observation group is joint/
        # object state with no image terms, and source demos are recorded
        # cameras-off. Drop the scene cameras so these steps don't require
        # --enable_cameras -- that flag spins up the RTX render pipeline whose
        # worker-thread init deadlocks during headless scene setup here, and
        # nothing in the datagen path consumes the camera output anyway.
        # (make_mimic_env_cfg() below re-applies real params, so a caller that
        # explicitly wants cameras still gets them via that path.)
        self.scene.camera_external = None
        self.scene.camera_wrist = None

        self.datagen_config.name = "demo_src_pick_place_bimanual_D0"
        self.datagen_config.generation_guarantee = True
        self.datagen_config.generation_keep_failed = False
        self.datagen_config.generation_num_trials = 100
        self.datagen_config.generation_select_src_per_subtask = True
        self.datagen_config.generation_transform_first_robot_pose = False
        self.datagen_config.generation_interpolate_from_last_target_pose = True
        self.datagen_config.max_num_failures = 25
        self.datagen_config.seed = 1

        self.subtask_configs[_EEF_NAME] = [
            SubTaskConfig(
                object_ref="object",
                subtask_term_signal="grasp",
                subtask_term_offset_range=(10, 20),
                selection_strategy="nearest_neighbor_object",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.01,
                num_interpolation_steps=5,
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
                description="Grasp the object",
                next_subtask_description="Move the object to the place target",
            ),
            SubTaskConfig(
                object_ref=None,  # resolved below by _wire_mimic_extras (mode-dependent)
                subtask_term_signal=None,  # final subtask: no termination signal needed
                subtask_term_offset_range=(0, 0),
                selection_strategy="nearest_neighbor_object",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.01,
                num_interpolation_steps=5,
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
            ),
        ]

        _wire_mimic_extras(self, default_params)


def make_mimic_env_cfg(params: PickPlaceTaskParams) -> PickPlaceBimanualMimicEnvCfg:
    """Same recipe as pick_place_env_cfg.make_env_cfg(), plus Mimic wiring.

    Re-applies apply_task_params() with the real params (overwriting the
    table-mode default __post_init__ used) and re-resolves everything
    _wire_mimic_extras() sets up for the actual place.mode.
    """
    cfg = PickPlaceBimanualMimicEnvCfg()
    apply_task_params(cfg, params)
    _wire_mimic_extras(cfg, params)
    return cfg


# ---------------------------------------------------------------------------
# Env
# ---------------------------------------------------------------------------


class PickPlaceBimanualMimicEnv(ManagerBasedRLMimicEnv):
    """Isaac Lab Mimic environment for the joint-position action space above.

    get_object_poses() is intentionally not overridden: the
    ManagerBasedRLMimicEnv default (all rigid-object scene state, env-local
    frame) already matches the robot base frame used everywhere else in this
    task, since the robot sits at the env origin (see README).
    """

    def __init__(self, cfg: PickPlaceBimanualMimicEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        robot = self.scene["robot"]
        joint_names = list(robot.data.joint_names)
        self._left_arm_ids = [joint_names.index(j) for j in LEFT_ARM_JOINTS]
        self._wrist_body_id, = resolve_body_ids(robot, [LEFT_EE_BODY])
        self._finger_body_ids = resolve_body_ids(robot, list(LEFT_FINGER_TIP_BODIES))
        self._ee_jacobi_idx = self._wrist_body_id - 1 if robot.is_fixed_base else self._wrist_body_id

        self._diff_ik = DifferentialIKController(
            DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
            num_envs=self.num_envs,
            device=self.device,
        )

    # ---- eef pose <-> action ------------------------------------------------

    def get_robot_eef_pose(self, eef_name: str, env_ids: Sequence[int] | None = None) -> torch.Tensor:
        if env_ids is None:
            env_ids = slice(None)
        robot = self.scene["robot"]
        root_pose_w = robot.data.root_state_w[:, 0:7]
        tip_pos_b, tip_quat_b = compute_gripper_tip_pose_b(
            robot, root_pose_w, self._wrist_body_id, self._finger_body_ids
        )
        return PoseUtils.make_pose(tip_pos_b[env_ids], PoseUtils.matrix_from_quat(tip_quat_b[env_ids]))

    def target_eef_pose_to_action(
        self,
        target_eef_pose_dict: dict,
        gripper_action_dict: dict,
        action_noise_dict: dict | None = None,
        env_id: int = 0,
    ) -> torch.Tensor:
        (target_pose,) = target_eef_pose_dict.values()
        target_pos, target_rot = PoseUtils.unmake_pose(target_pose)
        target_quat = PoseUtils.quat_from_matrix(target_rot)

        robot = self.scene["robot"]
        root_pose_w = robot.data.root_state_w[env_id : env_id + 1, 0:7]
        joint_pos = robot.data.joint_pos[env_id : env_id + 1][:, self._left_arm_ids]
        ee_pose_w = robot.data.body_state_w[env_id : env_id + 1, self._wrist_body_id, 0:7]
        ee_pos_b, _ = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
        )
        tip_pos_b, tip_quat_b = compute_gripper_tip_pose_b(
            robot, root_pose_w, self._wrist_body_id, self._finger_body_ids
        )

        self._diff_ik.set_command(torch.cat([target_pos, target_quat], dim=0).unsqueeze(0))
        jacobian_w = robot.root_physx_view.get_jacobians()[
            env_id : env_id + 1, self._ee_jacobi_idx, :, self._left_arm_ids
        ]
        jacobian = compute_tip_ik_jacobian(robot, jacobian_w, ee_pos_b, tip_pos_b)
        joint_pos_des = self._diff_ik.compute(tip_pos_b, tip_quat_b, jacobian, joint_pos)[0]

        (gripper_action,) = gripper_action_dict.values()
        action = torch.cat([joint_pos_des, gripper_action.to(joint_pos_des.device, joint_pos_des.dtype)], dim=0)
        if action_noise_dict is not None:
            noise = action_noise_dict[_EEF_NAME] * torch.randn_like(action)
            action = action + noise
        return action

    def action_to_target_eef_pose(self, action: torch.Tensor) -> dict[str, torch.Tensor]:
        # NOTE: reads the robot's CURRENT simulated pose rather than
        # forward-kinematics-ing `action`'s arm-joint slice directly. A true
        # FK-from-arbitrary-joint-vector needs an off-simulator kinematic
        # model this task doesn't have wired up (only cuRobo would offer
        # that, and it's deliberately not used here -- see module
        # docstring). This is valid because the only caller in the Mimic
        # pipeline is the per-step recorder pre-step hook
        # (annotate_demos.py's PreStepDatagenInfoRecorder), i.e. it's always
        # called immediately alongside the action actively being tracked by
        # the joint PD controller at this pipeline's streaming control rate
        # (50 Hz) -- per-step tracking lag is small relative to subtask
        # segmentation granularity. Do NOT reuse this method to FK an
        # offline/arbitrary joint vector; it will silently return the
        # wrong pose.
        del action
        pose = self.get_robot_eef_pose(_EEF_NAME, env_ids=None)
        return {_EEF_NAME: pose}

    def actions_to_gripper_actions(self, actions: torch.Tensor) -> dict[str, torch.Tensor]:
        return {_EEF_NAME: actions[..., -2:]}

    # ---- subtask signals ------------------------------------------------

    def get_subtask_term_signals(self, env_ids: Sequence[int] | None = None) -> dict[str, torch.Tensor]:
        if env_ids is None:
            env_ids = slice(None)
        signals = {}
        subtask_terms = self.obs_buf["subtask_terms"]
        if "grasp" in subtask_terms:
            signals["grasp"] = subtask_terms["grasp"][env_ids].squeeze(-1)
        return signals
