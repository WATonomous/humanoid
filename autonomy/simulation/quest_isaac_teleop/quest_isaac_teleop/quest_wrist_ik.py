"""
Delta-control wrist IK for Quest teleop.

Zero the solver once (on first message or explicit call) to record the Quest
"home" wrist position. Every subsequent call returns 6 arm joint angles that
move the simulated palm by the same delta the user's wrist moved in the real
world.

Frame notes
-----------
WebXR "local" space  : Y-up, X-right, -Z-forward (right-handed)
MuJoCo / URDF space  : Z-up, X-forward (standard URDF convention)

Mapping (applied to delta vectors only, so the origin mismatch doesn't matter):
  arm +x  ←  quest -z  (forward)
  arm +y  ←  quest +x  (right)
  arm +z  ←  quest +y  (up)

If motion directions feel inverted, flip the sign of the corresponding row
in _R_QUEST_TO_ARM.
"""

from pathlib import Path
from types import ModuleType

import importlib
import importlib.util
import re
import numpy as np

_SIMULATION_DIR = Path(__file__).resolve().parents[2]
_ARM_ASSEMBLY_DIR = _SIMULATION_DIR / "Humanoid_Wato" / "arm_assembly"
_RIGHT_URDF = _ARM_ASSEMBLY_DIR / "arm_assembly.urdf"
_LEFT_URDF = _ARM_ASSEMBLY_DIR / "left_arm_assembly.urdf"

_RIGHT_ARM_JOINTS = [
    "shoulder_flexion_extension",
    "shoulder_abduction_adduction",
    "shoulder_rotation",
    "elbow_flexion_extension",
    "forearm_rotation",
    "wrist_extension",
]
_LEFT_ARM_JOINTS = [
    "left_shoulder_flexion_extension",
    "left_shoulder_abduction_adduction",
    "left_shoulder_rotation",
    "left_elbow_flexion_extension",
    "left_forearm_rotation",
    "left_wrist_extension",
]

_RIGHT_PALM_BODY = "PALM_GAVIN_1DoF_Hinge_v2_1"
_LEFT_PALM_BODY = "left_PALM_GAVIN_1DoF_Hinge_v2_1"

# WebXR → arm frame rotation (applied to delta vectors)
_R_QUEST_TO_ARM = np.array([
    [0,  0, -1],   # arm +x = quest -z  (forward)
    [1,  0,  0],   # arm +y = quest +x  (right)
    [0,  1,  0],   # arm +z = quest +y  (up)
], dtype=np.float64)


class MuJoCoUnavailableError(RuntimeError):
    """Raised when wrist IK is requested without the MuJoCo Python package."""


def _as_xyz(position) -> np.ndarray:
    """Accept ROS geometry points or plain xyz sequences."""
    if hasattr(position, "x") and hasattr(position, "y") and hasattr(position, "z"):
        return np.array([position.x, position.y, position.z], dtype=np.float64)
    return np.asarray(position, dtype=np.float64).reshape(3)


def _log(msg: str) -> None:
    print(f"[wrist_ik] {msg}", flush=True)


def _require_mujoco() -> ModuleType:
    if importlib.util.find_spec("mujoco") is None:
        raise MuJoCoUnavailableError(
            "MuJoCo is required for Quest wrist IK, but the Python package is "
            "not installed in this environment. Install it in the Isaac/teleop "
            "container with: python3 -m pip install mujoco"
        )
    _log("importing mujoco ...")
    module = importlib.import_module("mujoco")
    _log(f"mujoco {getattr(module, '__version__', '?')} imported")
    return module


def _load_model(urdf_path: Path, mujoco_module: ModuleType):
    urdf_path = Path(urdf_path).resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"Arm URDF not found: {urdf_path}")

    xml = urdf_path.read_text()

    # Wrist IK only needs the kinematic tree (links, joints, frames, Jacobians).
    # Strip <visual> and <collision> geometry so MuJoCo never loads the dense
    # STL meshes -- compiling convex hulls for ~20 high-poly meshes (some >19k
    # triangles) at load time can take minutes or appear to hang. Every link
    # carries an explicit <inertial>/<mass>, so removing geometry is safe.
    xml = re.sub(r"<visual\b.*?</visual>", "", xml, flags=re.DOTALL)
    xml = re.sub(r"<collision\b.*?</collision>", "", xml, flags=re.DOTALL)

    _log(f"compiling MuJoCo model from {urdf_path.name} (geometry stripped) ...")
    model = mujoco_module.MjModel.from_xml_string(xml)
    _log(f"model compiled: nbody={model.nbody} njnt={model.njnt}")
    return model


class WristIKSolver:
    """Converts a Quest wrist position delta into 6 arm joint angles via damped-LS IK."""

    def __init__(
        self,
        urdf_path: Path,
        palm_body: str,
        arm_joint_names: list[str],
        gain: float = 1.0,
        damping: float = 1e-4,
        step: float = 0.5,
        max_iter: int = 60,
        tol: float = 5e-3,
    ):
        self._mujoco = _require_mujoco()
        self._model = _load_model(urdf_path, self._mujoco)
        self._data = self._mujoco.MjData(self._model)
        self._palm_id = self._model.body(palm_body).id
        self._arm_joint_names = arm_joint_names
        self._n_arm = len(arm_joint_names)
        self._dof_indices = np.array(
            [self._model.joint(name).dofadr[0] for name in arm_joint_names],
            dtype=np.int32,
        )
        self._qpos_indices = np.array(
            [self._model.joint(name).qposadr[0] for name in arm_joint_names],
            dtype=np.int32,
        )
        self._joint_ids = np.array(
            [self._model.joint(name).id for name in arm_joint_names],
            dtype=np.int32,
        )
        self._gain = gain
        self._damping = damping
        self._step = step
        self._max_iter = max_iter
        self._tol = tol
        self._quest_home: np.ndarray | None = None

        # Neutral palm position at qpos=0
        self._data.qpos[:] = 0.0
        self._mujoco.mj_forward(self._model, self._data)
        self._neutral_palm = self._data.xpos[self._palm_id].copy()

    def zero(self, quest_pos) -> None:
        """Record the current Quest wrist position as the reference (home) point."""
        self._quest_home = _as_xyz(quest_pos)
        # Reset MuJoCo state to neutral so the solver starts from a known pose
        self._data.qpos[:] = 0.0
        self._data.qvel[:] = 0.0

    def solve(self, quest_pos) -> np.ndarray:
        """
        Return 6 arm joint angles that track the Quest wrist.

        Zeros automatically on the first call. Subsequent calls compute the
        delta from the home position, convert frames, and run IK.
        """
        if self._quest_home is None:
            self.zero(quest_pos)
            return np.zeros(self._n_arm, dtype=np.float64)

        delta_quest = _as_xyz(quest_pos) - self._quest_home
        delta_arm = _R_QUEST_TO_ARM @ delta_quest * self._gain
        target = self._neutral_palm + delta_arm

        for _ in range(self._max_iter):
            self._mujoco.mj_forward(self._model, self._data)
            err = target - self._data.xpos[self._palm_id]
            if np.max(np.abs(err)) < self._tol:
                break

            jacp = np.zeros((3, self._model.nv))
            jacr = np.zeros((3, self._model.nv))
            self._mujoco.mj_jacBody(self._model, self._data, jacp, jacr, self._palm_id)

            J = jacp[:, self._dof_indices]
            normal = J.T @ J + self._damping * np.eye(self._n_arm)
            dq = np.linalg.solve(normal, J.T @ err)
            self._data.qpos[self._qpos_indices] += self._step * dq

            for joint_id, qpos_id in zip(self._joint_ids, self._qpos_indices):
                lo, hi = self._model.jnt_range[joint_id]
                if lo < hi:
                    self._data.qpos[qpos_id] = np.clip(self._data.qpos[qpos_id], lo, hi)

        return self._data.qpos[self._qpos_indices].copy()


def make_right_solver(**kwargs) -> WristIKSolver:
    return WristIKSolver(_RIGHT_URDF, _RIGHT_PALM_BODY, _RIGHT_ARM_JOINTS, **kwargs)


def make_left_solver(**kwargs) -> WristIKSolver:
    return WristIKSolver(_LEFT_URDF, _LEFT_PALM_BODY, _LEFT_ARM_JOINTS, **kwargs)
