"""Re-export SimLeRobotRecorder from humanoid_il."""
from humanoid_il.sim_recorder import SimLeRobotRecorder as LeRobotRecorder  # noqa: F401

__all__ = ["LeRobotRecorder"]
