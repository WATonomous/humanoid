import math
import os
import tempfile
import yaml
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from common_msgs.msg import MotorFeedback


class Real2SimMirrorNode(Node):
    def __init__(self):
        super().__init__("real2sim_mirror_node")
        self.urdf_path = (
            "/root/ament_ws/assets/pioneer_bimanual_arm/"
            "urdf/pioneer_bimanual_arm.urdf"
        )

        self.mesh_directory = (
            "/root/ament_ws/assets/pioneer_bimanual_arm/meshes"
        )

        self.hardware_mapping_path = (
            "/root/ament_ws/src/joint_command/"
            "config/hardware_mapping.yaml"
        )

        self.lookup_table = self.load_hardware_mapping(
            "src/joint_command/config/hardware_mapping.yaml"
        )

        self.left_joint_names = {
            "shoulder_pitch": "joint1L",
            "shoulder_yaw": "joint2l",
            "shoulder_roll": "joint3l",
            "elbow_pitch": "joint4l",
            "elbow_roll": "joint5l",
            "wrist_pitch": "joint6l",
        }

        self.right_joint_names = {
            "shoulder_pitch": "joint1",
            "shoulder_yaw": "joint2",
            "shoulder_roll": "joint3",
            "elbow_pitch": "joint4",
            "elbow_roll": "joint5",
            "wrist_pitch": "joint6",
        }

        self.mirror_directions = {
            "shoulder_pitch": -1,
            "shoulder_roll": -1,
            "shoulder_yaw": -1,
            "elbow_pitch": -1,
            "elbow_roll": -1,
            "wrist_pitch": -1,
        }

        #temporary file to store the modified URDF for MuJoCo
        self.mujoco_urdf_path = self.create_mujoco_urdf()

        self.model = mujoco.MjModel.from_xml_path(
            self.mujoco_urdf_path
        )
        # Initialize MuJoCo data structure
        self.data = mujoco.MjData(self.model)

        self.left_qpos = {}
        self.right_qpos = {}

        self.setup_joint_indices()


        self.subscription = self.create_subscription(
            MotorFeedback,
            "/interfacing/motorFeedback",
            self.feedback_callback,
            10,
        )

        self.get_logger().info(
            "Real2Sim mirror visualization node started."
        )

    #Converts the hardware mapping YAML file into a lookup table for easy access
    def load_hardware_mapping(self, yaml_file_path):
        with open(yaml_file_path, "r") as f:
            data = yaml.safe_load(f)

        lookup_table = {}

        for side, limbs in data.items():
            for limb, joints in limbs.items():
                for joint_type, config in joints.items():
                    joint_name = f"{side}_{limb}_{joint_type}"

                    entry = config.copy()
                    entry["joint_name"] = joint_name

                    lookup_table[config["can_id"]] = entry

        return lookup_table
    #Temporary function for Mujoco urdf simulation. 
    def create_mujoco_urdf(self):
        """
        The original URDF uses ROS package:// mesh paths.

        MuJoCo does not resolve those ROS package paths, so create
        a temporary copy with the mesh paths replaced by the actual
        mounted mesh directory.
        """

        with open(self.urdf_path, "r") as f:
            urdf = f.read()

        urdf = urdf.replace(
            "package://armv2URDF/meshes/",
            self.mesh_directory + "/",
        )

        temp_file = tempfile.NamedTemporaryFile(
            mode="w",
            suffix=".urdf",
            delete=False,
        )

        temp_file.write(urdf)
        temp_file.close()

        self.get_logger().info(
            f"Created MuJoCo-readable URDF: {temp_file.name}"
        )

        return temp_file.name
    #Temporary function for Mujoco urdf simulation. 
    def setup_joint_indices(self):
        for joint_type, joint_name in self.left_joint_names.items():
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )

            if joint_id == -1:
                raise RuntimeError(
                    f"Could not find MuJoCo joint: {joint_name}"
                )

            self.left_qpos[joint_type] = self.model.jnt_qposadr[joint_id]

        for joint_type, joint_name in self.right_joint_names.items():
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )

            if joint_id == -1:
                raise RuntimeError(
                    f"Could not find MuJoCo joint: {joint_name}"
                )

            self.right_qpos[joint_type] = self.model.jnt_qposadr[joint_id]

        self.get_logger().info(
            f"Left qpos indices: {self.left_qpos}"
        )

        self.get_logger().info(
            f"Right qpos indices: {self.right_qpos}"
        )

    def angle_computation(self, motor_id, position_deg):
        if motor_id not in self.lookup_table:
            self.get_logger().warn(
                f"Unknown Motor ID {motor_id}"
            )
            return None

        config = self.lookup_table[motor_id]

        true_angle_deg = (
            position_deg - config["zero_offset"]
        ) * config["direction"]

        return math.radians(true_angle_deg)

    def mirror_angle(self, angle_rad, joint_name):
        joint_type = "_".join(joint_name.split("_")[1:])
        direction = self.mirror_directions[joint_type]

        return angle_rad * direction

    def feedback_callback(self, msg):
        motor_id = msg.motor_id
        position_deg = msg.position

        angle_rad = self.angle_computation(
            motor_id,
            position_deg,
        )

        if angle_rad is None:
            return

        joint_name = self.lookup_table[motor_id]["joint_name"]

        if not joint_name.startswith("left_"):
            return

        joint_type = "_".join(joint_name.split("_")[1:])

        if joint_type not in self.left_qpos:
            return

        ##Temporary code for Mujoco urdf simulation. 
        left_qpos_index = self.left_qpos[joint_type]
        self.data.qpos[left_qpos_index] = angle_rad
        ##

        mirrored_angle_rad = self.mirror_angle(angle_rad, joint_name)

        ##Temporary code for Mujoco urdf simulation. 
        right_qpos_index = self.right_qpos[joint_type]
        self.data.qpos[right_qpos_index] = mirrored_angle_rad
        # Update MuJoCo forward kinematics.
        mujoco.mj_forward(
            self.model,
            self.data,
        )
        ##

        self.get_logger().info(
            f"{joint_name}: "
            f"{angle_rad:.3f} rad -> "
            f"{mirrored_angle_rad:.3f} rad"
        )
    
def main():
    rclpy.init()

    node = Real2SimMirrorNode()
    #temporary code for Mujoco urdf simulation, will remove once xml model is found for mjlabs
    try:
        # ------------------------------------------------------------
        # Start MuJoCo viewer
        # ------------------------------------------------------------

        with mujoco.viewer.launch_passive(
            node.model,
            node.data,
        ) as viewer:

            node.get_logger().info(
                "MuJoCo viewer started."
            )

            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(
                    node,
                    timeout_sec=0.01,
                )

                viewer.sync()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()