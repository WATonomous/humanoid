# Copyright (c) 2023, NVIDIA CORPORATION & AFFILIATES. All rights reserved.

# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.

# THIS CODE IS JUST A MODIFIED VERSION OF THE KUKA-ALLEGRO VERSION IN THE
# FABRICS REPO FOR OUR ARM

import torch
import os
import numpy as np

from fabrics_sim.fabric_terms.attractor import Attractor
from fabrics_sim.fabric_terms.joint_limit_repulsion import JointLimitRepulsion
from fabrics_sim.fabric_terms.body_sphere_3d_repulsion import BodySphereRepulsion
from fabrics_sim.fabric_terms.body_sphere_3d_repulsion import BaseFabricRepulsion
from fabrics_sim.fabrics.fabric import BaseFabric
from fabrics_sim.taskmaps.identity import IdentityMap
from fabrics_sim.taskmaps.upper_joint_limit import UpperJointLimitMap
from fabrics_sim.taskmaps.lower_joint_limit import LowerJointLimitMap
from fabrics_sim.taskmaps.linear_taskmap import LinearMap
from fabrics_sim.energy.euclidean_energy import EuclideanEnergy
from fabrics_sim.taskmaps.robot_frame_origins_taskmap import RobotFrameOriginsTaskMap


class HumanoidHandPoseFabric(BaseFabric):
    """
    Creates a fabric for the humanoid hand that opens up a pose action space for the palm
    and PCA'ed action space for the hand. Includes self-collision, env collision avoidance,
    joint limiting, accel/jerk limiting, speed control, redundancy resolution.
    """

    def __init__(self, batch_size, device, timestep, graph_capturable=True):
        """
        Constructor. Specifies parameter file and constructs the fabric.
        :param batch_size: size of the batch
        :param device: type str that sets the device for the fabric
        """
        # Load parameters
        fabric_params_filename = "humanoid_hand_params.yaml"
        super().__init__(device, batch_size, timestep, fabric_params_filename,
                         graph_capturable=graph_capturable)

        # URDF filpath for arm
        self.urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "../Humanoid_Wato/arm_assembly/arm_assembly.urdf"
                                      )

        self.load_robot(batch_size=batch_size)

        # Going to set a default config for the cspace attractor that gets
        # used until an actual cspace command comes in
        default_config = torch.tensor([
            # Arm joints (midpoint of each joint's range)
            0.0,    # shoulder_flexion_extension: mid of [-3.14, 3.14]
            1.57,   # shoulder_abduction_adduction: mid of [-0.35, 3.49]
            0.0,    # shoulder_rotation: mid of [-3.14, 3.14]
            1.57,   # elbow_flexion_extension: mid of [-0.35, 3.49]
            1.57,   # forearm_rotation: mid of [0.0, 3.14]
            0.0,    # wrist_extension: mid of [-0.96, 0.96]
            # Hand joints (midpoint of each joint's range)
            -0.79,  # mcp_index: mid of [-1.57, 0.0]
            0.79,   # pip_index: mid of [0.0, 1.57]
            -0.79,  # dip_index: mid of [-1.57, 0.0]
            -0.79,  # mcp_middle: mid of [-1.57, 0.0]
            0.79,   # pip_middle: mid of [0.0, 1.57]
            0.79,   # dip_middle: mid of [0.0, 1.57]
            0.79,   # mcp_ring: mid of [0.0, 1.57]
            -0.79,  # pip_ring: mid of [-1.57, 0.0]
            -0.79,  # dip_ring: mid of [-1.57, 0.0]
            0.79,   # mcp_pinky: mid of [0.0, 1.57]
            -0.79,  # pip_pinky: mid of [-1.57, 0.0]
            0.79,   # dip_pinky: mid of [0.0, 1.57]
            0.87,   # cmc_thumb: mid of [-0.35, 2.09]
            1.66,   # mcp_thumb: mid of [0.79, 2.53]
            0.79,   # ip_thumb: mid of [0.0, 1.57]
        ], device=self.device)

        self.default_config = default_config.unsqueeze(
            0).repeat(self.batch_size, 1)

        # Store pca matrix for hand
        self._pca_matrix = None

        # Construct the fabric.
        self.construct_fabric()

    # This is already a method in the base class but we need to override it to load our specific roboT
    # The original file only looks for files in the FABRICS folder. This
    # function overrides it to allow it to point to our URDF file
    def load_robot(self, robot_dir_name=None,
                   robot_name=None, batch_size=None):
        import warp as wp
        import warp.sim
        from urdfpy import URDF

        self.batch_size = batch_size
        builder = wp.sim.ModelBuilder()

        robot_urdf_filename = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../Humanoid_Wato/arm_assembly/arm_assembly.urdf"
        )

        initial_rotation = wp.quat(0., 0., 0., 1.)
        initial_position = wp.vec3(0., 0., 0.)
        initial_transform = wp.transform(initial_position, initial_rotation)

        self.urdfpy_robot = URDF.load(robot_urdf_filename)

        joints = self.urdfpy_robot.joints
        self._num_joints = 0
        for i in range(len(joints)):
            if joints[i].joint_type == 'revolute' or joints[i].joint_type == 'continuous':
                self._num_joints += 1
                self.joint_names.append(joints[i].name)

        print('importing robot')
        wp.sim.parse_urdf(robot_urdf_filename, builder, initial_transform)

        print('finalizing model')
        self.model = builder.finalize(device=self.device)
        self.model.ground = True

        self.allocate_scaled_accel()

        # Pre-allocate tensors (this is usually done in BaseFabric.load_robot)
        self.root_metrics = torch.zeros(
            self.batch_size, self._num_joints, self._num_joints, device=self.device)
        self.root_geometric_forces = torch.zeros(
            self.batch_size, self._num_joints, device=self.device)
        self.root_potential_forces = torch.zeros(
            self.batch_size, self._num_joints, device=self.device)
        self.root_energy_metrics = torch.zeros(
            self.batch_size, self._num_joints, self._num_joints, device=self.device)
        self.root_energy_forces = torch.zeros(
            self.batch_size, self._num_joints, device=self.device)
        self.energies = torch.zeros(self.batch_size, 1, device=self.device)
        self.metric_inv = torch.zeros(
            self.batch_size, self._num_joints, self._num_joints, device=self.device)

    def add_joint_limit_repulsion(self):
        joints = self.urdfpy_robot.joints
        upper_joint_limits = []
        lower_joint_limits = []
        for i in range(len(joints)):
            if joints[i].joint_type == 'revolute':
                upper_joint_limits.append(joints[i].limit.upper)
                lower_joint_limits.append(joints[i].limit.lower)
            elif joints[i].joint_type == 'continuous':
                # Fake limits for continuous joints
                upper_joint_limits.append(3.14)
                lower_joint_limits.append(-3.14)

        # Upper joint limit repulsion
        taskmap_name = "upper_joint_limit"
        taskmap = UpperJointLimitMap(
            upper_joint_limits, self.batch_size, self.device)
        self.add_taskmap(taskmap_name, taskmap,
                         graph_capturable=self.graph_capturable)

        is_forcing = True
        fabric_name = "joint_limit_repulsion"
        fabric = JointLimitRepulsion(is_forcing, self.fabric_params['joint_limit_repulsion'],
                                     self.device, graph_capturable=self.graph_capturable)
        self.add_fabric(taskmap_name, fabric_name, fabric)

        # Lower joint limit repulsion
        taskmap_name = "lower_joint_limit"
        taskmap = LowerJointLimitMap(
            lower_joint_limits, self.batch_size, self.device)
        self.add_taskmap(taskmap_name, taskmap,
                         graph_capturable=self.graph_capturable)

        is_forcing = True
        fabric_name = "joint_limit_repulsion"
        fabric = JointLimitRepulsion(is_forcing, self.fabric_params['joint_limit_repulsion'],
                                     self.device, graph_capturable=self.graph_capturable)
        self.add_fabric(taskmap_name, fabric_name, fabric)

    def add_cspace_attractor(self, is_forcing):
        """
        Add a cspace attractors to the fabric.
        -----------------------------
        :param is_forcing: bool, indicates whether the fabric term will be forcing
                           or not (geometric)
        """
        # Create taskmap and its container.
        taskmap_name = "identity"
        taskmap = IdentityMap(self.device)
        self.add_taskmap(taskmap_name, taskmap,
                         graph_capturable=self.graph_capturable)

        # Create fabric term and add to taskmap container.
        if not is_forcing:
            fabric_name = "cspace_attractor"
            fabric = Attractor(is_forcing, self.fabric_params['cspace_attractor'],
                               self.device, graph_capturable=self.graph_capturable)
        else:
            fabric_name = "forcing_cspace_attractor"
            fabric = Attractor(is_forcing, self.fabric_params['forcing_cspace_attractor'],
                               self.device, graph_capturable=self.graph_capturable)

        # Add it to container list in the root space
        self.add_fabric(taskmap_name, fabric_name, fabric)

    def add_hand_fabric(self):

        # Loading PCA matrix from other script (compute_pca.py)
        pca_path = os.path.join(os.path.dirname(
            os.path.abspath(__file__)), "pca_matrix.npy")
        pca_numpy = np.load(pca_path)

        # Transpose to (7, 15) — each row is one principal component
        pca_matrix = torch.tensor(
            pca_numpy.T, device=self.device, dtype=torch.float32)
        self._pca_matrix = torch.clone(pca_matrix.detach())

        # Load the PCA mean (mu) and encode it into PCA space.
        # sklearn PCA mean-centers data before fitting, so the correct PCA encoding is
        # W^T @ (q - mu). The LinearMap only computes W^T @ q, so we precompute
        # W^T @ mu and add it to every target in set_features to recover the correct
        # equilibrium (q* = W @ hand_target + mu instead of q* = W @
        # hand_target).
        mu_path = os.path.join(os.path.dirname(
            os.path.abspath(__file__)), "pca_mean.npy")
        mu = torch.tensor(
            np.load(mu_path),
            device=self.device,
            dtype=torch.float32)
        self._encoded_mean = self._pca_matrix @ mu  # (7,)

        # Pad with 6 zeros for arm joints (PCA only controls the hand, not the
        # arm)
        pca_matrix = torch.cat([
            torch.zeros(pca_matrix.shape[0], 6, device=self.device),
            pca_matrix
        ], dim=1)

        # Create taskmap and its container.
        taskmap_name = "pca_hand"
        taskmap = LinearMap(pca_matrix, self.device)
        self.add_taskmap(taskmap_name, taskmap,
                         graph_capturable=self.graph_capturable)

        # Place an attractor in this space
        fabric_name = "hand_attractor"
        is_forcing = True
        fabric = Attractor(is_forcing, self.fabric_params['hand_attractor'],
                           self.device, graph_capturable=self.graph_capturable)

        # Add it to container list
        self.add_fabric(taskmap_name, fabric_name, fabric)

    def add_body_repulsion(self):
        """
        Creates body spheres and repulsion between body spheres (self-collision) and also between
        body spheres and environment objects.
        """
        # Create list of frames that will be used to place body spheres at
        # their origins
        collision_sphere_frames = self.fabric_params['body_repulsion']['collision_sphere_frames']

        # List of sphere radii, one for each frame origin
        self.collision_sphere_radii = self.fabric_params['body_repulsion']['collision_sphere_radii']

        assert (len(collision_sphere_frames) == len(self.collision_sphere_radii)), \
            "length of link names does not equal length of radii"

        # Declare which body spheres need to avoid collision
        collision_sphere_pairs = self.fabric_params['body_repulsion']['collision_sphere_pairs']

        # Calculate the body collision matrix
        collision_matrix = torch.zeros(len(collision_sphere_frames), len(collision_sphere_frames), dtype=int,
                                       device=self.device)

        # If frames for collision sphere pairs were not manually specified, then look for the
        # link prefix pairs so that spheres associated with one link can avoid
        # spheres of the other link
        if len(collision_sphere_pairs) == 0:
            # Find links via prefixes to gather collision spheres for self
            # collision avoidance.
            collision_link_prefix_pairs = self.fabric_params[
                'body_repulsion']['collision_link_prefix_pairs']
            frames_for_prefix1 = None
            frames_for_prefix2 = None
            for prefix1, prefix2 in collision_link_prefix_pairs:
                frames_for_prefix1 = [
                    s for s in collision_sphere_frames if prefix1 in s]
                frames_for_prefix2 = [
                    s for s in collision_sphere_frames if prefix2 in s]

                for sphere1 in frames_for_prefix1:
                    for sphere2 in frames_for_prefix2:
                        collision_sphere_pairs.append([sphere1, sphere2])

        for sphere1, sphere2 in collision_sphere_pairs:
            collision_matrix[collision_sphere_frames.index(
                sphere1), collision_sphere_frames.index(sphere2)] = 1

        # Set name for taskmap, create it, and add to pool of taskmaps.
        taskmap_name = "body_points"
        taskmap = RobotFrameOriginsTaskMap(self.urdf_path, collision_sphere_frames,
                                           self.batch_size, self.device)
        self.add_taskmap(taskmap_name, taskmap,
                         graph_capturable=self.graph_capturable)

        # Create fabric term and add to taskmap container.
        fabric_name = "repulsion"
        is_forcing = True
        sphere_radius = torch.tensor(
            self.collision_sphere_radii, device=self.device)
        sphere_radius = sphere_radius.repeat(self.batch_size, 1)
        fabric = BodySphereRepulsion(is_forcing, self.fabric_params['body_repulsion'],
                                     self.batch_size, sphere_radius, collision_matrix, self.device,
                                     graph_capturable=self.graph_capturable)

        # Add it to container list
        self.add_fabric(taskmap_name, fabric_name, fabric)

        # Add geometric body repulsion
        fabric_geom = BodySphereRepulsion(False, self.fabric_params['body_repulsion'],
                                          self.batch_size, sphere_radius, collision_matrix, self.device,
                                          graph_capturable=self.graph_capturable)

        # Add it to container list
        self.add_fabric(taskmap_name, "geom_repulsion", fabric_geom)

        # Create object that constructs base response and signed distance
        self.base_fabric_repulsion =\
            BaseFabricRepulsion(self.fabric_params['body_repulsion'],
                                self.batch_size,
                                sphere_radius,
                                collision_matrix,
                                self.device)

    def add_cspace_energy(self):
        """
        Add a Euclidean cspace energy to the fabric.
        """
        # Add gripper energy.
        taskmap_name = "identity"
        energy_name = "euclidean"
        self.add_energy(taskmap_name, energy_name, EuclideanEnergy(
            self.batch_size, self._num_joints, self.device))

    def construct_fabric(self):
        """
        Construct the fabric by adding the various geometric, potential, and energy
        components.
        """
        # Add joint limit repulsion
        self.add_joint_limit_repulsion()

        # Add geometric cspace attractor
        self.add_cspace_attractor(False)

        # Add hand attractor
        self.add_hand_fabric()

        # Add multi-point gripper attractor
        # self.add_palm_points_attractor()
        # This line was commented out since it is not compatible with our URDF
        # file

        # Add collision avoidance
        self.add_body_repulsion()

        # Add energy
        self.add_cspace_energy()

    def get_sphere_radii(self):
        """
        Returns the radii for the body collision spheres.
        ------------------------------------------
        :return collision_sphere_radii: list of floats containing the radii
        """
        return self.collision_sphere_radii

    @property
    def collision_status(self):
        """
        Returns the collision state for each body sphere of the robot
        ------------------------------------------
        :return collision_status: bxn bool Pytorch tensor, b is batch size, n is number of body
                                  spheres
        """
        return self.base_fabric_repulsion.collision_status

    @property
    def pca_matrix(self):
        return self._pca_matrix

    @pca_matrix.setter
    def pca_matrix(self, pca_matrix):
        self._pca_matrix = pca_matrix

    def set_features(self, hand_target,
                     batched_cspace_position, batched_cspace_velocity,
                     object_ids,
                     object_indicator,
                     cspace_damping_gain=None):
        """
        Passes the input features to the various fabric terms.
        -----------------------------
        :param hand_target: bx7 Pytorch tensor that sets the desired location in PCA space.
                            Controls the fingers of the Aem.
        :param batched_cspace_position: bx21 Pytorch tensor, current fabric position
        :param batched_cspace_velocity: bx21 Pytorch tensor, current fabric velocity
        :param object_ids: 2D int Warp array referencing object meshes
        :param object_indicator: 2D Warp array of type uint64, indicating the presence
                                 of a Warp mesh in object_ids at corresponding index
                                 0=no mesh, 1=mesh
        """
        self.fabrics_features["pca_hand"]["hand_attractor"] = hand_target + \
            self._encoded_mean
        self.fabrics_features["identity"]["cspace_attractor"] = self.default_config

        # Calculate current location of body sphere origins and their velocity
        body_point_pos, jac = self.get_taskmap(
            "body_points")(batched_cspace_position, None)
        body_point_vel = torch.bmm(
            jac, batched_cspace_velocity.unsqueeze(2)).squeeze(2)

        # Calculate signed distance and repulsion response based on body sphere origin
        # position and velocity and objects in the world
        # NOTE: this calculates both self-collision and robot-world collision
        # response
        self.base_fabric_repulsion.calculate_response(body_point_pos,
                                                      body_point_vel,
                                                      object_ids,
                                                      object_indicator)

        # Pass the collision response data into both the forcing and geometric collision
        # avoidance fabric terms.
        self.fabrics_features["body_points"]["repulsion"] =\
            self.base_fabric_repulsion
        self.fabrics_features["body_points"]["geom_repulsion"] =\
            self.base_fabric_repulsion

        if cspace_damping_gain is not None:
            self.fabric_params['cspace_damping']['gain'] = cspace_damping_gain
