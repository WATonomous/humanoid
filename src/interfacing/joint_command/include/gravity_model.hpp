#pragma once

#include <array>

// Static gravity load of the LEFT arm (URDF chain joint1L..joint6l, in ArmPose order), from the
// CAD masses and COMs in assets/pioneer_bimanual_arm/urdf/pioneer_bimanual_arm.urdf. The base is
// assumed upright (URDF +z up). Gripper fingers are lumped into link6l at their zero position.
//
// q_urdf_rad: joint angles in the URDF's own convention (NOT the ArmPose cmd frame).
// Returns the torque each joint must apply about its URDF axis to hold the arm still (N.m).
std::array<double, 6> leftArmGravityHoldTorque(const std::array<double, 6>& q_urdf_rad);
