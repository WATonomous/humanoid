#pragma once

#include <array>

// Static gravity hold torque (N.m) of the LEFT arm (joint1L..joint6l, ArmPose order) from the
// URDF's CAD masses, base upright. q_urdf_rad is in the URDF's convention, not the cmd frame.
std::array<double, 6> leftArmGravityHoldTorque(const std::array<double, 6>& q_urdf_rad);
