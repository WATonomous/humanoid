#include "gravity_model.hpp"

#include <cmath>

namespace {

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<Vec3, 3>; // row-major

struct Link {
  Vec3 origin; // joint origin in the parent link frame (every URDF rpy here is 0)
  Vec3 axis;   // joint axis in the parent frame
  double mass; // child link, kg
  Vec3 com;    // child link COM in the child frame
};

// Generated from pioneer_bimanual_arm.urdf. joint6l's link includes link7l + link8l (fingers at 0).
constexpr std::array<Link, 6> kLeftArm = {{
    {{0.021573, 0.163325, 0.060482}, {0, -1, 0}, 0.946692, {0.006392, 0.072904, -0.000182}},
    {{-0.008820, 0.116000, -0.000181}, {1, 0, 0}, 0.524799, {-0.011359, 0.000048, -0.045618}},
    {{0.002000, 0.000000, -0.117300}, {0, 0, -1}, 0.617869, {-0.005983, -0.000142, -0.098088}},
    {{0.000000, 0.025250, -0.183600}, {0, 1, 0}, 0.236100, {0.000590, -0.023496, -0.053409}},
    {{0.000000, -0.023025, -0.169243}, {0, 0, 1}, 0.260193, {-0.000067, -0.001137, -0.009573}},
    {{0.000000, -0.045975, -0.066257}, {0, 1, 0}, 0.195008, {0.001487, 0.056667, -0.092801}},
}};

constexpr double kGravity = 9.81;

Vec3 add(const Vec3& a, const Vec3& b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 sub(const Vec3& a, const Vec3& b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

double dot(const Vec3& a, const Vec3& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

Vec3 mul(const Mat3& m, const Vec3& v) {
  return {dot(m[0], v), dot(m[1], v), dot(m[2], v)};
}

Mat3 mul(const Mat3& a, const Mat3& b) {
  Mat3 out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c];
    }
  }
  return out;
}

// Rodrigues: rotation by q about unit axis k.
Mat3 axisAngle(const Vec3& k, double q) {
  const double s = std::sin(q);
  const double c = 1.0 - std::cos(q);
  const Mat3 K = {{{0, -k[2], k[1]}, {k[2], 0, -k[0]}, {-k[1], k[0], 0}}};
  const Mat3 KK = mul(K, K);
  Mat3 R{};
  for (int r = 0; r < 3; ++r) {
    for (int col = 0; col < 3; ++col) {
      R[r][col] = (r == col ? 1.0 : 0.0) + s * K[r][col] + c * KK[r][col];
    }
  }
  return R;
}

} // namespace

std::array<double, 6> leftArmGravityHoldTorque(const std::array<double, 6>& q_urdf_rad) {
  // Forward kinematics in the base frame: joint positions, world axes, link COMs.
  std::array<Vec3, 6> joint_pos{};
  std::array<Vec3, 6> joint_axis{};
  std::array<Vec3, 6> com{};
  Mat3 R = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  Vec3 p = {0, 0, 0};
  for (size_t i = 0; i < kLeftArm.size(); ++i) {
    p = add(p, mul(R, kLeftArm[i].origin));
    joint_pos[i] = p;
    joint_axis[i] = mul(R, kLeftArm[i].axis);
    R = mul(R, axisAngle(kLeftArm[i].axis, q_urdf_rad[i]));
    com[i] = add(p, mul(R, kLeftArm[i].com));
  }

  // Gravity's moment about joint i from every link it carries; the joint must cancel it.
  std::array<double, 6> tau{};
  for (size_t i = 0; i < kLeftArm.size(); ++i) {
    Vec3 moment = {0, 0, 0};
    for (size_t j = i; j < kLeftArm.size(); ++j) {
      const Vec3 weight = {0, 0, -kLeftArm[j].mass * kGravity};
      moment = add(moment, cross(sub(com[j], joint_pos[i]), weight));
    }
    tau[i] = -dot(joint_axis[i], moment);
  }
  return tau;
}
