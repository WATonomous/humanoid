#ifndef QUATERNION_H
#define QUATERNION_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Minimal quaternion + small vector/matrix helpers for embedded use.
 * Convention:
 *   - Quaternion q = (w, x, y, z), scalar-first, unit norm.
 *   - q represents the rotation from BODY frame to NAV frame:
 *         v_nav = R(q) * v_body
 *     So R(q)^T rotates a nav-frame vector into the body frame, which is
 *     what we need to predict accelerometer/magnetometer readings.
 *   - NAV frame here is just "world frame, Z up". Adjust reference
 *     vectors (accel_ref / mag_ref) if you use NED instead.
 */

typedef struct {
    float w, x, y, z;
} quat_t;

/* Identity quaternion (no rotation) */
quat_t quat_identity(void);

/* Normalize to unit length (safe against near-zero norm) */
quat_t quat_normalize(quat_t q);

/* Hamilton product: a "applied after" b, i.e. result = a ⊗ b */
quat_t quat_mult(quat_t a, quat_t b);

/*
 * Build the incremental rotation quaternion for a body-frame angular rate
 * `w` (rad/s) integrated over `dt` seconds, using the exact exponential
 * map (accurate even for larger dt / faster rotations, not just a
 * first-order small-angle approximation).
 */
quat_t quat_from_gyro_delta(const float w[3], float dt);

/*
 * Build a quaternion from a small rotation vector (attitude error
 * correction from the Kalman filter). Uses the first-order approximation
 * q = [1, 0.5*da] which is valid because da is expected to be small
 * after each correction step.
 */
quat_t quat_from_small_angle(const float da[3]);

/* Rotation matrix R(q) such that v_nav = R * v_body */
void quat_to_rotmat(quat_t q, float R[3][3]);

/* R transpose (nav -> body direction) */
void mat3_transpose(const float R[3][3], float Rt[3][3]);

/* out = M * v  (3x3 * 3x1) */
void mat3_vec_mult(const float M[3][3], const float v[3], float out[3]);

/* Normalize a 3-vector in place. No-op if norm is ~0. */
void vec3_normalize(float v[3]);

/* Skew-symmetric ("cross-product") matrix of v, such that S*x == v cross x */
void skew3(const float v[3], float S[3][3]);

/* Convert quaternion to roll/pitch/yaw (radians), Z-Y-X (yaw-pitch-roll) */
void quat_to_euler(quat_t q, float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif /* QUATERNION_H */
