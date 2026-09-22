#ifndef EKF_AHRS_H
#define EKF_AHRS_H

#include "quaternion.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Error-state (multiplicative) EKF for 9-axis AHRS.
 *
 * State carried between calls:
 *   - q       : unit quaternion, body->nav attitude
 *   - bias    : gyro bias estimate (rad/s), body frame
 *
 * Kalman filter operates internally on a 6-element error state:
 *   [ delta_theta (3) ]  small-angle attitude error (rad)
 *   [ delta_bias  (3) ]  gyro bias error (rad/s)
 * which is reset to zero after every correction (standard MEKF pattern).
 *
 * Usage per control loop:
 *   ekf_ahrs_predict(&ekf, gyro, dt);        // every loop, using gyro (rad/s)
 *   ekf_ahrs_update_accel(&ekf, accel);      // whenever new accel sample (m/s^2 or g, any consistent unit)
 *   ekf_ahrs_update_mag(&ekf, mag);          // whenever new mag sample (any consistent unit)
 *   ekf_ahrs_get_euler(&ekf, &roll, &pitch, &yaw);
 *
 * Notes:
 *   - accel and mag vectors are normalized internally, so units don't need
 *     to match the reference vectors exactly, only directions matter.
 *   - Only use ekf_ahrs_update_accel() when the vehicle is close to static
 *     equilibrium (low linear acceleration) — otherwise it will fight
 *     against real motion and corrupt roll/pitch. A common trick is to
 *     gate the update on |accel_norm - 1g| being small, or to inflate
 *     R_accel_var when dynamic acceleration is high.
 *   - mag_ref must be calibrated for your location (or you can just use
 *     it purely for yaw disambiguation with a rough reference — see
 *     README for a simple startup calibration procedure).
 */

typedef struct {
    /* --- state --- */
    quat_t q;             /* attitude: body -> nav */
    float  bias[3];       /* gyro bias estimate, rad/s */

    /* --- covariance of the 6-dim error state [dtheta; dbias] --- */
    float P[6][6];

    /* --- tunable noise parameters (variances) --- */
    float gyro_noise_var;  /* gyro white noise variance, (rad/s)^2 */
    float gyro_bias_var;   /* gyro bias random-walk variance, (rad/s)^2 per second */
    float accel_noise_var; /* accel direction measurement noise variance */
    float mag_noise_var;   /* mag heading measurement noise variance, rad^2 */

    /* --- reference vectors in nav frame (unit vectors) --- */
    float accel_ref[3];    /* typically {0, 0, 1} = "up" */
    float mag_ref[3];      /* local magnetic field direction, from calibration */

    /* --- magnetic disturbance gating ---
     * Earth's field has a fixed magnitude and dip. A sample whose magnitude
     * or dip disagrees with the reference is being distorted (nearby iron,
     * unconverged hard-iron calibration), and its heading is wrong in a way
     * that depends on orientation -- it is rejected rather than trusted. */
    float mag_ref_norm;    /* reference |B| in raw units; 0 disables the magnitude gate */
    float mag_norm_tol;    /* reject if | |B|/mag_ref_norm - 1 | exceeds this */
    float mag_dip_tol;     /* reject if dip differs from the reference by more (rad) */
    unsigned long mag_rejects; /* diagnostic: samples rejected by the gates */
} ekf_ahrs_t;

/*
 * Initialize the filter.
 *   mag_ref_nav: unit vector of local magnetic field expressed in the NAV
 *                frame, NOT a raw magnetometer reading. A raw reading is in
 *                the body frame, and the two only coincide when the board
 *                happens to be at identity attitude. Passing the body vector
 *                by mistake biases heading by the angle between the two --
 *                see ekf_ahrs_set_mag_ref_from_body().
 * Sets q = identity, bias = 0, P = moderate initial uncertainty,
 * and fills in default noise parameters (tune these for your sensors).
 */
void ekf_ahrs_init(ekf_ahrs_t *ekf, const float mag_ref_nav[3]);

/*
 * Seed the attitude estimate, e.g. from an external fused quaternion at
 * startup so the filter doesn't have to converge from identity.
 * Call this BEFORE ekf_ahrs_set_mag_ref_from_body().
 */
void ekf_ahrs_set_attitude(ekf_ahrs_t *ekf, quat_t q);

/*
 * Startup magnetometer calibration: takes a raw BODY-frame magnetometer
 * sample and rotates it into the nav frame using the filter's CURRENT
 * attitude, storing the result as mag_ref. This is the correct way to
 * capture the local field when the board is not level/north-aligned at
 * startup.
 */
void ekf_ahrs_set_mag_ref_from_body(ekf_ahrs_t *ekf, const float mag_body[3]);

/* Prediction step: integrate gyro, propagate covariance. Call every loop. */
void ekf_ahrs_predict(ekf_ahrs_t *ekf, const float gyro[3], float dt);

/* Correction step using accelerometer (corrects roll/pitch). */
void ekf_ahrs_update_accel(ekf_ahrs_t *ekf, const float accel[3]);

/* Correction step using magnetometer. A scalar HEADING measurement -- see the
 * comment on the definition for why it must not be a 3-axis vector update. */
void ekf_ahrs_update_mag(ekf_ahrs_t *ekf, const float mag[3]);

/*
 * The magnetometer's opinion of the current heading error (radians, wrapped
 * to [-pi, pi]): how far the estimate must rotate about nav "up" to line the
 * measured field up with mag_ref. Returns 0 (and leaves *err untouched) when
 * the field is too close to vertical to define a heading.
 * Used by ekf_ahrs_update_mag(); also handy as a diagnostic.
 */
int ekf_ahrs_mag_heading_error(const ekf_ahrs_t *ekf, const float mag[3], float *err);

/* Convenience: current attitude estimate as Euler angles (radians). */
void ekf_ahrs_get_euler(const ekf_ahrs_t *ekf, float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif /* EKF_AHRS_H */
