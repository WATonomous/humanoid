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
    float mag_noise_var;   /* mag direction measurement noise variance */

    /* --- reference vectors in nav frame (unit vectors) --- */
    float accel_ref[3];    /* typically {0, 0, 1} = "up" */
    float mag_ref[3];      /* local magnetic field direction, from calibration */
} ekf_ahrs_t;

/*
 * Initialize the filter.
 *   mag_ref_nav: unit vector of local magnetic field in nav frame
 *                (e.g. from a startup calibration routine).
 * Sets q = identity, bias = 0, P = moderate initial uncertainty,
 * and fills in default noise parameters (tune these for your sensors).
 */
void ekf_ahrs_init(ekf_ahrs_t *ekf, const float mag_ref_nav[3]);

/* Prediction step: integrate gyro, propagate covariance. Call every loop. */
void ekf_ahrs_predict(ekf_ahrs_t *ekf, const float gyro[3], float dt);

/* Correction step using accelerometer (corrects roll/pitch). */
void ekf_ahrs_update_accel(ekf_ahrs_t *ekf, const float accel[3]);

/* Correction step using magnetometer (corrects yaw). */
void ekf_ahrs_update_mag(ekf_ahrs_t *ekf, const float mag[3]);

/* Convenience: current attitude estimate as Euler angles (radians). */
void ekf_ahrs_get_euler(const ekf_ahrs_t *ekf, float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif /* EKF_AHRS_H */
