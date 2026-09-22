#include "ekf_ahrs.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------- small fixed-size matrix helpers (no malloc) ---------- */

static void mat6x6_mult(const float A[6][6], const float B[6][6], float C[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
    }
}

static void mat6x6_transpose(const float A[6][6], float At[6][6]) {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            At[i][j] = A[j][i];
}

static void mat6x6_add(const float A[6][6], const float B[6][6], float C[6][6]) {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            C[i][j] = A[i][j] + B[i][j];
}

/* 3x3 inverse via adjugate/determinant (fine for well-conditioned S) */
static int mat3_inverse(const float M[3][3], float Minv[3][3]) {
    float det =
        M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
        M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
        M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

    if (fabsf(det) < 1e-12f) return 0; /* singular, bail out */
    float inv_det = 1.0f / det;

    Minv[0][0] =  (M[1][1] * M[2][2] - M[1][2] * M[2][1]) * inv_det;
    Minv[0][1] = -(M[0][1] * M[2][2] - M[0][2] * M[2][1]) * inv_det;
    Minv[0][2] =  (M[0][1] * M[1][2] - M[0][2] * M[1][1]) * inv_det;

    Minv[1][0] = -(M[1][0] * M[2][2] - M[1][2] * M[2][0]) * inv_det;
    Minv[1][1] =  (M[0][0] * M[2][2] - M[0][2] * M[2][0]) * inv_det;
    Minv[1][2] = -(M[0][0] * M[1][2] - M[0][2] * M[1][0]) * inv_det;

    Minv[2][0] =  (M[1][0] * M[2][1] - M[1][1] * M[2][0]) * inv_det;
    Minv[2][1] = -(M[0][0] * M[2][1] - M[0][1] * M[2][0]) * inv_det;
    Minv[2][2] =  (M[0][0] * M[1][1] - M[0][1] * M[1][0]) * inv_det;

    return 1;
}

/* ---------- init ---------- */

void ekf_ahrs_init(ekf_ahrs_t *ekf, const float mag_ref_nav[3]) {
    

    memset(ekf, 0, sizeof(*ekf));
    ekf->q = quat_identity();

    for (int i = 0; i < 6; i++) {
        ekf->P[i][i] = (i < 3) ? 0.05f   /* initial attitude uncertainty, rad^2 */
                               : 7.6e-7f; /* initial gyro bias uncertainty, (rad/s)^2 = (0.05 deg/s)^2 */
    }

    ekf->gyro_noise_var  = 3e-4f;  /* gyro white noise, (rad/s)^2 -- from datasheet or Allan variance */
    ekf->gyro_bias_var   = 1e-11f; /* gyro bias random walk, (rad/s)^2/s */
    ekf->accel_noise_var = 5e-2f;  /* accel direction noise -- raise this if vehicle moves/vibrates a lot */
    ekf->mag_noise_var   = 7.6e-3f; /* mag HEADING noise, rad^2 = (5 deg)^2 -- raise near magnetic interference */

    ekf->accel_ref[0] = 0.0f;
    ekf->accel_ref[1] = 0.0f;
    ekf->accel_ref[2] = 1.0f; /* "up" in nav frame */

    ekf->mag_ref[0] = mag_ref_nav[0];
    ekf->mag_ref[1] = mag_ref_nav[1];
    ekf->mag_ref[2] = mag_ref_nav[2];

    vec3_normalize(ekf->mag_ref);

    ekf->mag_ref_norm = 0.0f;              /* unknown until set_mag_ref_from_body() */
    ekf->mag_norm_tol = 0.15f;             /* +/-15% field magnitude */
    ekf->mag_dip_tol  = 10.0f * (float)M_PI / 180.0f;
}

void ekf_ahrs_set_attitude(ekf_ahrs_t *ekf, quat_t q) {
    ekf->q = quat_normalize(q);
}

void ekf_ahrs_set_mag_ref_from_body(ekf_ahrs_t *ekf, const float mag_body[3]) {
    float m[3] = {mag_body[0], mag_body[1], mag_body[2]};

    ekf->mag_ref_norm = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);

    vec3_normalize(m);

    /* mag_ref lives in the nav frame, so the body sample has to be rotated. */
    float R[3][3];
    quat_to_rotmat(ekf->q, R);
    mat3_vec_mult(R, m, ekf->mag_ref);
    vec3_normalize(ekf->mag_ref);
}

/* ---------- predict ---------- */

void ekf_ahrs_predict(ekf_ahrs_t *ekf, const float gyro[3], float dt) {
    if (dt <= 0.0f) return;

    float w[3] = {
        gyro[0] - ekf->bias[0],
        gyro[1] - ekf->bias[1],
        gyro[2] - ekf->bias[2]
    };

    /* --- propagate attitude exactly (exponential map), not just linearized --- */
    quat_t dq = quat_from_gyro_delta(w, dt);
    ekf->q = quat_normalize(quat_mult(ekf->q, dq));

    /* --- propagate error-state covariance ---
     * F = [ I - skew(w)*dt   -I*dt ]
     *     [ 0                 I    ]
     * (first-order discretization; fine for small dt e.g. <= 10-20 ms)
     */
    float Sw[3][3];
    skew3(w, Sw);

    float F[6][6] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F[i][j] = -Sw[i][j] * dt;
        }
        F[i][i] += 1.0f;
        F[i][i + 3] = -dt;
        F[i + 3][i + 3] = 1.0f;
    }

    float Q[6][6] = {0};
    for (int i = 0; i < 3; i++) {
        Q[i][i]         = ekf->gyro_noise_var * dt * dt;
        Q[i + 3][i + 3] = ekf->gyro_bias_var * dt;
    }

    float Ft[6][6], FP[6][6], FPFt[6][6], Pnew[6][6];
    mat6x6_transpose(F, Ft);
    mat6x6_mult(F, ekf->P, FP);
    mat6x6_mult(FP, Ft, FPFt);
    mat6x6_add(FPFt, Q, Pnew);
    memcpy(ekf->P, Pnew, sizeof(Pnew));
}

/* ---------- generic 3-dof vector-observation correction ----------
 * Shared logic for accel and mag updates: both are "measure a known
 * nav-frame reference direction, rotated into the body frame" updates,
 * differing only in the reference vector and measurement noise used.
 */
static void vector_update(ekf_ahrs_t *ekf, const float meas_body_in[3],
                           const float ref_nav[3], float meas_noise_var) {
    float meas_body[3] = {meas_body_in[0], meas_body_in[1], meas_body_in[2]};
    vec3_normalize(meas_body);

    float R[3][3], Rt[3][3];
    quat_to_rotmat(ekf->q, R);
    mat3_transpose(R, Rt);

    /* predicted body-frame direction of the reference vector */
    float h[3];
    mat3_vec_mult(Rt, ref_nav, h);
    vec3_normalize(h);

    /* innovation */
    float y[3] = {
        meas_body[0] - h[0],
        meas_body[1] - h[1],
        meas_body[2] - h[2]
    };

    /* H = [ skew(h), 0 ]  (3x6) -- Jacobian of predicted measurement
     * w.r.t. the attitude-error part of the state */
    float Sh[3][3];
    skew3(h, Sh);
    float H[3][6] = {0};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            H[i][j] = Sh[i][j];

    /* S = H P H^T + R  (3x3) */
    float HP[3][6] = {0};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 6; j++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += H[i][k] * ekf->P[k][j];
            HP[i][j] = s;
        }

    float S[3][3] = {0};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += HP[i][k] * H[j][k]; /* H^T */
            S[i][j] = s;
        }
    S[0][0] += meas_noise_var;
    S[1][1] += meas_noise_var;
    S[2][2] += meas_noise_var;

    float Sinv[3][3];
    if (!mat3_inverse(S, Sinv)) return; /* skip update if S is singular */

    /* K = P H^T S^-1  (6x3) */
    float PHt[6][3] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) s += ekf->P[i][k] * H[j][k]; /* H^T */
            PHt[i][j] = s;
        }

    float K[6][3] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += PHt[i][k] * Sinv[k][j];
            K[i][j] = s;
        }

    /* error-state correction dx = K * y  (6x1) */
    float dx[6] = {0};
    for (int i = 0; i < 6; i++) {
        float s = 0.0f;
        for (int k = 0; k < 3; k++) s += K[i][k] * y[k];
        dx[i] = s;
    }

    /* apply attitude correction, reset bias, reset attitude error to 0 */
    float da[3] = {dx[0], dx[1], dx[2]};
    quat_t dq = quat_from_small_angle(da);
    ekf->q = quat_normalize(quat_mult(ekf->q, dq));

    ekf->bias[0] += dx[3];
    ekf->bias[1] += dx[4];
    ekf->bias[2] += dx[5];

    /* covariance update, Joseph form for numerical stability:
     * P = (I - K H) P (I - K H)^T + K R K^T
     */
    float KH[6][6] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += K[i][k] * H[k][j];
            KH[i][j] = s;
        }

    float IKH[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            IKH[i][j] = ((i == j) ? 1.0f : 0.0f) - KH[i][j];

    float IKHt[6][6];
    mat6x6_transpose(IKH, IKHt);

    float term1[6][6], term1b[6][6];
    mat6x6_mult(IKH, ekf->P, term1);
    mat6x6_mult(term1, IKHt, term1b);

    float KRKt[6][6] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float s = 0.0f;
            /* R = meas_noise_var * I, so K R K^T reduces to a scaled K K^T */
            for (int k = 0; k < 3; k++)
                s += K[i][k] * meas_noise_var * K[j][k];
            KRKt[i][j] = s;
        }

    float Pnew[6][6];
    mat6x6_add(term1b, KRKt, Pnew);
    memcpy(ekf->P, Pnew, sizeof(Pnew));
}

void ekf_ahrs_update_accel(ekf_ahrs_t *ekf, const float accel[3]) {
    vector_update(ekf, accel, ekf->accel_ref, ekf->accel_noise_var);
}

/* ---------- generic scalar correction ----------
 * y is a scalar innovation, H a 1x6 Jacobian row w.r.t. the error state,
 * r the scalar measurement noise variance.
 */
static void scalar_update(ekf_ahrs_t *ekf, const float H[6], float y, float r) {
    /* PHt = P H^T (6x1),  S = H P H^T + r (scalar) */
    float PHt[6];
    for (int i = 0; i < 6; i++) {
        float s = 0.0f;
        for (int k = 0; k < 6; k++) s += ekf->P[i][k] * H[k];
        PHt[i] = s;
    }

    float S = r;
    for (int k = 0; k < 6; k++) S += H[k] * PHt[k];
    if (S < 1e-12f) return;

    float K[6];
    for (int i = 0; i < 6; i++) K[i] = PHt[i] / S;

    /* apply correction, then reset the attitude error to 0 (MEKF) */
    float da[3] = {K[0] * y, K[1] * y, K[2] * y};
    quat_t dq = quat_from_small_angle(da);
    ekf->q = quat_normalize(quat_mult(ekf->q, dq));

    ekf->bias[0] += K[3] * y;
    ekf->bias[1] += K[4] * y;
    ekf->bias[2] += K[5] * y;

    /* Joseph form: P = (I - K H) P (I - K H)^T + r K K^T */
    float IKH[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            IKH[i][j] = ((i == j) ? 1.0f : 0.0f) - K[i] * H[j];

    float IKHt[6][6], tmp[6][6], Pnew[6][6];
    mat6x6_transpose(IKH, IKHt);
    mat6x6_mult(IKH, ekf->P, tmp);
    mat6x6_mult(tmp, IKHt, Pnew);

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            Pnew[i][j] += r * K[i] * K[j];

    memcpy(ekf->P, Pnew, sizeof(Pnew));
}

int ekf_ahrs_mag_heading_error(const ekf_ahrs_t *ekf, const float mag[3], float *err) {
    float m[3] = {mag[0], mag[1], mag[2]};
    vec3_normalize(m);

    float R[3][3];
    quat_to_rotmat(ekf->q, R);

    float m_nav[3];
    mat3_vec_mult(R, m, m_nav);

    /* A field that is (near) vertical in either frame has no defined
     * horizontal direction, so it carries no heading information. */
    float m_horiz = sqrtf(m_nav[0] * m_nav[0] + m_nav[1] * m_nav[1]);
    float ref_horiz = sqrtf(ekf->mag_ref[0] * ekf->mag_ref[0] +
                            ekf->mag_ref[1] * ekf->mag_ref[1]);
    if (m_horiz < 0.1f || ref_horiz < 0.1f) return 0;

    /* If the true attitude is our estimate rotated by d_psi about nav z, the
     * field seen through our estimate appears rotated by -d_psi. So the
     * heading error is the reference azimuth minus the measured azimuth. */
    float e = atan2f(ekf->mag_ref[1], ekf->mag_ref[0]) - atan2f(m_nav[1], m_nav[0]);
    while (e >  (float)M_PI) e -= 2.0f * (float)M_PI;
    while (e < -(float)M_PI) e += 2.0f * (float)M_PI;

    *err = e;
    return 1;
}

/*
 * Magnetometer correction -- HEADING ONLY, as a scalar measurement.
 *
 * Measuring heading as an angle, with H = d(nav yaw)/d(error state), makes
 * the magnetometer physically unable to touch roll or pitch.
 */
void ekf_ahrs_update_mag(ekf_ahrs_t *ekf, const float mag[3]) {
    float norm = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
    if (norm < 1e-6f) return;

    /* Magnitude gate: Earth's field strength doesn't change as you rotate. */
    if (ekf->mag_ref_norm > 0.0f &&
        fabsf(norm / ekf->mag_ref_norm - 1.0f) > ekf->mag_norm_tol) {
        ekf->mag_rejects++;
        return;
    }

    float R[3][3];
    quat_to_rotmat(ekf->q, R);

    /* Dip gate: nor does its inclination. Roll/pitch come from the
     * accelerometer and are accurate, so the measured field's vertical
     * component in the nav frame is a trustworthy disturbance check. */
    float m[3] = {mag[0] / norm, mag[1] / norm, mag[2] / norm};
    float m_nav[3];
    mat3_vec_mult(R, m, m_nav);

    float dip_meas = asinf(fmaxf(-1.0f, fminf(1.0f, m_nav[2])));
    float dip_ref  = asinf(fmaxf(-1.0f, fminf(1.0f, ekf->mag_ref[2])));
    if (fabsf(dip_meas - dip_ref) > ekf->mag_dip_tol) {
        ekf->mag_rejects++;
        return;
    }

    float y;
    if (!ekf_ahrs_mag_heading_error(ekf, mag, &y)) return;

    /* q <- q (x) dq(da) rotates by R*da in the nav frame, so the nav yaw
     * component of a body-frame error da is the third row of R dotted with
     * it. Bias does not enter the measurement directly. */

    float H[6] = {R[2][0], R[2][1], R[2][2], 0.0f, 0.0f, 0.0f};

    scalar_update(ekf, H, y, ekf->mag_noise_var);
}

void ekf_ahrs_get_euler(const ekf_ahrs_t *ekf, float *roll, float *pitch, float *yaw) {
    quat_to_euler(ekf->q, roll, pitch, yaw);
}
