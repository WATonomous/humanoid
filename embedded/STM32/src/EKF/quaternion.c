#include "quaternion.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

quat_t quat_identity(void) {
    quat_t q = {1.0f, 0.0f, 0.0f, 0.0f};
    return q;
}

quat_t quat_normalize(quat_t q) {
    float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-12f) {
        return quat_identity();
    }
    float inv = 1.0f / n;
    q.w *= inv; q.x *= inv; q.y *= inv; q.z *= inv;
    return q;
}

quat_t quat_mult(quat_t a, quat_t b) {
    quat_t r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}

quat_t quat_from_gyro_delta(const float w[3], float dt) {
    float angle = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * dt;
    quat_t dq;
    if (angle < 1e-8f) {
        /* Avoid division by ~0; falls back to small-angle form */
        dq.w = 1.0f;
        dq.x = 0.5f * w[0] * dt;
        dq.y = 0.5f * w[1] * dt;
        dq.z = 0.5f * w[2] * dt;
        return quat_normalize(dq);
    }
    float half = 0.5f * angle;
    float s = sinf(half) / angle; /* sin(half)/angle, not /half — matches w*dt scaling */
    dq.w = cosf(half);
    dq.x = w[0] * dt * s;
    dq.y = w[1] * dt * s;
    dq.z = w[2] * dt * s;
    return quat_normalize(dq);
}

quat_t quat_from_small_angle(const float da[3]) {
    quat_t dq;
    dq.w = 1.0f;
    dq.x = 0.5f * da[0];
    dq.y = 0.5f * da[1];
    dq.z = 0.5f * da[2];
    return quat_normalize(dq);
}

void quat_to_rotmat(quat_t q, float R[3][3]) {
    float w = q.w, x = q.x, y = q.y, z = q.z;
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;

    R[0][0] = 1.0f - 2.0f * (yy + zz);
    R[0][1] = 2.0f * (xy - wz);
    R[0][2] = 2.0f * (xz + wy);

    R[1][0] = 2.0f * (xy + wz);
    R[1][1] = 1.0f - 2.0f * (xx + zz);
    R[1][2] = 2.0f * (yz - wx);

    R[2][0] = 2.0f * (xz - wy);
    R[2][1] = 2.0f * (yz + wx);
    R[2][2] = 1.0f - 2.0f * (xx + yy);
}

void mat3_transpose(const float R[3][3], float Rt[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Rt[i][j] = R[j][i];
        }
    }
}

void mat3_vec_mult(const float M[3][3], const float v[3], float out[3]) {
    for (int i = 0; i < 3; i++) {
        out[i] = M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2];
    }
}

void vec3_normalize(float v[3]) {
    float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n < 1e-9f) return;
    float inv = 1.0f / n;
    v[0] *= inv; v[1] *= inv; v[2] *= inv;
}

void skew3(const float v[3], float S[3][3]) {
    S[0][0] =  0.0f;  S[0][1] = -v[2];  S[0][2] =  v[1];
    S[1][0] =  v[2];  S[1][1] =  0.0f;  S[1][2] = -v[0];
    S[2][0] = -v[1];  S[2][1] =  v[0];  S[2][2] =  0.0f;
}

void quat_to_euler(quat_t q, float *roll, float *pitch, float *yaw) {
    /* Z-Y-X convention (yaw-pitch-roll), standard aerospace sequence */
    float w = q.w, x = q.x, y = q.y, z = q.z;

    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (w * y - z * x);
    if (sinp >= 1.0f) {
        *pitch = (float)M_PI / 2.0f;
    } else if (sinp <= -1.0f) {
        *pitch = -(float)M_PI / 2.0f;
    } else {
        *pitch = asinf(sinp);
    }

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}
