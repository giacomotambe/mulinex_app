#pragma once

#include <array>
#include <cmath>

inline std::array<double, 4> quat_mult(std::array<double, 4> q, std::array<double, 4> p)
{
    std::array<double, 4> r;
    r[0] = q[0] * p[3] + q[1] * p[2] - q[2] * p[1] + q[3] * p[0];
    r[1] = -q[0] * p[2] + q[1] * p[3] + q[2] * p[0] + q[3] * p[1];
    r[2] = q[0] * p[1] - q[1] * p[0] + q[2] * p[3] + q[3] * p[2];
    r[3] = -q[0] * p[0] - q[1] * p[1] - q[2] * p[2] + q[3] * p[3];
    return r;
}

inline std::array<double, 4> quat_exp_vec(std::array<double, 3> v)
{
    std::array<double, 4> q;
    double norm_v = 0.0;
    for (int i = 0; i < 3; i++)
        norm_v += v[i] * v[i];
    norm_v = std::sqrt(norm_v);
    if (norm_v == 0.0) {
        q[0] = q[1] = q[2] = 0.0;
    } else {
        for (int i = 0; i < 3; i++)
            q[i] = v[i] / norm_v * std::sin(norm_v);
    }
    q[3] = std::cos(norm_v);
    return q;
}

inline void normalize_quat(std::array<double, 4>& q)
{
    double norm = 0;
    for (int i = 0; i < 4; i++)
        norm += q[i] * q[i];
    norm = std::sqrt(norm);
    for (int i = 0; i < 4; i++)
        q[i] = q[i] / norm;
}

inline std::array<double, 4> quat_int(std::array<double, 4> q, std::array<double, 3> w_b, double dt)
{
    std::array<double, 4> q_out;
    for (int i = 0; i < 3; i++)
        w_b[i] = 0.5 * dt * w_b[i];
    q_out = quat_mult(q, quat_exp_vec(w_b));
    normalize_quat(q_out);
    return q_out;
}
