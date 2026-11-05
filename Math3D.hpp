#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
namespace inverse {
struct V3 {
    float x{}, y{}, z{};
    V3 operator+(V3 v) const {
        return {x + v.x, y + v.y, z + v.z};
    }
    V3 operator-(V3 v) const {
        return {x - v.x, y - v.y, z - v.z};
    }
    V3 operator*(float s) const {
        return {x * s, y * s, z * s};
    }
    float dot(V3 v) const {
        return x * v.x + y * v.y + z * v.z;
    }
    V3 cross(V3 v) const {
        return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
    }
    float length() const {
        return std::sqrt(dot(*this));
    }
    V3 normalized() const {
        return *this * (1 / std::max(length(), 1e-9f));
    }
};
struct M4 {
    std::array<float, 16> v{};
};
inline float dot(V3 a, V3 b) {
    return a.dot(b);
}
inline V3 cross(V3 a, V3 b) {
    return a.cross(b);
}
inline M4 perspective(float aspect) {
    const float f = 1 / std::tan(.65f * .5f), clipNear = .025f, clipFar = 10000;
    return {{{f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (clipFar + clipNear) / (clipNear - clipFar), -1, 0, 0,
              2 * clipFar * clipNear / (clipNear - clipFar), 0}}};
}
struct Camera {
    float yaw = .65f, pitch = .65f, distance = 3.8f;
    V3 target{0, .45f, 0};
    V3 eye() const {
        return target +
               V3{std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)} *
                   distance;
    }
    V3 forward() const {
        return (target - eye()).normalized();
    }
    V3 right() const {
        return forward().cross({0, 1, 0}).normalized();
    }
    V3 up() const {
        return right().cross(forward());
    }
    M4 view() const {
        auto r = right(), u = up(), f = forward(), e = eye();
        return {
            {{r.x, u.x, -f.x, 0, r.y, u.y, -f.y, 0, r.z, u.z, -f.z, 0, -r.dot(e), -u.dot(e), f.dot(e), 1}}};
    }
    V3 ray(float nx, float ny, float aspect) const {
        float t = std::tan(.65f * .5f);
        return (forward() + right() * (nx * t * aspect) + up() * (ny * t)).normalized();
    }
};
} // namespace inverse
