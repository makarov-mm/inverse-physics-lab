#pragma once
// Small vector/matrix types and the orbiting perspective camera used by the renderer and the brush.
#include <algorithm>
#include <array>
#include <cmath>

namespace inverse {
    constexpr float fieldOfView = .65f; // vertical, radians; shared with the ray-marching shader
    constexpr float maxPitch = 1.45f;   // orbit limit, radians

    struct V3 {
        float x{}, y{}, z{};
        constexpr V3 operator+(V3 v) const {
            return { x + v.x, y + v.y, z + v.z };
        }
        constexpr V3 operator-(V3 v) const {
            return { x - v.x, y - v.y, z - v.z };
        }
        constexpr V3 operator-() const {
            return { -x, -y, -z };
        }
        constexpr V3 operator*(float s) const {
            return { x * s, y * s, z * s };
        }
        constexpr float& operator[](int axis) {
            return axis == 0 ? x : (axis == 1 ? y : z);
        }
        constexpr float operator[](int axis) const {
            return axis == 0 ? x : (axis == 1 ? y : z);
        }
        constexpr float dot(V3 v) const {
            return x * v.x + y * v.y + z * v.z;
        }
        constexpr V3 cross(V3 v) const {
            return { y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x };
        }
        float length() const {
            return std::sqrt(dot(*this));
        }
        V3 normalized() const {
            return *this * (1 / std::max(length(), 1e-9f));
        }
    };

    // Column-major 4x4 matrix, laid out as OpenGL and std140 expect it.
    struct M4 {
        std::array<float, 16> v{};
    };

    inline M4 perspective(float aspect) {
        const float f = 1 / std::tan(fieldOfView / 2), clipNear = .025f, clipFar = 10000;
        return { { { f / aspect, 0, 0, 0,
                  0, f, 0, 0,
                  0, 0, (clipFar + clipNear) / (clipNear - clipFar), -1,
                  0, 0, 2 * clipFar * clipNear / (clipNear - clipFar), 0 } } };
    }

    // Orbits the target at a fixed distance; the view direction is derived from yaw and pitch.
    struct Camera {
        float yaw = .65f, pitch = .65f, distance = 2.4f;
        V3 target{};

        V3 eye() const {
            return target + V3{ std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch) } *
                distance;
        }

        V3 forward() const {
            return (target - eye()).normalized();
        }

        V3 right() const {
            return forward().cross({ 0, 1, 0 }).normalized();
        }

        V3 up() const {
            return right().cross(forward());
        }

        M4 view() const {
            const V3 r = right(), u = up(), f = forward(), e = eye();

            return { { { r.x, u.x, -f.x, 0,
                      r.y, u.y, -f.y, 0,
                      r.z, u.z, -f.z, 0,
                      -r.dot(e), -u.dot(e), f.dot(e), 1 } } };
        }

        // World-space direction through normalized device coordinates (nx, ny in [-1, 1]).
        V3 ray(float nx, float ny, float aspect) const {
            const float t = std::tan(fieldOfView / 2);
            return (forward() + right() * (nx * t * aspect) + up() * (ny * t)).normalized();
        }
    };
}
