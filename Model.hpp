#pragma once
// Model constants and the GPU-visible records shared by the shaders, the estimator, the CSV
// importer and the verification code. The std140 structs here mirror the uniform blocks in
// InverseShaders.hpp; the static_asserts below pin their layout.
#include "Math3D.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace inverse {
    using F4 = std::array<float, 4>;
    inline V3 xyz(const F4& f) {
        return { f[0], f[1], f[2] };
    }
    inline F4 vec4(V3 p, float w = 1) {
        return { p.x, p.y, p.z, w };
    }

    // ---- Spring network and sampling
    constexpr int nodes = 8, freeNodes = 6;  // nodes 6 and 7 are fixed anchors
    constexpr int springs = nodes * (nodes - 1) / 2; // every pair of nodes: 28
    constexpr int sampleRate = 60;           // samples per model second
    constexpr int minSamples = 8, maxSamples = 360;
    constexpr int allNodes = nodes;          // "displayed trajectories" value meaning every node

    // ---- Optimizer
    constexpr int historyCapacity = 256;     // recorded iterations (the plot shows up to 255 segments)
    constexpr int maxIterations = 100;
    constexpr int jacobianJobs = 7;          // current estimate + (-, +) perturbation per parameter
    constexpr int lineSearchJobs = 8;        // step scales 1, 1/2, ..., 1/128
    constexpr int seedJobs = 33;             // current estimate + 8 x 2 x 2 parameter grid
    constexpr float fdStep = .002f;          // central finite-difference step in log-parameter space
    constexpr int workgroupSize = 128;       // layout(local_size_x) of the per-element kernels

    constexpr int workgroups(int invocations) {
        return (invocations + workgroupSize - 1) / workgroupSize;
    }

    // ---- Render geometry produced by shaders::pack, in buffer order
    constexpr int bodyVertices = 2 * springs * 2;        // fitted and clean reference wireframes
    constexpr int residualVertices = nodes * 2;          // observation-to-fit segments at the playback sample
    constexpr int overlayVertices = bodyVertices + residualVertices;
    constexpr int plotVertices = (historyCapacity - 1) * 2;

    enum class Scene : int { clean, noisy, sparse, stiffnessOnly };
    enum class Mode : int { current, jacobian, lineSearch, seed }; // what shaders::prepare puts in candidates[]
    enum class FitStatus : int { ready, fitting, converged, stopped, uninformative };

    struct Settings {
        int samples = 240;            // 60 Hz samples including the initial state
        int substeps = 4;             // RK4 substeps per sample interval: 4 or 8
        Scene scene = Scene::clean;
        int node = allNodes;          // displayed observation trajectories
        float stiffness = 18, damping = .6f, gravity = 5.5f; // synthetic truth
        float noise = .012f;          // uniform noise half-width for the noisy presets
        bool seedSearch = true, trails = true, reference = true;

        int size() const {
            return samples * nodes;
        }
        int trajectoryVertices() const {
            return (samples - 1) * nodes * 2;
        }
        bool stiffnessOnly() const {
            return scene == Scene::stiffnessOnly;
        }
        V3 truth() const {
            return { stiffness, damping, gravity };
        }
    };

    /// 80-byte optimizer state (SSBO binding 0), written by the GPU and read back after each iteration.
    struct alignas(16) FitState {
        F4 parameters{};  // xyz: log(stiffness, damping, gravity), w: Levenberg-Marquardt lambda
        F4 statistics{};  // x: weighted MSE, y: Σ weights, z: iterations, w: accepted steps
        F4 proposal{};    // xyz: proposed log-space step, w: accepted step scale
        F4 diagnostics{}; // x: normalized Gram determinant, y: |step|, z: FitStatus, w: seed search done
        F4 quality{};     // x: RMSE, y: clean-reference RMSE or -1, z: peak residual, w: invalid records

        V3 physical() const {
            return { std::exp(parameters[0]), std::exp(parameters[1]), std::exp(parameters[2]) };
        }
        float lambda() const {
            return parameters[3];
        }
        float mse() const {
            return statistics[0];
        }
        float weightSum() const {
            return statistics[1];
        }
        float iterations() const {
            return statistics[2];
        }
        float gramDeterminant() const {
            return diagnostics[0];
        }
        float stepNorm() const {
            return diagnostics[1];
        }
        FitStatus status() const {
            return static_cast<FitStatus>(static_cast<int>(diagnostics[2]));
        }
        float rmse() const {
            return quality[0];
        }
        float referenceRmse() const {
            return quality[1];
        }
        float invalid() const {
            return quality[3];
        }
    };

    /// Line/point vertex produced by shaders::pack and the static floor/plot grids.
    struct alignas(16) Vertex {
        F4 p{}, color{};
    };

    /// std140 mirror of shaders::paramsBlock (uniform binding 0).
    struct alignas(16) Params {
        std::int32_t samples, substeps, mode, jobs;
        std::int32_t scene, stiffnessOnly, hasReference, simulateTruth;
        std::int32_t playback, selectedNode, showTrails, showReference;
        V3 truth;
        float noise;
        float fdStep;
        float padding[3];
    };

    /// std140 mirror of shaders::viewBlock (uniform binding 1).
    struct alignas(16) ViewParams {
        M4 view, projection;
        std::int32_t selectedNode;
        std::int32_t padding[3];
    };

    static_assert(sizeof(FitState) == 80 && sizeof(Vertex) == 32);
    static_assert(sizeof(Params) == 80 && offsetof(Params, truth) == 48 && offsetof(Params, noise) == 60 &&
        offsetof(Params, fdStep) == 64);
    static_assert(sizeof(ViewParams) == 144 && offsetof(ViewParams, selectedNode) == 128);

    /// Observations in sample-major order: index = sample * nodes + node; xyz position, w weight.
    struct Dataset {
        int samples = 0;
        std::vector<F4> observations;
    };

    /// Window-space rectangle, in pixels from the bottom-left corner as OpenGL counts them.
    struct Viewport {
        int x, y, width, height;
    };
}
