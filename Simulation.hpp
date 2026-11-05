#pragma once
// GPU-resident parameter estimation: trial trajectories, Jacobian, normal equations, the damped
// solve and the step acceptance all run in compute shaders. The CPU schedules dispatches and reads
// back the 80-byte FitState after each iteration.
#include "GpuApi.hpp"
#include "InverseShaders.hpp"
#include "Math3D.hpp"
#include "Model.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace inverse {
    class Simulation {
    public:
        Settings settings;
        FitState state{};
        std::uint64_t playback = 0; // ticks since reset; drives the trajectory playback
        float gpuMs = 0;            // most recently completed timing of one optimizer iteration

        explicit Simulation(GpuApi& api) : gpu(api) {
            auto kernel = [&](std::string_view body, const char* label) {
                return gpu.program({ { gl::COMPUTE_SHADER,
                                     { shaders::version, shaders::computePrelude, shaders::paramsBlock,
                                      shaders::computeHelpers, body } } },
                    label);
                };
            programs.initialize = kernel(shaders::initialize, "Initialize parameter fit");
            programs.prepare = kernel(shaders::prepare, "Prepare log-space candidates");
            programs.forward = kernel(shaders::forward, "3D spring network RK4");
            programs.observe = kernel(shaders::observe, "Generate weighted observations");
            programs.cost = kernel(shaders::cost, "Parallel candidate objectives");
            programs.normals = kernel(shaders::normals, "Finite-difference normal equations");
            programs.solve = kernel(shaders::solve, "Damped 3x3 solve");
            programs.choose = kernel(shaders::choose, "Monotone candidate acceptance");
            programs.diagnostics = kernel(shaders::diagnostics, "Fit diagnostics");
            programs.pack = kernel(shaders::pack, "GPU render geometry");
            programs.lines = gpu.program({ { gl::VERTEX_SHADER,{ shaders::version, shaders::viewBlock, shaders::lineVertex } },
                                          { gl::FRAGMENT_SHADER,{ shaders::version, shaders::lineFragment } } },
                "Spring and history renderer");
            programs.points =
                gpu.program({ { gl::VERTEX_SHADER,{ shaders::version, shaders::viewBlock, shaders::pointVertex } },
                             { gl::FRAGMENT_SHADER,{ shaders::version, shaders::pointFragment } } },
                    "Observation and body renderer");
            lineVariant = gpu.GetUniformLocation(programs.lines, "variant");
            pointVariant = gpu.GetUniformLocation(programs.points, "variant");

            buffers.control = gpu.buffer(gl::SHADER_STORAGE_BUFFER, 0, sizeof(FitState), gl::DYNAMIC_COPY);
            buffers.trajectories = gpu.buffer(gl::SHADER_STORAGE_BUFFER, 1, sizeof(F4), gl::DYNAMIC_COPY);
            buffers.observations = gpu.buffer(gl::SHADER_STORAGE_BUFFER, 2, sizeof(F4), gl::DYNAMIC_COPY);
            buffers.reference = gpu.buffer(gl::SHADER_STORAGE_BUFFER, 3, sizeof(F4), gl::DYNAMIC_COPY);
            buffers.equations = gpu.buffer(gl::SHADER_STORAGE_BUFFER, 4, sizeof(F4), gl::DYNAMIC_COPY);
            buffers.candidates = gpu.buffer(gl::SHADER_STORAGE_BUFFER, 5, seedJobs * sizeof(F4), gl::DYNAMIC_COPY);
            buffers.history = gpu.buffer(gl::SHADER_STORAGE_BUFFER, 6, historyCapacity * sizeof(F4), gl::DYNAMIC_COPY);
            buffers.geometry = gpu.buffer(gl::SHADER_STORAGE_BUFFER, 7, sizeof(Vertex), gl::DYNAMIC_COPY);
            buffers.params = gpu.buffer(gl::UNIFORM_BUFFER, 0, sizeof(Params), gl::DYNAMIC_DRAW);
            buffers.view = gpu.buffer(gl::UNIFORM_BUFFER, 1, sizeof(ViewParams), gl::DYNAMIC_DRAW);
            gpu.GenBuffers(1, &buffers.grid);
            gpu.GenQueries(1, &query);
            for (GLuint* vao : { &vaos.geometry, &vaos.observations, &vaos.trajectories, &vaos.grid })
                gpu.GenVertexArrays(1, vao);
            gpu.BindVertexArray(vaos.geometry);
            attribute(0, buffers.geometry, sizeof(Vertex), offsetof(Vertex, p));
            attribute(1, buffers.geometry, sizeof(Vertex), offsetof(Vertex, color));
            gpu.BindVertexArray(vaos.observations);
            attribute(0, buffers.observations, sizeof(F4));
            gpu.BindVertexArray(vaos.trajectories);
            attribute(0, buffers.trajectories, sizeof(F4));
            gpu.BindVertexArray(vaos.grid);
            attribute(0, buffers.grid, sizeof(Vertex), offsetof(Vertex, p));
            attribute(1, buffers.grid, sizeof(Vertex), offsetof(Vertex, color));
            buildGrids();
            reset();
        }

        ~Simulation() {
            gpu.DeleteQueries(1, &query);

            for (GLuint vao : {vaos.geometry, vaos.observations, vaos.trajectories, vaos.grid})
                gpu.DeleteVertexArrays(1, &vao);

            for (GLuint buffer : {buffers.control, buffers.trajectories, buffers.observations, buffers.reference,
                buffers.equations, buffers.candidates, buffers.history, buffers.geometry,
                buffers.params, buffers.view, buffers.grid})
                gpu.DeleteBuffers(1, &buffer);

            for (GLuint program : {programs.initialize, programs.prepare, programs.forward, programs.observe,
                programs.cost, programs.normals, programs.solve, programs.choose,
                programs.diagnostics, programs.pack, programs.lines, programs.points})
                gpu.DeleteProgram(program);
        }

        Simulation(const Simulation&) = delete;
        Simulation& operator=(const Simulation&) = delete;

        int size() const {
            return settings.size();
        }

        bool imported() const {
            return importedData.has_value();
        }

        int playbackSample() const {
            return static_cast<int>(playback % static_cast<std::uint64_t>(settings.samples));
        }

        V3 parameters() const {
            return state.physical();
        }

        // Generates a synthetic dataset from the settings and restarts the estimate.
        void reset() {
            importedData.reset();
            initialize();
        }
        // Adopts imported observations (kept across restarts) and restarts the estimate.
        void reset(const Dataset& data) {
            importedData = data;
            settings.samples = data.samples;
            initialize();
        }
        // Restarts the estimate on the current dataset, synthetic or imported.
        void restart() {
            initialize();
        }

        // Reads back the optimizer state (a GPU sync point).
        void measure() {
            gpu.MemoryBarrier(gl::BUFFER_UPDATE_BARRIER_BIT);
            gpu.BindBuffer(gl::SHADER_STORAGE_BUFFER, buffers.control);
            gpu.GetBufferSubData(gl::SHADER_STORAGE_BUFFER, 0, sizeof(state), &state);
        }

        // Advances playback by one sample and, while fitting, runs one optimizer iteration.
        void tick() {
            ++playback;
            pollTimer();

            if (state.status() >= FitStatus::converged)
                return;

            const bool timing = !timerPending;

            if (timing)
                gpu.BeginQuery(gl::TIME_ELAPSED, query);

            if (!seeded && settings.seedSearch) {
                simulate(Mode::seed, seedJobs, true);
                dispatch(programs.choose, 1);
            }

            seeded = true;
            simulate(Mode::jacobian, jacobianJobs, false);
            dispatch(programs.normals, size());
            dispatch(programs.solve, 1);
            simulate(Mode::lineSearch, lineSearchJobs, true);
            dispatch(programs.choose, 1);
            simulate(Mode::current, 1, false);
            dispatch(programs.diagnostics, 1);

            if (timing) {
                gpu.EndQuery(gl::TIME_ELAPSED);
                timerPending = true;
            }

            measure();
        }

        bool healthy() const {
            return state.invalid() == 0 && std::isfinite(state.mse()) && state.mse() >= 0 && state.weightSum() > 0 &&state.status() != FitStatus::uninformative;
        }

        // Readback helpers. Full transfers are used by exports and verification only.

        std::vector<F4> predictionSnapshot() {
            return read(buffers.trajectories, size());
        }

        std::vector<F4> observationSnapshot() {
            return read(buffers.observations, size());
        }

        std::vector<F4> referenceSnapshot() {
            return read(buffers.reference, size());
        }

        std::vector<F4> historySnapshot() {
            return read(buffers.history, historyCapacity);
        }

        // Verification hook: the GPU trajectory for explicit physical parameters.
        std::vector<F4> forwardAt(V3 physical) {
            const F4 candidate{ std::log(physical.x), std::log(physical.y), std::log(physical.z), 0 };
            gpu.MemoryBarrier(gl::BUFFER_UPDATE_BARRIER_BIT);
            gpu.BindBuffer(gl::SHADER_STORAGE_BUFFER, buffers.candidates);
            gpu.BufferSubData(gl::SHADER_STORAGE_BUFFER, 0, sizeof(candidate), candidate.data());
            uploadParams(Mode::current, 1);
            dispatchGroups(programs.forward, 1);
            return predictionSnapshot();
        }

        // Verification hook: the seven Jacobian trajectories (current, then -/+ per parameter).
        std::vector<F4> jacobianTrajectories() {
            simulate(Mode::jacobian, jacobianJobs, false);
            return read(buffers.trajectories, jacobianJobs * size());
        }

        // Verification hook: the per-workgroup normal-equation partial sums.
        std::vector<F4> normalSnapshot() {
            simulate(Mode::jacobian, jacobianJobs, false);
            dispatch(programs.normals, size());
            return read(buffers.equations, workgroups(size()) * 3);
        }

        // Writes the observations in the importable CSV format (no truth values).
        void exportObservations(const std::filesystem::path& path) {
            const auto data = observationSnapshot();
            std::string text;
            text.reserve(data.size() * 64);
            auto out = std::back_inserter(text);
            std::format_to(out, "# Inverse Physics Lab 3D; dt=1/60; nodes 6 and 7 are fixed\n# samples={}\n{}\n",
                settings.samples, observationHeaderLine);
            for (std::size_t i = 0; i < data.size(); ++i)
                std::format_to(out, "{},{},{},{},{},{}\n", i / nodes, i % nodes, data[i][0], data[i][1], data[i][2],
                    data[i][3]);
            write(path, text, "Cannot write observations.");
        }

        // Writes observed and fitted positions for every sample and node, with the fitted parameters.
        void exportFit(const std::filesystem::path& path) {
            const auto observed = observationSnapshot(), fitted = predictionSnapshot();
            const V3 p = parameters();
            std::string text;
            text.reserve(observed.size() * 128);
            auto out = std::back_inserter(text);
            std::format_to(out, "# fitted stiffness={}, damping={}, gravity={}, MSE={}\n", p.x, p.y, p.z, state.mse());
            std::format_to(out, "sample,time,node,observed_x,observed_y,observed_z,weight,fitted_x,fitted_y,fitted_z\n");
            for (std::size_t i = 0; i < observed.size(); ++i) {
                const auto& o = observed[i];
                const auto& f = fitted[i];
                std::format_to(out, "{},{},{},{},{},{},{},{},{},{}\n", i / nodes,
                    static_cast<double>(i / nodes) / sampleRate, i % nodes, o[0], o[1], o[2], o[3], f[0],
                    f[1], f[2]);
            }
            write(path, text, "Cannot write fit CSV.");
        }

        // Builds the line geometry on the GPU, then draws the 3D scene and the convergence plot.
        void draw(const Camera& camera, const Viewport& world, const Viewport& plot) {
            uploadParams(Mode::current, 1);
            dispatch(programs.pack, settings.trajectoryVertices() + overlayVertices + plotVertices);
            gpu.MemoryBarrier(gl::VERTEX_ATTRIB_ARRAY_BARRIER_BIT);
            uploadView(camera, static_cast<float>(world.width) / static_cast<float>(world.height));

            glViewport(world.x, world.y, world.width, world.height);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            gpu.UseProgram(programs.lines);
            gpu.Uniform1i(lineVariant, 0);
            gpu.BindVertexArray(vaos.grid);
            glDrawArrays(GL_LINES, 0, floorVertices);
            gpu.BindVertexArray(vaos.geometry);
            glDrawArrays(GL_LINES, 0, settings.trajectoryVertices() + overlayVertices);
            gpu.UseProgram(programs.points);
            glEnable(gl::PROGRAM_POINT_SIZE);
            if (settings.trails) {
                gpu.Uniform1i(pointVariant, 0);
                gpu.BindVertexArray(vaos.observations);
                glDrawArrays(GL_POINTS, 0, size());
            }
            gpu.Uniform1i(pointVariant, 1);
            gpu.BindVertexArray(vaos.trajectories);
            glDrawArrays(GL_POINTS, playbackSample() * nodes, nodes);

            glViewport(plot.x, plot.y, plot.width, plot.height);
            gpu.UseProgram(programs.lines);
            gpu.Uniform1i(lineVariant, 1);
            gpu.BindVertexArray(vaos.grid);
            glDrawArrays(GL_LINES, floorVertices, plotGridVertices);
            gpu.BindVertexArray(vaos.geometry);
            glDrawArrays(GL_LINES, settings.trajectoryVertices() + overlayVertices, plotVertices);
            glDisable(GL_BLEND);
        }

    private:
        static constexpr std::string_view observationHeaderLine = "sample,node,x,y,z,weight";

        GpuApi& gpu;

        struct {
            GLuint initialize{}, prepare{}, forward{}, observe{}, cost{}, normals{}, solve{}, choose{}, diagnostics{},
                pack{}, lines{}, points{};
        } programs;

        struct {
            GLuint control{}, trajectories{}, observations{}, reference{}, equations{}, candidates{}, history{},
                geometry{}, params{}, view{}, grid{};
        } buffers;

        struct {
            GLuint geometry{}, observations{}, trajectories{}, grid{};
        } vaos;

        GLuint query{};
        GLint lineVariant = -1, pointVariant = -1;
        int floorVertices = 0, plotGridVertices = 0; // static line counts in buffers.grid
        bool timerPending = false, seeded = false;
        std::optional<Dataset> importedData;

        template <class T> static std::ptrdiff_t bytes(const std::vector<T>& data) {
            return static_cast<std::ptrdiff_t>(data.size() * sizeof(T));
        }

        static void write(const std::filesystem::path& path, const std::string& text, const char* error) {
            std::ofstream file(path);
            file.write(text.data(), static_cast<std::streamsize>(text.size()));
            if (!file)
                throw std::runtime_error(error);
        }

        void allocate(GLuint buffer, std::size_t size, const void* data = nullptr) {
            gpu.BindBuffer(gl::SHADER_STORAGE_BUFFER, buffer);
            gpu.BufferData(gl::SHADER_STORAGE_BUFFER, static_cast<std::ptrdiff_t>(size), data, gl::DYNAMIC_COPY);
        }

        void attribute(GLuint slot, GLuint buffer, std::size_t stride, std::size_t offset = 0) {
            gpu.BindBuffer(gl::ARRAY_BUFFER, buffer);
            gpu.EnableVertexAttribArray(slot);
            gpu.VertexAttribPointer(slot, 4, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(stride),
                reinterpret_cast<const void*>(offset));
        }

        std::vector<F4> read(GLuint buffer, int count) {
            std::vector<F4> data(static_cast<std::size_t>(count));
            gpu.MemoryBarrier(gl::BUFFER_UPDATE_BARRIER_BIT);
            gpu.BindBuffer(gl::SHADER_STORAGE_BUFFER, buffer);
            gpu.GetBufferSubData(gl::SHADER_STORAGE_BUFFER, 0, bytes(data), data.data());
            return data;
        }

        void dispatchGroups(GLuint program, int groups) {
            gpu.UseProgram(program);
            gpu.DispatchCompute(static_cast<GLuint>(groups), 1, 1);
            gpu.MemoryBarrier(gl::SHADER_STORAGE_BARRIER_BIT);
        }

        void dispatch(GLuint program, int invocations) {
            dispatchGroups(program, workgroups(invocations));
        }

        void uploadParams(Mode mode, int jobs, bool simulateTruth = false) {
            const Params params{
                .samples = settings.samples,
                .substeps = settings.substeps,
                .mode = std::to_underlying(mode),
                .jobs = jobs,
                .scene = std::to_underlying(settings.scene),
                .stiffnessOnly = settings.stiffnessOnly() ? 1 : 0,
                .hasReference = imported() ? 0 : 1,
                .simulateTruth = simulateTruth ? 1 : 0,
                .playback = playbackSample(),
                .selectedNode = settings.node,
                .showTrails = settings.trails ? 1 : 0,
                .showReference = settings.reference ? 1 : 0,
                .truth = settings.truth(),
                .noise = settings.noise,
                .fdStep = fdStep,
                .padding = {},
            };
            gpu.BindBuffer(gl::UNIFORM_BUFFER, buffers.params);
            gpu.BufferSubData(gl::UNIFORM_BUFFER, 0, sizeof(params), &params);
        }

        void uploadView(const Camera& camera, float aspect) {
            const ViewParams view{
                .view = camera.view(), .projection = perspective(aspect), .selectedNode = settings.node, .padding = {} };
            gpu.BindBuffer(gl::UNIFORM_BUFFER, buffers.view);
            gpu.BufferSubData(gl::UNIFORM_BUFFER, 0, sizeof(view), &view);
        }

        // Simulates `jobs` candidate trajectories (one workgroup each) and optionally their objectives.
        void simulate(Mode mode, int jobs, bool evaluate) {
            uploadParams(mode, jobs);
            dispatch(programs.prepare, jobs);
            dispatchGroups(programs.forward, jobs);
            if (evaluate)
                dispatchGroups(programs.cost, jobs);
        }

        void initialize() {
            if (settings.samples < minSamples || settings.samples > maxSamples ||
                (settings.substeps != 4 && settings.substeps != 8))
                throw std::runtime_error("Unsupported sample count or integrator setting.");
            if (timerPending) {
                std::uint64_t ignored{};
                gpu.GetQueryObjectui64v(query, gl::QUERY_RESULT, &ignored); // drain before reuse
                timerPending = false;
            }
            gpuMs = 0;
            playback = 0;
            seeded = false;
            const std::size_t records = static_cast<std::size_t>(size());
            allocate(buffers.trajectories, seedJobs * records * sizeof(F4));
            allocate(buffers.observations, records * sizeof(F4), importedData ? importedData->observations.data() : nullptr);
            const std::vector<F4> zero(records);
            allocate(buffers.reference, records * sizeof(F4), zero.data());
            allocate(buffers.equations, static_cast<std::size_t>(workgroups(size())) * 3 * sizeof(F4));
            allocate(buffers.geometry,
                static_cast<std::size_t>(settings.trajectoryVertices() + overlayVertices + plotVertices) * sizeof(Vertex));
            uploadParams(Mode::current, 1, true);
            dispatch(programs.initialize, historyCapacity);
            if (!importedData) {
                dispatchGroups(programs.forward, 1); // clean reference from the true parameters
                dispatch(programs.observe, size());
            }
            simulate(Mode::current, 1, false);
            dispatch(programs.diagnostics, 1);
            measure();
        }

        // Collects the previous iteration's timing once the driver has it, without stalling.
        void pollTimer() {
            if (!timerPending)
                return;
            GLuint ready{};
            gpu.GetQueryObjectuiv(query, gl::QUERY_RESULT_AVAILABLE, &ready);
            if (!ready)
                return;
            std::uint64_t nanoseconds{};
            gpu.GetQueryObjectui64v(query, gl::QUERY_RESULT, &nanoseconds);
            gpuMs = static_cast<float>(nanoseconds) * 1e-6f;
            timerPending = false;
        }

        // Static reference lines: the floor grid with the support posts, then the plot grid.
        void buildGrids() {
            std::vector<Vertex> vertices;
            auto segment = [&](V3 a, V3 b, F4 color) {
                vertices.push_back({ vec4(a), color });
                vertices.push_back({ vec4(b), color });
                };
            const F4 floorColor{ .2f, .3f, .4f, .25f }, plotColor{ .2f, .3f, .4f, .35f };
            for (int i = -8; i <= 8; ++i) {
                const float d = static_cast<float>(i) * .25f;
                segment({ -2, 0, d }, { 2, 0, d }, floorColor);
                segment({ d, 0, -2 }, { d, 0, 2 }, floorColor);
            }
            segment({ -.5f, 1, .25f }, { .5f, 1, .25f }, { .65f, .7f, .8f, .9f }); // beam between the anchors
            segment({ -.5f, 0, .25f }, { -.5f, 1, .25f }, { .4f, .45f, .55f, .5f });
            segment({ .5f, 0, .25f }, { .5f, 1, .25f }, { .4f, .45f, .55f, .5f });
            floorVertices = static_cast<int>(vertices.size());
            for (int i = 0; i <= 5; ++i) {
                const float y = -.75f + 1.5f * static_cast<float>(i) / 5;
                segment({ -.92f, y, 0 }, { .92f, y, 0 }, plotColor);
            }
            for (int i = 0; i <= 4; ++i) {
                const float x = -.92f + 1.84f * static_cast<float>(i) / 4;
                segment({ x, -.75f, 0 }, { x, .75f, 0 }, plotColor);
            }
            plotGridVertices = static_cast<int>(vertices.size()) - floorVertices;
            gpu.BindBuffer(gl::ARRAY_BUFFER, buffers.grid);
            gpu.BufferData(gl::ARRAY_BUFFER, bytes(vertices), vertices.data(), gl::STATIC_DRAW);
        }
    };
}
