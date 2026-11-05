#include "ModernUI.hpp"
#include "GpuApi.hpp"
#include "Math3D.hpp"
#include "ObservationIO.hpp"
#include "InverseShaders.hpp"
#include <commctrl.h>
#include <commdlg.h>
#include <windowsx.h>
#include <chrono>
#include <format>
#include <iomanip>
#include <memory>
namespace inverse {
constexpr int panelWidth = 320, plotHeight = 170;
V3 xyz(const F4 &f) {
    return {f[0], f[1], f[2]};
}
F4 vec4(V3 p, float w = 1) {
    return {p.x, p.y, p.z, w};
}
struct alignas(16) FitState {
    F4 parameters{}, statistics{}, proposal{}, diagnostics{}, quality{};
};
struct alignas(16) Vertex {
    F4 p{}, color{};
};
static_assert(sizeof(FitState) == 80 && sizeof(Vertex) == 32);
struct Settings {
    int samples = 240, substeps = 4, scene = 0, node = 8;
    float stiffness = 18, damping = .6f, gravity = 5.5f, noise = .012f;
    bool seedSearch = true, trails = true, reference = true;
};
class Simulation {
    GL &gl;
    GLuint initialize{}, prepare{}, forward{}, observe{}, cost{}, normals{}, solve{}, choose{}, diagnostics{},
        pack{}, lineProgram{}, pointProgram{};
    GLuint control{}, trajectories{}, observations{}, reference{}, equations{}, candidates{}, history{},
        geometry{}, grid{}, query{};
    std::array<GLuint, 4> vaos{};
    int worldVertices = 0, plotVertices = 0;
    bool pending = false, seeded = false;
    void uniforms(GLuint p, int mode = 0, int jobs = 1) {
        gl.UseProgram(p);
        gl.integer(p, "samples", settings.samples);
        gl.integer(p, "substeps", settings.substeps);
        gl.integer(p, "mode", mode);
        gl.integer(p, "jobs", jobs);
        gl.integer(p, "scene", settings.scene);
        gl.integer(p, "stiffnessOnly", settings.scene == 3);
        gl.integer(p, "hasReference", !imported);
        gl.scalar(p, "noise", settings.noise);
        gl.scalar(p, "fdStep", .002f);
        gl.vector(p, "truth", settings.stiffness, settings.damping, settings.gravity);
    }
    void dispatch(int elements) {
        gl.DispatchCompute((elements + 127) / 128, 1, 1);
        gl.MemoryBarrier(STORAGE_BARRIER);
    }
    void allocate(GLuint buffer, size_t bytes, const void *data = nullptr) {
        gl.BindBuffer(SSBO, buffer);
        gl.BufferData(SSBO, static_cast<std::ptrdiff_t>(bytes), data, 0x88E8);
    }
    void batch(int mode, int count, bool evaluate = true) {
        uniforms(prepare, mode, count);
        dispatch(count);
        uniforms(forward, 1, count);
        gl.DispatchCompute(count, 1, 1);
        gl.MemoryBarrier(STORAGE_BARRIER);
        if (evaluate) {
            uniforms(cost, 0, count);
            gl.DispatchCompute(count, 1, 1);
            gl.MemoryBarrier(STORAGE_BARRIER);
        }
    }
    void diagnose() {
        uniforms(diagnostics);
        dispatch(1);
        measure();
    }
    void attribute(GLuint slot, GLuint buffer, int stride, size_t offset = 0) {
        gl.BindBuffer(0x8892, buffer);
        gl.EnableVertexAttribArray(slot);
        gl.VertexAttribPointer(slot, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void *>(offset));
    }

  public:
    Settings settings;
    FitState state{};
    double time = 0;
    float gpuMs = 0;
    bool imported = false;
    explicit Simulation(GL &api) : gl(api) {
        auto kernel = [&](const std::string &s, const char *label) {
            return gl.program({{COMPUTE, shaders::common + s}}, label);
        };
        initialize = kernel(shaders::initialize, "Initialize parameter fit");
        prepare = kernel(shaders::prepare, "Prepare log-space candidates");
        forward = kernel(shaders::forward, "3D spring network RK4");
        observe = kernel(shaders::observe, "Generate weighted observations");
        cost = kernel(shaders::cost, "Parallel candidate objectives");
        normals = kernel(shaders::normals, "Finite-difference normal equations");
        solve = kernel(shaders::solve, "Damped 3x3 solve");
        choose = kernel(shaders::choose, "Monotone candidate acceptance");
        diagnostics = kernel(shaders::diagnostics, "Fit diagnostics");
        pack = kernel(shaders::pack, "GPU render geometry");
        lineProgram = gl.program({{VERTEX, shaders::lineVertex}, {FRAGMENT, shaders::lineFragment}},
                                 "Spring and history renderer");
        pointProgram = gl.program({{VERTEX, shaders::pointVertex}, {FRAGMENT, shaders::pointFragment}},
                                  "Observation and body renderer");
        control = gl.buffer(0, 80);
        trajectories = gl.buffer(1, 16);
        observations = gl.buffer(2, 16);
        reference = gl.buffer(3, 16);
        equations = gl.buffer(4, 16);
        candidates = gl.buffer(5, 33 * 16);
        history = gl.buffer(6, 256 * 16);
        geometry = gl.buffer(7, 32);
        gl.GenBuffers(1, &grid);
        gl.buffers.push_back(grid);
        gl.GenQueries(1, &query);
        gl.GenVertexArrays(4, vaos.data());
        gl.BindVertexArray(vaos[0]);
        attribute(0, geometry, 32);
        attribute(1, geometry, 32, 16);
        gl.BindVertexArray(vaos[1]);
        attribute(0, observations, 16);
        gl.BindVertexArray(vaos[2]);
        attribute(0, trajectories, 16);
        gl.BindVertexArray(vaos[3]);
        attribute(0, grid, 32);
        attribute(1, grid, 32, 16);
        std::vector<Vertex> vertices;
        auto segment = [&](V3 a, V3 b, F4 color) {
            vertices.push_back({vec4(a), color});
            vertices.push_back({vec4(b), color});
        };
        for (int i = -8; i <= 8; ++i) {
            float d = i * .25f;
            segment({-2, 0, d}, {2, 0, d}, {.2f, .3f, .4f, .25f});
            segment({d, 0, -2}, {d, 0, 2}, {.2f, .3f, .4f, .25f});
        }
        segment({-.5f, 1, .25f}, {.5f, 1, .25f}, {.65f, .7f, .8f, .9f});
        segment({-.5f, 0, .25f}, {-.5f, 1, .25f}, {.4f, .45f, .55f, .5f});
        segment({.5f, 0, .25f}, {.5f, 1, .25f}, {.4f, .45f, .55f, .5f});
        worldVertices = static_cast<int>(vertices.size());
        for (int i = 0; i <= 5; ++i) {
            float y = -.75f + 1.5f * i / 5;
            segment({-.92f, y, 0}, {.92f, y, 0}, {.2f, .3f, .4f, .35f});
        }
        for (int i = 0; i <= 4; ++i) {
            float x = -.92f + 1.84f * i / 4;
            segment({x, -.75f, 0}, {x, .75f, 0}, {.2f, .3f, .4f, .35f});
        }
        plotVertices = static_cast<int>(vertices.size()) - worldVertices;
        allocate(grid, vertices.size() * sizeof(Vertex), vertices.data());
        reset();
    }
    ~Simulation() {
        gl.DeleteQueries(1, &query);
        gl.DeleteVertexArrays(4, vaos.data());
    }
    int size() const {
        return settings.samples * 8;
    }
    int trajectoryVertices() const {
        return (settings.samples - 1) * 16;
    }
    V3 parameters() const {
        return {std::exp(state.parameters[0]), std::exp(state.parameters[1]), std::exp(state.parameters[2])};
    }
    void reset(const Dataset *data = nullptr) {
        if (data)
            settings.samples = data->samples;
        if (settings.samples < 8 || settings.samples > 360 ||
            (settings.substeps != 4 && settings.substeps != 8))
            throw std::runtime_error("Unsupported sample count or integrator setting.");
        if (pending) {
            unsigned long long ignored{};
            gl.GetQueryObjectui64v(query, 0x8866, &ignored);
            pending = false;
        }
        gpuMs = 0;
        time = 0;
        seeded = false;
        imported = data != nullptr;
        allocate(trajectories, 33 * size() * sizeof(F4));
        allocate(observations, size() * sizeof(F4), data ? data->observations.data() : nullptr);
        std::vector<F4> zero(size());
        allocate(reference, size() * sizeof(F4), zero.data());
        allocate(equations, ((size() + 127) / 128) * 3 * sizeof(F4));
        allocate(geometry, (trajectoryVertices() + 128 + 510) * sizeof(Vertex));
        uniforms(initialize);
        dispatch(256);
        if (!imported) {
            uniforms(forward, 0);
            gl.DispatchCompute(1, 1, 1);
            gl.MemoryBarrier(STORAGE_BARRIER);
            uniforms(observe);
            dispatch(size());
        }
        batch(0, 1, false);
        diagnose();
    }
    void restart() {
        if (imported) {
            Dataset data{settings.samples, observationSnapshot()};
            reset(&data);
        } else
            reset();
    }
    void measure() {
        gl.MemoryBarrier(READBACK_BARRIER);
        gl.BindBuffer(SSBO, control);
        gl.GetBufferSubData(SSBO, 0, sizeof(state), &state);
    }
    void tick() {
        time += 1. / 60;
        if (pending) {
            GLuint ready{};
            gl.GetQueryObjectuiv(query, 0x8867, &ready);
            if (ready) {
                unsigned long long ns{};
                gl.GetQueryObjectui64v(query, 0x8866, &ns);
                gpuMs = static_cast<float>(ns) * 1e-6f;
                pending = false;
            }
        }
        if (state.diagnostics[2] >= 2)
            return;
        bool timing = !pending;
        if (timing)
            gl.BeginQuery(0x88BF, query);
        if (!seeded && settings.seedSearch) {
            batch(3, 33);
            uniforms(choose, 3, 33);
            dispatch(1);
        }
        seeded = true;
        batch(1, 7, false);
        uniforms(normals);
        dispatch(size());
        uniforms(solve);
        dispatch(1);
        batch(2, 8);
        uniforms(choose, 2, 8);
        dispatch(1);
        batch(0, 1, false);
        uniforms(diagnostics);
        dispatch(1);
        if (timing) {
            gl.EndQuery(0x88BF);
            pending = true;
        }
        measure();
    }
    bool healthy() const {
        return state.quality[3] == 0 && std::isfinite(state.statistics[0]) && state.statistics[0] >= 0 &&
               state.statistics[1] > 0 && state.diagnostics[2] != 4;
    }
    std::vector<F4> read(GLuint buffer, int n) {
        std::vector<F4> data(n);
        gl.MemoryBarrier(READBACK_BARRIER);
        gl.BindBuffer(SSBO, buffer);
        gl.GetBufferSubData(SSBO, 0, data.size() * sizeof(F4), data.data());
        return data;
    }
    std::vector<F4> predictionSnapshot() {
        return read(trajectories, size());
    }
    std::vector<F4> observationSnapshot() {
        return read(observations, size());
    }
    std::vector<F4> referenceSnapshot() {
        return read(reference, size());
    }
    std::vector<F4> historySnapshot() {
        return read(history, 256);
    }
    // Verification-only hooks; the live fitting pipeline never receives CPU-computed trajectories.
    std::vector<F4> forwardAt(V3 physical) {
        F4 candidate{std::log(physical.x), std::log(physical.y), std::log(physical.z), 0};
        gl.MemoryBarrier(READBACK_BARRIER);
        gl.BindBuffer(SSBO, candidates);
        gl.BufferSubData(SSBO, 0, 16, candidate.data());
        uniforms(forward, 1);
        gl.DispatchCompute(1, 1, 1);
        gl.MemoryBarrier(STORAGE_BARRIER);
        return predictionSnapshot();
    }
    std::vector<F4> jacobianTrajectories() {
        batch(1, 7, false);
        return read(trajectories, 7 * size());
    }
    std::vector<F4> normalSnapshot() {
        batch(1, 7, false);
        uniforms(normals);
        dispatch(size());
        return read(equations, ((size() + 127) / 128) * 3);
    }
    void exportObservations(const std::filesystem::path &path) {
        auto data = observationSnapshot();
        std::ofstream out(path);
        out << std::setprecision(9)
            << "# Inverse Physics Lab 3D; dt=1/60; nodes 6 and 7 are fixed\n# samples=" << settings.samples
            << "\nsample,node,x,y,z,weight\n";
        for (int i = 0; i < size(); ++i) {
            out << i / 8 << ',' << i % 8;
            for (float v : data[i])
                out << ',' << v;
            out << '\n';
        }
        if (!out)
            throw std::runtime_error("Cannot write observations.");
    }
    void exportFit(const std::filesystem::path &path) {
        auto observed = observationSnapshot(), fitted = predictionSnapshot();
        auto p = parameters();
        std::ofstream out(path);
        out << std::setprecision(9) << "# fitted stiffness=" << p.x << ", damping=" << p.y
            << ", gravity=" << p.z << ", MSE=" << state.statistics[0] << "\n";
        out << "sample,time,node,observed_x,observed_y,observed_z,weight,fitted_x,fitted_y,fitted_z\n";
        for (int i = 0; i < size(); ++i) {
            out << i / 8 << ',' << (i / 8) / 60. << ',' << i % 8;
            for (float f : observed[i])
                out << ',' << f;
            for (int k = 0; k < 3; ++k)
                out << ',' << fitted[i][k];
            out << '\n';
        }
        if (!out)
            throw std::runtime_error("Cannot write fit CSV.");
    }
    void draw(const Camera &camera, int width, int height) {
        int frame = static_cast<int>(time * 60) % settings.samples;
        uniforms(pack);
        gl.integer(pack, "playback", frame);
        gl.integer(pack, "selectedNode", settings.node);
        gl.integer(pack, "showTrails", settings.trails);
        gl.integer(pack, "showReference", settings.reference);
        dispatch(trajectoryVertices() + 638);
        gl.MemoryBarrier(0x1);
        glViewport(panelWidth, 64 + plotHeight, width, std::max(1, height - plotHeight));
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        auto view = camera.view(),
             projection = perspective(static_cast<float>(width) / std::max(1, height - plotHeight));
        gl.UseProgram(lineProgram);
        gl.integer(lineProgram, "screenSpace", 0);
        gl.matrix(lineProgram, "view", view.v.data());
        gl.matrix(lineProgram, "projection", projection.v.data());
        gl.BindVertexArray(vaos[3]);
        glDrawArrays(GL_LINES, 0, worldVertices);
        gl.BindVertexArray(vaos[0]);
        glDrawArrays(GL_LINES, 0, trajectoryVertices() + 128);
        gl.UseProgram(pointProgram);
        gl.matrix(pointProgram, "view", view.v.data());
        gl.matrix(pointProgram, "projection", projection.v.data());
        gl.integer(pointProgram, "selectedNode", settings.node);
        glEnable(0x8642);
        if (settings.trails) {
            gl.integer(pointProgram, "pointMode", 0);
            gl.BindVertexArray(vaos[1]);
            glDrawArrays(GL_POINTS, 0, size());
        }
        gl.integer(pointProgram, "pointMode", 1);
        gl.BindVertexArray(vaos[2]);
        glDrawArrays(GL_POINTS, frame * 8, 8);
        glViewport(panelWidth, 64, width, plotHeight);
        gl.UseProgram(lineProgram);
        gl.integer(lineProgram, "screenSpace", 1);
        gl.BindVertexArray(vaos[3]);
        glDrawArrays(GL_LINES, worldVertices, plotVertices);
        gl.BindVertexArray(vaos[0]);
        glDrawArrays(GL_LINES, trajectoryVertices() + 128, 510);
        glDisable(GL_BLEND);
    }
};
void capture(const std::filesystem::path &path, int x, int y, int width, int height) {
    int stride = (width * 3 + 3) & ~3;
    std::vector<unsigned char> pixels(static_cast<size_t>(stride) * height);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glReadPixels(x, y, width, height, GL_BGR_EXT, GL_UNSIGNED_BYTE, pixels.data());
    BITMAPFILEHEADER header{};
    BITMAPINFOHEADER info{};
    header.bfType = 0x4d42;
    header.bfOffBits = sizeof(header) + sizeof(info);
    header.bfSize = header.bfOffBits + static_cast<DWORD>(pixels.size());
    info.biSize = sizeof(info);
    info.biWidth = width;
    info.biHeight = height;
    info.biPlanes = 1;
    info.biBitCount = 24;
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char *>(&header), sizeof(header));
    out.write(reinterpret_cast<const char *>(&info), sizeof(info));
    out.write(reinterpret_cast<const char *>(pixels.data()), pixels.size());
    if (!out)
        throw std::runtime_error("Unable to save framebuffer image.");
}

struct App {
    GL gl;
    std::unique_ptr<Simulation> sim;
    Camera camera;
    HWND window{}, status{}, pause{}, values{}, combos[4]{}, sliders[4]{}, labels[4]{}, trailsBox{},
        referenceBox{}, seedBox{};
    HFONT font{};
    int width = 1360, height = 1000;
    bool running = true, paused = false, single = false, orbit = false;
    POINT previous{};
    float fps = 0;
    ~App() {
        sim.reset();
        if (font)
            DeleteObject(font);
    }
    HWND control(const wchar_t *kind, const std::wstring &text, DWORD style, int x, int y, int w, int h,
                 int id = 0) {
        auto c = CreateWindowExW(0, kind, text.c_str(), WS_CHILD | WS_VISIBLE | style, x, y, w, h, window,
                                 reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)), GetModuleHandleW(nullptr),
                                 nullptr);
        SendMessageW(c, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
        return c;
    }
    HWND combo(const wchar_t *title, int y, int id, std::initializer_list<const wchar_t *> items) {
        control(L"STATIC", title, 0, 18, y, 250, 21);
        auto c = control(L"COMBOBOX", L"", CBS_DROPDOWNLIST | WS_TABSTOP, 18, y + 22, 250, 210, id);
        for (auto s : items)
            SendMessageW(c, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(s));
        SendMessageW(c, CB_SETCURSEL, 0, 0);
        return c;
    }
    void controls() {
        font = CreateFontW(-16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                           CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
        control(L"STATIC", L"INVERSE PHYSICS / 3D GPU", 0, 18, 14, 255, 27);
        control(L"STATIC", L"Recover a 3D spring network model", 0, 18, 43, 255, 24);
        combos[0] = combo(L"Synthetic experiment (new data)", 76, 10,
                          {L"Fit k, damping and gravity", L"Noisy 3D observations",
                           L"Sparse and noisy observations", L"Stiffness only (d, g known)"});
        combos[1] = combo(L"Samples at 60 Hz (new data)", 136, 11,
                          {L"120 / 1.98 seconds", L"240 / 3.98 seconds", L"360 / 5.98 seconds"});
        SendMessageW(combos[1], CB_SETCURSEL, 1, 0);
        combos[2] = combo(L"RK4 substeps (restarts fit)", 196, 12, {L"4 / balanced", L"8 / accurate"});
        combos[3] = combo(L"Displayed observation trajectories", 256, 13,
                          {L"All nodes", L"Node 0", L"Node 1", L"Node 2", L"Node 3", L"Node 4", L"Node 5",
                           L"Anchor 6", L"Anchor 7"});
        int positions[]{40, 40, 50, 30};
        for (int i = 0; i < 4; ++i) {
            labels[i] = control(L"STATIC", L"", 0, 18, 332 + i * 52, 250, 22);
            sliders[i] =
                control(TRACKBAR_CLASSW, L"", TBS_HORZ | WS_TABSTOP, 12, 354 + i * 52, 260, 29, 20 + i);
            SendMessageW(sliders[i], TBM_SETRANGE, TRUE, MAKELPARAM(0, 100));
            SendMessageW(sliders[i], TBM_SETPOS, TRUE, positions[i]);
        }
        trailsBox = control(L"BUTTON", L"Show observation / fitted paths", BS_AUTOCHECKBOX | WS_TABSTOP, 18,
                            554, 250, 25, 40);
        SendMessageW(trailsBox, BM_SETCHECK, BST_CHECKED, 0);
        referenceBox = control(L"BUTTON", L"Show clean synthetic body", BS_AUTOCHECKBOX | WS_TABSTOP, 18, 582,
                               250, 25, 41);
        SendMessageW(referenceBox, BM_SETCHECK, BST_CHECKED, 0);
        seedBox = control(L"BUTTON", L"GPU seed search (on restart)", BS_AUTOCHECKBOX | WS_TABSTOP, 18, 610,
                          250, 25, 42);
        SendMessageW(seedBox, BM_SETCHECK, BST_CHECKED, 0);
        values = control(L"STATIC", L"", 0, 18, 645, 250, 85);
        pause = control(L"BUTTON", L"Pause", BS_PUSHBUTTON | WS_TABSTOP, 18, 742, 116, 32, 30);
        control(L"BUTTON", L"Restart fit", BS_PUSHBUTTON | WS_TABSTOP, 148, 742, 120, 32, 31);
        control(L"BUTTON", L"Single step", BS_PUSHBUTTON | WS_TABSTOP, 18, 784, 116, 32, 32);
        control(L"BUTTON", L"Fit view", BS_PUSHBUTTON | WS_TABSTOP, 148, 784, 120, 32, 33);
        control(L"BUTTON", L"Import CSV...", BS_PUSHBUTTON | WS_TABSTOP, 18, 826, 116, 32, 34);
        control(L"BUTTON", L"Export data...", BS_PUSHBUTTON | WS_TABSTOP, 148, 826, 120, 32, 35);
        control(L"BUTTON", L"Export reconstructed motion...", BS_PUSHBUTTON | WS_TABSTOP, 18, 868, 250, 30,
                36);
        control(L"STATIC",
                L"Cyan: data  Orange: fit\nRMB: orbit   Wheel: zoom\nSpace: pause   R: restart   N: step", 0,
                18, 908, 255, 52);
        status = control(L"STATIC", L"Initializing...", 0, panelWidth + 12, height - 60,
                         width - panelWidth - 24, 58);
        updateSliders();
        fit();
        updateStatus();
    }
    void syncSource() {
        bool enabled = !sim->imported;
        for (auto slider : sliders)
            EnableWindow(slider, enabled);
        EnableWindow(combos[1], enabled);
        EnableWindow(referenceBox, enabled);
    }
    void fit() {
        camera = Camera{};
        camera.pitch = .35f;
    }
    void updateSliders() {
        if (!sim || sim->imported)
            return;
        int v[4]{};
        for (int i = 0; i < 4; ++i)
            v[i] = static_cast<int>(SendMessageW(sliders[i], TBM_GETPOS, 0, 0));
        auto old = sim->settings;
        auto &s = sim->settings;
        s.stiffness = 10 + .2f * v[0];
        s.damping = .2f + .01f * v[1];
        s.gravity = 3 + .05f * v[2];
        s.noise = .0004f * v[3];
        std::wstring text[]{std::format(L"Target stiffness: {:.2f}", s.stiffness),
                            std::format(L"Target damping: {:.2f}", s.damping),
                            std::format(L"Target gravity: {:.2f}", s.gravity),
                            std::format(L"Noise half-width: {:.4f}", s.noise)};
        for (int i = 0; i < 4; ++i)
            SetWindowTextW(labels[i], text[i].c_str());
        if (old.stiffness != s.stiffness || old.damping != s.damping || old.gravity != s.gravity ||
            old.noise != s.noise) {
            sim->reset();
            updateStatus();
        }
    }
    void resize() {
        RECT r{};
        GetClientRect(window, &r);
        width = r.right;
        height = r.bottom;
        if (status)
            MoveWindow(status, panelWidth + 12, height - 60, std::max(1, width - panelWidth - 24), 58, TRUE);
    }
    void toggle() {
        paused = !paused;
        SetWindowTextW(pause, paused ? L"Resume" : L"Pause");
    }
    void advanceOne() {
        sim->tick();
        single = false;
        updateStatus();
    }
    void stop() {
        orbit = false;
        if (GetCapture() == window)
            ReleaseCapture();
    }
    bool inCanvas(int x, int y) const {
        return x > panelWidth && x < width && y >= 0 && y < height - 64 - plotHeight;
    }
    void mouse(int x, int y) {
        if (orbit) {
            camera.yaw += (x - previous.x) * .007f;
            camera.pitch = std::clamp(camera.pitch + (y - previous.y) * .007f, -1.45f, 1.45f);
        }
        previous = {x, y};
    }
    void command(int id, int code) {
        if (!sim)
            return;
        auto &s = sim->settings;
        if (id >= 10 && id <= 13 && code == CBN_SELCHANGE) {
            int v = static_cast<int>(SendMessageW(combos[id - 10], CB_GETCURSEL, 0, 0));
            if (id == 10) {
                s.scene = v;
                s.samples = std::array{
                    120, 240,
                    360}[std::max(0, static_cast<int>(SendMessageW(combos[1], CB_GETCURSEL, 0, 0)))];
                sim->reset();
                syncSource();
            }
            if (id == 11) {
                s.samples = std::array{120, 240, 360}[v];
                sim->reset();
            }
            if (id == 12) {
                s.substeps = v == 0 ? 4 : 8;
                sim->restart();
            }
            if (id == 13)
                s.node = v == 0 ? 8 : v - 1;
            updateStatus();
        }
        if (id == 30)
            toggle();
        if (id == 31) {
            sim->restart();
            single = false;
            updateStatus();
        }
        if (id == 32) {
            paused = true;
            single = true;
            SetWindowTextW(pause, L"Resume");
        }
        if (id == 33) {
            stop();
            fit();
        }
        if (id == 40)
            s.trails = SendMessageW(trailsBox, BM_GETCHECK, 0, 0) == BST_CHECKED;
        if (id == 41)
            s.reference = SendMessageW(referenceBox, BM_GETCHECK, 0, 0) == BST_CHECKED;
        if (id == 42)
            s.seedSearch = SendMessageW(seedBox, BM_GETCHECK, 0, 0) == BST_CHECKED;
        if (id >= 34 && id <= 36) {
            wchar_t filename[MAX_PATH]{};
            if (id != 34)
                wcscpy_s(filename, id == 35 ? L"Inverse-observations.csv" : L"Inverse-fit.csv");
            OPENFILENAMEW dialog{};
            dialog.lStructSize = sizeof(dialog);
            dialog.hwndOwner = window;
            dialog.lpstrFilter = L"CSV files\0*.csv\0All files\0*.*\0";
            dialog.lpstrFile = filename;
            dialog.nMaxFile = MAX_PATH;
            dialog.lpstrDefExt = L"csv";
            dialog.Flags =
                OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR | (id == 34 ? OFN_FILEMUSTEXIST : OFN_OVERWRITEPROMPT);
            if (id == 34 ? GetOpenFileNameW(&dialog) : GetSaveFileNameW(&dialog))
                try {
                    if (id == 34) {
                        auto data = readObservations(filename);
                        sim->reset(&data);
                        syncSource();
                        updateStatus();
                    }
                    if (id == 35)
                        sim->exportObservations(filename);
                    if (id == 36)
                        sim->exportFit(filename);
                } catch (const std::exception &e) {
                    MessageBoxA(window, e.what(), "Inverse Physics CSV", MB_ICONERROR);
                }
        }
    }
    void draw() {
        glViewport(0, 0, width, height);
        glClearColor(.012f, .018f, .032f, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_SCISSOR_TEST);
        glScissor(0, 0, panelWidth, height);
        glClearColor(246.f / 255, 248.f / 255, 252.f / 255, 1);
        glClear(GL_COLOR_BUFFER_BIT);
        glDisable(GL_SCISSOR_TEST);
        sim->draw(camera, std::max(1, width - panelWidth), std::max(plotHeight + 1, height - 64));
    }
    void updateStatus() {
        auto p = sim->parameters();
        auto &s = sim->settings;
        auto &data = sim->state;
        std::wstring comparison =
            sim->imported ? L"Imported CSV; truth unavailable" : L"Fit / synthetic target";
        std::wstring value = sim->imported
                                 ? std::format(L"{}\nk {:.4f}\nd {:.4f}\ng {:.4f}", comparison, p.x, p.y, p.z)
                                 : std::format(L"{}\nk {:.4f} / {:.2f}\nd {:.4f} / {:.2f}\ng {:.4f} / {:.2f}",
                                               comparison, p.x, s.stiffness, p.y, s.damping, p.z, s.gravity);
        SetWindowTextW(values, value.c_str());
        const wchar_t *names[]{L"Ready", L"Fitting", L"Converged", L"Stopped: inspect residual",
                               L"Insufficient sensitivity / invalid data"};
        int code = std::clamp(static_cast<int>(data.diagnostics[2]), 0, 4);
        auto text = std::format(
            L"{} | {} samples | iteration {:.0f} | MSE {:.7g} | GPU {:.2f} ms | {:.0f} fps\nRMSE {:.5f} | "
            L"clean-reference RMSE {} | normalized Gram determinant {:.3g}\nLoss: log10 MSE [-10,0], "
            L"iterations [0,{:.0f}] | {}",
            names[code], s.samples, data.statistics[2], data.statistics[0], sim->gpuMs, fps, data.quality[0],
            sim->imported ? std::wstring(L"n/a") : std::format(L"{:.5f}", data.quality[1]),
            data.diagnostics[0], std::max(10.f, std::ceil(data.statistics[2] / 10) * 10),
            sim->imported ? L"Imported observations" : L"Synthetic observations");
        SetWindowTextW(status, text.c_str());
    }
};
LRESULT CALLBACK windowProc(HWND hwnd, UINT message, WPARAM w, LPARAM l) {
    auto app = reinterpret_cast<App *>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    if (message == WM_NCCREATE) {
        app = static_cast<App *>(reinterpret_cast<CREATESTRUCTW *>(l)->lpCreateParams);
        app->window = hwnd;
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(app));
    }
    if (app) {
        switch (message) {
        case WM_CLOSE:
            app->running = false;
            return 0;
        case WM_SIZE:
            app->resize();
            return 0;
        case WM_ERASEBKGND:
            return 1;
        case WM_GETMINMAXINFO:
            reinterpret_cast<MINMAXINFO *>(l)->ptMinTrackSize = {1080, 760};
            return 0;
        case WM_COMMAND:
            app->command(LOWORD(w), HIWORD(w));
            return 0;
        case WM_HSCROLL:
            app->updateSliders();
            return 0;
        case WM_RBUTTONDOWN:
            if (app->inCanvas(GET_X_LPARAM(l), GET_Y_LPARAM(l))) {
                app->orbit = true;
                app->previous = {GET_X_LPARAM(l), GET_Y_LPARAM(l)};
                SetCapture(hwnd);
            }
            return 0;
        case WM_LBUTTONUP:
        case WM_RBUTTONUP:
        case WM_CAPTURECHANGED:
            app->stop();
            return 0;
        case WM_MOUSEMOVE:
            app->mouse(GET_X_LPARAM(l), GET_Y_LPARAM(l));
            return 0;
        case WM_MOUSEWHEEL:
            app->camera.distance =
                std::clamp(app->camera.distance * std::exp(-GET_WHEEL_DELTA_WPARAM(w) * .001f), 1.2f, 12.f);
            return 0;
        case WM_CTLCOLORSTATIC:
            SetTextColor(reinterpret_cast<HDC>(w), RGB(30, 40, 55));
            SetBkColor(reinterpret_cast<HDC>(w), RGB(240, 242, 246));
            return reinterpret_cast<LRESULT>(GetSysColorBrush(COLOR_BTNFACE));
        }
    }
    return DefWindowProcW(hwnd, message, w, l);
}

// An independent double-precision model is compiled only as a verification path.
struct D3 {
    double x{}, y{}, z{};
    D3 operator+(D3 b) const {
        return {x + b.x, y + b.y, z + b.z};
    }
    D3 operator-(D3 b) const {
        return {x - b.x, y - b.y, z - b.z};
    }
    D3 operator*(double s) const {
        return {x * s, y * s, z * s};
    }
    double dot(D3 b) const {
        return x * b.x + y * b.y + z * b.z;
    }
    double length() const {
        return std::sqrt(dot(*this));
    }
};
struct Phase {
    std::array<D3, 8> p{}, v{};
};
D3 referenceRest(int i) {
    return {(i & 1) ? .25 : -.25, (i & 2) ? 1. : .5, (i & 4) ? .25 : -.25};
}
Phase derivative(const Phase &state, D3 parameter) {
    Phase result;
    for (int i = 0; i < 6; ++i) {
        result.p[i] = state.v[i];
        result.v[i] = D3{0, -parameter.z, 0} - state.v[i] * parameter.y;
        for (int j = 0; j < 8; ++j)
            if (j != i) {
                D3 d = state.p[j] - state.p[i];
                int bits = 0;
                for (int value = i ^ j; value; value >>= 1)
                    bits += value & 1;
                double weight = bits == 1 ? 1. : (bits == 2 ? .7 : .45), length = d.length(),
                       restLength = (referenceRest(j) - referenceRest(i)).length();
                result.v[i] =
                    result.v[i] + d * (parameter.x * weight * (length - restLength) / std::max(length, 1e-6));
            }
    }
    return result;
}
Phase offset(const Phase &base, const Phase &delta, double scale) {
    Phase result;
    for (int i = 0; i < 8; ++i) {
        result.p[i] = base.p[i] + delta.p[i] * scale;
        result.v[i] = base.v[i] + delta.v[i] * scale;
    }
    return result;
}
std::vector<D3> doubleTrajectory(D3 parameter, int samples, int substeps) {
    Phase state;
    for (int i = 0; i < 8; ++i) {
        D3 p = referenceRest(i);
        state.p[i] = p;
        if (i < 6) {
            state.p[i] = {std::cos(.35) * p.x + std::sin(.35) * p.z + .1, p.y,
                          -std::sin(.35) * p.x + std::cos(.35) * p.z - .06};
            state.v[i] = {.4 * (.5 - p.z), .1 * p.x, .35 + .1 * i / 8};
        }
    }
    std::vector<D3> result;
    result.reserve(samples * 8);
    double h = 1. / (60 * substeps);
    for (int sample = 0; sample < samples; ++sample) {
        if (sample > 0)
            for (int step = 0; step < substeps; ++step) {
                auto k1 = derivative(state, parameter), k2 = derivative(offset(state, k1, h * .5), parameter),
                     k3 = derivative(offset(state, k2, h * .5), parameter),
                     k4 = derivative(offset(state, k3, h), parameter);
                for (int i = 0; i < 8; ++i) {
                    state.p[i] = state.p[i] + (k1.p[i] + k2.p[i] * 2 + k3.p[i] * 2 + k4.p[i]) * (h / 6);
                    state.v[i] = state.v[i] + (k1.v[i] + k2.v[i] * 2 + k3.v[i] * 2 + k4.v[i]) * (h / 6);
                }
            }
        result.insert(result.end(), state.p.begin(), state.p.end());
    }
    return result;
}
bool check(Simulation &sim, std::ostream &out, const std::string &name) {
    sim.measure();
    auto p = sim.parameters();
    bool ok = sim.healthy() && sim.state.statistics[0] < .002f;
    out << name << ": " << (ok ? "PASS" : "FAIL") << " k=" << p.x << " d=" << p.y << " g=" << p.z
        << " MSE=" << sim.state.statistics[0] << " referenceRMSE=" << sim.state.quality[1]
        << " iterations=" << sim.state.statistics[2] << " status=" << sim.state.diagnostics[2]
        << " GramDet=" << sim.state.diagnostics[0] << " GPUms=" << sim.gpuMs << '\n';
    return ok;
}
bool gpuTests(App &app, const std::filesystem::path &folder) {
    std::ofstream out(folder / "Inverse3D-tests.txt");
    out << std::setprecision(10) << app.gl.driver << '\n';
    bool all = true;
    auto &sim = *app.sim;
    auto require = [&](bool ok, const std::string &name) {
        out << name << ": " << (ok ? "PASS" : "FAIL") << '\n';
        out.flush();
        all = all && ok;
    };
    auto pump = [&]() {
        MSG msg{};
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        if (!app.running)
            throw std::runtime_error("Tests cancelled.");
    };
    auto fit = [&]() {
        for (int i = 0; i < 100 && sim.state.diagnostics[2] < 2; ++i) {
            sim.tick();
            pump();
        }
    };
    auto relative = [](V3 p, V3 target) {
        return std::max(
            {std::abs(p.x / target.x - 1), std::abs(p.y / target.y - 1), std::abs(p.z / target.z - 1)});
    };
    auto picture = [&](const std::string &name) {
        app.fit();
        app.draw();
        glFinish();
        capture(folder / (name + ".bmp"), panelWidth, 64, app.width - panelWidth, app.height - 64);
    };
    auto framebuffer = [&]() {
        app.draw();
        glFinish();
        std::vector<unsigned char> bytes(static_cast<size_t>(app.width - panelWidth) * (app.height - 64) * 4);
        glReadPixels(panelWidth, 64, app.width - panelWidth, app.height - 64, GL_RGBA, GL_UNSIGNED_BYTE,
                     bytes.data());
        std::uint64_t h = 1469598103934665603ull;
        for (auto b : bytes) {
            h ^= b;
            h *= 1099511628211ull;
        }
        return h;
    };
    for (int substeps : {4, 8}) {
        sim.settings = Settings{};
        sim.settings.samples = 129;
        sim.settings.substeps = substeps;
        sim.reset();
        auto result = sim.forwardAt({18, .6f, 5.5f});
        auto expected = doubleTrajectory({18, .6f, 5.5f}, 129, substeps);
        double error = 0, anchorError = 0, zMotion = 0;
        for (size_t i = 0; i < result.size(); ++i) {
            D3 delta{result[i][0] - expected[i].x, result[i][1] - expected[i].y,
                     result[i][2] - expected[i].z};
            error = std::max(error, delta.length());
            if (i % 8 >= 6)
                anchorError = std::max(anchorError, delta.length());
            zMotion = std::max(zMotion, std::abs(static_cast<double>(result[i][2] - result[i % 8][2])));
        }
        out << "RK4 substeps=" << substeps << " maximum 3D position error=" << error
            << " anchor error=" << anchorError << '\n';
        require(error < 3e-5 && anchorError < 1e-7, "Independent double-precision 3D trajectory");
        require(zMotion > .1, "Out-of-plane physical motion");
    }
    sim.settings = Settings{};
    sim.settings.samples = 129;
    sim.reset();
    auto p = sim.parameters();
    auto gpu = sim.jacobianTrajectories();
    auto observed = sim.observationSnapshot();
    std::array<std::vector<D3>, 3> jac;
    for (int parameter = 0; parameter < 3; ++parameter) {
        double plus[]{p.x, p.y, p.z}, minus[]{p.x, p.y, p.z};
        plus[parameter] *= std::exp(.002);
        minus[parameter] *= std::exp(-.002);
        auto high = doubleTrajectory({plus[0], plus[1], plus[2]}, 129, 4),
             low = doubleTrajectory({minus[0], minus[1], minus[2]}, 129, 4);
        jac[parameter].resize(sim.size());
        for (int i = 0; i < sim.size(); ++i)
            jac[parameter][i] = (high[i] - low[i]) * (1 / .004);
    }
    double error2 = 0, scale2 = 0;
    for (int j = 0; j < 3; ++j)
        for (int i = 0; i < sim.size(); ++i) {
            auto d = (xyz(gpu[(2 * j + 2) * sim.size() + i]) - xyz(gpu[(2 * j + 1) * sim.size() + i])) *
                     (1 / .004f);
            D3 e{d.x - jac[j][i].x, d.y - jac[j][i].y, d.z - jac[j][i].z};
            error2 += e.dot(e);
            scale2 += jac[j][i].dot(jac[j][i]);
        }
    double relativeJacobian = std::sqrt(error2 / scale2);
    out << "Log-parameter Jacobian relative L2 error=" << relativeJacobian << '\n';
    require(relativeJacobian < .005, "GPU Jacobian versus double finite differences");
    auto normals = sim.normalSnapshot();
    std::array<double, 12> gpuNormal{}, cpuNormal{};
    for (size_t i = 0; i < normals.size(); ++i)
        for (int k = 0; k < 4; ++k)
            gpuNormal[(i % 3) * 4 + k] += normals[i][k];
    auto base = doubleTrajectory({p.x, p.y, p.z}, 129, 4);
    for (int i = 0; i < sim.size(); ++i) {
        double w = observed[i][3];
        auto a = jac[0][i], b = jac[1][i], c = jac[2][i];
        D3 r = base[i] - D3{observed[i][0], observed[i][1], observed[i][2]};
        double v[]{a.dot(a), a.dot(b), a.dot(c), b.dot(b), b.dot(c), c.dot(c),
                   a.dot(r), b.dot(r), c.dot(r), r.dot(r), 1,        0};
        for (int k = 0; k < 12; ++k)
            cpuNormal[k] += w * v[k];
    }
    error2 = 0;
    scale2 = 0;
    for (int k = 0; k < 11; ++k) {
        error2 += std::pow(gpuNormal[k] - cpuNormal[k], 2);
        scale2 += cpuNormal[k] * cpuNormal[k];
    }
    out << "Normal equations relative L2 error=" << std::sqrt(error2 / scale2) << '\n';
    require(std::sqrt(error2 / scale2) < .005 && gpuNormal[11] == 0,
            "GPU reductions and normal equations / partial workgroup");
    for (int scene = 0; scene < 4; ++scene) {
        sim.settings = Settings{};
        sim.settings.scene = scene;
        sim.reset();
        if (scene == 0) {
            sim.time = 1.6;
            picture("Inverse-before-fit");
            sim.time = 0;
        }
        auto initialLoss = sim.state.statistics[0];
        fit();
        all = check(sim, out, "Scene " + std::to_string(scene)) && all;
        auto fitted = sim.parameters();
        require(relative(fitted, {18, .6f, 5.5f}) < (scene == 0 || scene == 3 ? .001f : .025f),
                "Recover synthetic physical parameters");
        auto history = sim.historySnapshot();
        bool monotone = true;
        for (int i = 1; i <= static_cast<int>(sim.state.statistics[2]); ++i)
            monotone = monotone && history[i][0] <= history[i - 1][0] + 1e-8f;
        require(monotone && sim.state.statistics[0] < initialLoss * .02,
                "Accepted objective history is monotone and improves");
        if (scene == 2) {
            auto o = sim.observationSnapshot();
            int used = 0;
            for (auto row : o)
                used += row[3] > 0;
            require(used == 180 && sim.state.quality[1] < .003,
                    "Sparse observations reconstruct unobserved motion");
        }
        if (scene == 3)
            require(std::abs(fitted.y - .6f) < 1e-6 && std::abs(fitted.z - 5.5f) < 1e-6,
                    "Stiffness-only fit preserves supplied damping and gravity");
        sim.time = 1.6;
        picture("Inverse-scene" + std::to_string(scene));
    }
    for (V3 target : {V3{10, .2f, 3}, V3{30, 1.2f, 8}, V3{25, .42f, 7.2f}}) {
        sim.settings = Settings{};
        sim.settings.stiffness = target.x;
        sim.settings.damping = target.y;
        sim.settings.gravity = target.z;
        sim.settings.samples = 360;
        sim.reset();
        fit();
        auto recovered = sim.parameters();
        out << "Range target " << target.x << ',' << target.y << ',' << target.z << " -> " << recovered.x
            << ',' << recovered.y << ',' << recovered.z << '\n';
        require(sim.healthy() && relative(recovered, target) < .003 && sim.state.statistics[0] < 1e-8,
                "Parameter-range recovery / 360 samples");
    }
    sim.settings = Settings{};
    sim.settings.scene = 2;
    sim.settings.noise = .04f;
    sim.settings.substeps = 8;
    sim.settings.samples = 360;
    sim.reset();
    fit();
    require(sim.healthy() && sim.state.statistics[0] < .0008f &&
                relative(sim.parameters(), {18, .6f, 5.5f}) < .08,
            "Maximum noise, sparse data and eight RK4 substeps");
    sim.settings = Settings{};
    sim.settings.scene = 2;
    sim.reset();
    auto deterministic = sim.observationSnapshot();
    sim.reset();
    require(deterministic == sim.observationSnapshot(), "Deterministic noisy observation generation");
    fit();
    auto firstFit = sim.parameters();
    sim.restart();
    fit();
    require(relative(sim.parameters(), firstFit) < 1e-6, "Repeatable GPU fit after restart");
    sim.exportObservations(folder / "Inverse-observations.csv");
    auto data = readObservations(folder / "Inverse-observations.csv");
    require(data.samples == 240 && data.observations == sim.observationSnapshot(),
            "Observation CSV round-trip");
    sim.settings.scene = 0;
    sim.settings.stiffness = 30;
    sim.settings.damping = 1.2f;
    sim.settings.gravity = 8;
    sim.reset(&data);
    fit();
    require(sim.imported && sim.state.quality[1] == -1 && relative(sim.parameters(), firstFit) < .001,
            "Imported fit uses observations, not synthetic truth");
    auto importedFit = sim.parameters();
    for (auto &row : data.observations)
        if (row[3] == 0) {
            row[0] += 2;
            row[1] -= 1;
            row[2] += .75f;
        }
    sim.reset(&data);
    fit();
    require(relative(sim.parameters(), importedFit) < 1e-5,
            "Masked coordinates do not influence the optimizer");
    auto originalData = readObservations(folder / "Inverse-observations.csv");
    sim.reset(&originalData);
    fit();
    sim.exportFit(folder / "Inverse-fit.csv");
    std::ifstream exportFile(folder / "Inverse-fit.csv");
    std::string line;
    int rows = 0;
    while (std::getline(exportFile, line))
        ++rows;
    require(rows == sim.size() + 2, "Reconstructed-motion CSV contains every sample and node");
    auto invalidPath = folder / "Inverse-invalid-test.csv";
    std::array<std::string, 6> invalidRows{"0,0,nan,0,0,1\n", "0,0,0,0,0,2\n",
                                           "8,0,0,0,0,1\n",   "0,0,0,0,0,1\n0,0,0,0,0,1\n",
                                           "0,0,0,0,0,1,7\n", "0,0,0x,0,0,1\n"};
    bool rejected = true;
    for (const auto &row : invalidRows) {
        std::ofstream file(invalidPath);
        file << "# samples=8\nsample,node,x,y,z,weight\n" << row;
        file.close();
        try {
            auto unexpected = readObservations(invalidPath);
            rejected = false;
        } catch (const std::exception &) {
        }
    }
    std::filesystem::remove(invalidPath);
    require(rejected, "CSV rejects nonfinite, range, duplicate and malformed records");
    Dataset empty{16, std::vector<F4>(128)};
    sim.reset(&empty);
    sim.tick();
    require(sim.state.diagnostics[2] == 4 && !sim.healthy(), "Zero-weight dataset halts safely");
    Dataset anchors{16, std::vector<F4>(128)};
    for (int s = 0; s < 16; ++s)
        for (int node = 6; node < 8; ++node) {
            auto p = referenceRest(node);
            anchors.observations[s * 8 + node] = {static_cast<float>(p.x), static_cast<float>(p.y),
                                                  static_cast<float>(p.z), 1};
        }
    sim.reset(&anchors);
    sim.tick();
    require(sim.state.diagnostics[2] == 4, "Uninformative anchored observations are detected");
    sim.settings = Settings{};
    sim.reset();
    auto iteration = sim.state.statistics[2];
    app.command(32, 0);
    app.advanceOne();
    require(app.paused && !app.single && sim.state.statistics[2] == iteration + 1,
            "Native single-iteration control");
    auto currentIteration = sim.state.statistics[2];
    SendMessageW(app.combos[3], CB_SETCURSEL, 1, 0);
    app.command(13, CBN_SELCHANGE);
    require(sim.settings.node == 0 && sim.state.statistics[2] == currentIteration,
            "Trajectory selection preserves the fit");
    SendMessageW(app.sliders[0], TBM_SETPOS, TRUE, 70);
    app.updateSliders();
    require(sim.state.statistics[2] == 0 && std::abs(sim.settings.stiffness - 24) < 1e-6 && !sim.imported,
            "Target slider regenerates observations and restarts estimation");
    sim.settings = Settings{};
    sim.reset();
    fit();
    sim.time = 1.6;
    app.fit();
    auto full = framebuffer();
    app.camera.yaw += .7f;
    require(framebuffer() != full, "Perspective camera orbit changes projection");
    app.fit();
    sim.settings.trails = false;
    require(framebuffer() != full, "Trajectory display toggle");
    sim.settings.trails = true;
    sim.settings.node = 0;
    require(framebuffer() != full, "Individual-node trajectory view");
    picture("Inverse-selected-node");
    sim.settings = Settings{};
    sim.reset();
    app.fit();
    app.updateStatus();
    app.draw();
    glFinish();
    require(glGetError() == GL_NO_ERROR, "OpenGL error state");
    out << (all ? "ALL PASSED" : "FAILED") << '\n';
    return all;
}
} // namespace inverse
int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR args, int show) {
    using namespace inverse;
    std::wstring command = args;
    bool tests = command.find(L"--gpu-test") != std::wstring::npos,
         smoke = command.find(L"--smoke") != std::wstring::npos;
    auto folder = std::filesystem::current_path();
    try {
        SetProcessDPIAware();
        INITCOMMONCONTROLSEX cc{sizeof(cc), ICC_BAR_CLASSES};
        InitCommonControlsEx(&cc);
        App app;
        WNDCLASSEXW wc{};
        wc.cbSize = sizeof(wc);
        wc.style = CS_OWNDC;
        wc.lpfnWndProc = windowProc;
        wc.hInstance = instance;
        wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
        wc.hbrBackground = GetSysColorBrush(COLOR_BTNFACE);
        wc.lpszClassName = L"Inverse3DGPU";
        if (!RegisterClassExW(&wc))
            throw std::runtime_error("Window registration failed.");
        auto hwnd = CreateWindowExW(0, wc.lpszClassName, L"Inverse Physics Lab / 3D GPU / OpenGL 4.3",
                                    WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN, CW_USEDEFAULT, CW_USEDEFAULT, 1360,
                                    1000, nullptr, nullptr, instance, &app);
        if (!hwnd)
            throw std::runtime_error("Window creation failed.");
        app.gl.initialize(hwnd);
        app.sim = std::make_unique<Simulation>(app.gl);
        app.controls();
        modern::apply(hwnd, panelWidth, 286, L"Inverse Physics Lab", L"Recover parameters from motion");
        app.resize();
        ShowWindow(hwnd, (tests || smoke) ? SW_SHOWNOACTIVATE : show);
        if (tests)
            return gpuTests(app, folder) ? 0 : 1;
        using Clock = std::chrono::steady_clock;
        auto last = Clock::now(), report = last;
        double elapsed = 0;
        int frames = 0, counter = 0;
        while (app.running) {
            MSG msg{};
            while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
                if (msg.message == WM_QUIT)
                    app.running = false;
                if (msg.message == WM_KEYDOWN && msg.hwnd == hwnd) {
                    if (msg.wParam == VK_ESCAPE)
                        app.running = false;
                    if (msg.wParam == VK_SPACE) {
                        app.toggle();
                        continue;
                    }
                    if (msg.wParam == 'R') {
                        app.command(31, 0);
                        continue;
                    }
                    if (msg.wParam == 'N') {
                        app.command(32, 0);
                        continue;
                    }
                }
                if (IsDialogMessageW(hwnd, &msg))
                    continue;
                TranslateMessage(&msg);
                DispatchMessageW(&msg);
            }
            if (!app.running)
                break;
            auto now = Clock::now();
            elapsed = std::min(.05, elapsed + std::chrono::duration<double>(now - last).count());
            last = now;
            if (IsIconic(hwnd)) {
                Sleep(30);
                elapsed = 0;
                continue;
            }
            if (smoke)
                app.sim->tick();
            else if (app.single) {
                app.advanceOne();
                elapsed = 0;
            } else if (!app.paused && elapsed >= 1. / 60) {
                app.sim->tick();
                elapsed = 0;
            } else if (app.paused)
                elapsed = 0;
            app.draw();
            ++frames;
            ++counter;
            if (smoke && frames == 180) {
                std::ofstream out(folder / "Lab07_smoke.txt");
                out << app.gl.driver << "\n";
                bool ok = check(*app.sim, out, "3D inverse physics smoke");
                capture(folder / "Lab07.bmp", panelWidth, 64, app.width - panelWidth, app.height - 64);
                auto error = glGetError();
                out << "OpenGL error: " << error << "\n";
                return ok && error == 0 ? 0 : 1;
            }
            SwapBuffers(app.gl.dc);
            if (now - report > std::chrono::milliseconds(500)) {
                app.fps = counter / static_cast<float>(std::chrono::duration<double>(now - report).count());
                counter = 0;
                report = now;
                app.updateStatus();
            }
            Sleep(1);
        }
        return 0;
    } catch (const std::exception &error) {
        if (tests || smoke) {
            std::ofstream out(folder / "Inverse3D-error.txt");
            out << error.what();
        } else
            MessageBoxA(nullptr, error.what(), "Inverse Physics Lab 3D", MB_ICONERROR);
        return 1;
    }
}
