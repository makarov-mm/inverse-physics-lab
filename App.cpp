#include "App.hpp"
#include "ModernUI.hpp"
#include "ObservationIO.hpp"
#include <commctrl.h>
#include <commdlg.h>
#include <windowsx.h>
#include <algorithm>
#include <cmath>
#include <format>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <utility>

namespace inverse {
    namespace {
        constexpr int margin = 20, contentWidth = 280;      // panel content spans x = 20..300
        constexpr int buttonWidth = 130, buttonHeight = 32; // two buttons per row: 20..150 and 170..300
        constexpr float orbitSpeed = .007f, zoomSpeed = .001f, minDistance = 1.2f, maxDistance = 12.f;
        constexpr std::array sampleOptions{ 120, 240, 360 };
        constexpr std::array substepOptions{ 4, 8 };

        int comboIndex(Id id) {
            return std::to_underlying(id) - std::to_underlying(Id::experiment);
        }
        bool isChecked(HWND box) {
            return SendMessageW(box, BM_GETCHECK, 0, 0) == BST_CHECKED;
        }

        LRESULT CALLBACK windowProc(HWND hwnd, UINT message, WPARAM w, LPARAM l) {
            auto* app = reinterpret_cast<App*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
            if (message == WM_NCCREATE) {
                app = static_cast<App*>(reinterpret_cast<CREATESTRUCTW*>(l)->lpCreateParams);
                app->window = hwnd;
                SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(app));
            }
            if (message == WM_GETMINMAXINFO) {
                reinterpret_cast<MINMAXINFO*>(l)->ptMinTrackSize = { minimumWidth, minimumHeight };
                return 0;
            }
            if (!app)
                return DefWindowProcW(hwnd, message, w, l);
            const int x = GET_X_LPARAM(l), y = GET_Y_LPARAM(l);
            switch (message) {
            case WM_CLOSE:
                app->running = false;
                return 0;
            case WM_SIZE:
                app->resize();
                return 0;
            case WM_ERASEBKGND:
                return 1; // the canvas and the panel are painted by OpenGL and the themed controls
            case WM_COMMAND:
                app->command(static_cast<Id>(LOWORD(w)), HIWORD(w));
                return 0;
            case WM_HSCROLL:
                app->readSliders();
                return 0;
            case WM_RBUTTONDOWN:
                if (app->inCanvas(x, y)) {
                    app->orbiting = true;
                    app->previous = { x, y };
                    SetCapture(hwnd);
                }
                return 0;
            case WM_LBUTTONUP:
            case WM_RBUTTONUP:
            case WM_CAPTURECHANGED:
                app->stopInteraction();
                return 0;
            case WM_MOUSEMOVE:
                app->mouse(x, y);
                return 0;
            case WM_MOUSEWHEEL:
                app->camera.distance = std::clamp(app->camera.distance * std::exp(-GET_WHEEL_DELTA_WPARAM(w) * zoomSpeed),
                    minDistance, maxDistance);
                return 0;
            }
            return DefWindowProcW(hwnd, message, w, l);
        }
    } // namespace

    const std::array<SliderSpec, sliderCount> sliderSpecs{ {
        { L"Target stiffness", 10, 30, 40, L"{:.2f}", &Settings::stiffness },
        { L"Target damping", .2f, 1.2f, 40, L"{:.2f}", &Settings::damping },
        { L"Target gravity", 3, 8, 50, L"{:.2f}", &Settings::gravity },
        { L"Noise half-width", 0, .04f, 30, L"{:.4f}", &Settings::noise },
    } };

    App::~App() {
        sim.reset();    // GPU objects need the context
        gpu.shutdown(); // the context needs the window's DC
        if (window)
            DestroyWindow(window);
    }

    void App::create(HINSTANCE instance) {
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
        if (!CreateWindowExW(0, wc.lpszClassName, appTitle, WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN, CW_USEDEFAULT,
            CW_USEDEFAULT, initialWidth, initialHeight, nullptr, nullptr, instance, this))
            throw std::runtime_error("Window creation failed.");
        gpu.initialize(window);
        sim = std::make_unique<Simulation>(gpu);
        createControls();
        modern::apply(window, { .sidebarWidth = panelWidth,
                               .designWidth = panelWidth,
                               .title = appTitle,
                               .subtitle = appSubtitle,
                               .primaryButtonId = std::to_underlying(Id::pause) });
        resize();
    }

    void App::createControls() {
        auto control = [&](const wchar_t* kind, const wchar_t* text, DWORD style, int x, int y, int w, int h,
            int id = 0) {
                return CreateWindowExW(0, kind, text, WS_CHILD | WS_VISIBLE | style, x, y, w, h, window,
                    reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)), GetModuleHandleW(nullptr),
                    nullptr);
            };
        auto combo = [&](const wchar_t* label, int y, Id id, std::initializer_list<const wchar_t*> items,
            int initial) {
                control(L"STATIC", label, 0, margin, y, contentWidth, 21);
                HWND c = control(L"COMBOBOX", L"", CBS_DROPDOWNLIST | WS_TABSTOP, margin, y + 22, contentWidth, 210,
                    std::to_underlying(id));
                for (auto item : items)
                    SendMessageW(c, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(item));
                SendMessageW(c, CB_SETCURSEL, static_cast<WPARAM>(initial), 0);
                return c;
            };
        auto checkbox = [&](const wchar_t* text, int y, Id id, bool checked) {
            HWND box = control(L"BUTTON", text, BS_AUTOCHECKBOX | WS_TABSTOP, margin, y, contentWidth, 25,
                std::to_underlying(id));
            SendMessageW(box, BM_SETCHECK, checked ? BST_CHECKED : BST_UNCHECKED, 0);
            return box;
            };
        auto button = [&](const wchar_t* text, int x, int y, int w, Id id) {
            return control(L"BUTTON", text, BS_PUSHBUTTON | WS_TABSTOP, x, y, w, buttonHeight, std::to_underlying(id));
            };

        control(L"STATIC", appTitle, 0, margin, 14, contentWidth, 27);
        control(L"STATIC", appSubtitle, 0, margin, 43, contentWidth, 24);
        combos[comboIndex(Id::experiment)] =
            combo(L"Synthetic experiment (new data)", 76, Id::experiment,
                { L"Fit k, damping and gravity", L"Noisy 3D observations", L"Sparse and noisy observations",
                 L"Stiffness only (d, g known)" },
                0);
        combos[comboIndex(Id::samples)] = combo(L"Samples at 60 Hz (new data)", 136, Id::samples,
            { L"120 / 1.98 seconds", L"240 / 3.98 seconds", L"360 / 5.98 seconds" }, 1);
        combos[comboIndex(Id::substeps)] =
            combo(L"RK4 substeps (restarts fit)", 196, Id::substeps, { L"4 / balanced", L"8 / accurate" }, 0);
        combos[comboIndex(Id::node)] = combo(L"Displayed observation trajectories", 256, Id::node,
            { L"All nodes", L"Node 0", L"Node 1", L"Node 2", L"Node 3", L"Node 4",
             L"Node 5", L"Anchor 6", L"Anchor 7" },
            0);
        for (int i = 0; i < sliderCount; ++i) {
            const int y = 332 + i * 52;
            labels[i] = control(L"STATIC", L"", 0, margin, y, contentWidth, 22);
            sliders[i] = control(TRACKBAR_CLASSW, L"", TBS_HORZ | WS_TABSTOP, margin - 6, y + 22, contentWidth + 12, 29,
                std::to_underlying(Id::slider) + i);
            SendMessageW(sliders[i], TBM_SETRANGE, TRUE, MAKELPARAM(0, 100));
            SendMessageW(sliders[i], TBM_SETPOS, TRUE, sliderSpecs[i].initial);
        }
        trailsBox = checkbox(L"Show observation / fitted paths", 554, Id::trails, true);
        referenceBox = checkbox(L"Show clean synthetic body", 582, Id::reference, true);
        seedBox = checkbox(L"GPU seed search (on restart)", 610, Id::seedSearch, true);
        values = control(L"STATIC", L"", 0, margin, 645, contentWidth, 85);
        const int secondColumn = margin + contentWidth - buttonWidth;
        pauseButton = button(L"Pause", margin, 742, buttonWidth, Id::pause);
        button(L"Restart fit", secondColumn, 742, buttonWidth, Id::restart);
        button(L"Single step", margin, 784, buttonWidth, Id::step);
        button(L"Fit view", secondColumn, 784, buttonWidth, Id::fit);
        button(L"Import CSV...", margin, 826, buttonWidth, Id::importCsv);
        button(L"Export data...", secondColumn, 826, buttonWidth, Id::exportObservations);
        control(L"BUTTON", L"Export reconstructed motion...", BS_PUSHBUTTON | WS_TABSTOP, margin, 868, contentWidth, 30,
            std::to_underlying(Id::exportFit));
        control(L"STATIC", L"Cyan: data  Orange: fit\nRMB: orbit   Wheel: zoom\nSpace: pause   R: restart   N: step", 0,
            margin, 908, contentWidth, 52);
        status = control(L"STATIC", L"Initializing...", 0, panelWidth + 12, height - 60, width - panelWidth - 24, 58);
        readSliders();
        fit();
        updateStatus();
    }

    void App::fit() {
        camera = Camera{ .yaw = .65f, .pitch = .35f, .distance = 3.8f, .target = { 0, .45f, 0 } };
    }

    void App::readSliders() {
        if (!sim || sim->imported())
            return;
        auto& s = sim->settings;
        const Settings before = s;
        for (int i = 0; i < sliderCount; ++i) {
            const auto& spec = sliderSpecs[i];
            const int position = static_cast<int>(SendMessageW(sliders[i], TBM_GETPOS, 0, 0));
            float value = spec.low + (spec.high - spec.low) * (static_cast<float>(position) / 100);
            s.*spec.target = value;
            const auto text = std::format(L"{}: {}", spec.label, std::vformat(spec.format, std::make_wformat_args(value)));
            SetWindowTextW(labels[i], text.c_str());
        }
        // Synthetic-data sliders regenerate the observations and restart the estimate.
        if (before.stiffness != s.stiffness || before.damping != s.damping || before.gravity != s.gravity ||
            before.noise != s.noise) {
            sim->reset();
            updateStatus();
        }
    }

    void App::resize() {
        RECT r{};
        GetClientRect(window, &r);
        width = r.right;
        height = r.bottom;
        if (status)
            MoveWindow(status, panelWidth + 12, height - 60, std::max(1, width - panelWidth - 24), 58, TRUE);
    }

    void App::togglePause() {
        paused = !paused;
        SetWindowTextW(pauseButton, paused ? L"Resume" : L"Pause");
    }

    void App::advanceOne() {
        sim->tick();
        single = false;
        updateStatus();
    }

    void App::syncDataSource() {
        const bool synthetic = !sim->imported();
        for (HWND slider : sliders)
            EnableWindow(slider, synthetic);
        EnableWindow(combos[comboIndex(Id::samples)], synthetic);
        EnableWindow(referenceBox, synthetic);
    }

    void App::command(Id id, int code) {
        if (!sim)
            return;
        auto& s = sim->settings;
        switch (id) {
        case Id::experiment:
        case Id::samples:
        case Id::substeps:
        case Id::node: {
            if (code != CBN_SELCHANGE)
                return;
            const int v = static_cast<int>(SendMessageW(combos[static_cast<size_t>(comboIndex(id))], CB_GETCURSEL, 0, 0));
            if (v < 0)
                return;
            if (id == Id::experiment) { // a synthetic experiment always leaves imported-data mode
                s.scene = static_cast<Scene>(v);
                const auto samples = SendMessageW(combos[comboIndex(Id::samples)], CB_GETCURSEL, 0, 0);
                s.samples = sampleOptions[static_cast<size_t>(std::max<LRESULT>(0, samples))];
                sim->reset();
                syncDataSource();
            }
            if (id == Id::samples) {
                s.samples = sampleOptions[static_cast<size_t>(v)];
                sim->reset();
            }
            if (id == Id::substeps) {
                s.substeps = substepOptions[static_cast<size_t>(v)];
                sim->restart();
            }
            if (id == Id::node)
                s.node = v == 0 ? allNodes : v - 1;
            updateStatus();
            return;
        }
        case Id::pause:
            togglePause();
            return;
        case Id::restart:
            sim->restart();
            single = false;
            updateStatus();
            return;
        case Id::step:
            paused = true;
            single = true;
            SetWindowTextW(pauseButton, L"Resume");
            return;
        case Id::fit:
            stopInteraction();
            fit();
            return;
        case Id::trails:
            s.trails = isChecked(trailsBox);
            return;
        case Id::reference:
            s.reference = isChecked(referenceBox);
            return;
        case Id::seedSearch:
            s.seedSearch = isChecked(seedBox);
            return;
        case Id::importCsv:
        case Id::exportObservations:
        case Id::exportFit:
            fileDialog(id);
            return;
        default:
            return;
        }
    }

    bool App::key(WPARAM pressed) {
        switch (pressed) {
        case VK_ESCAPE:
            running = false;
            return true;
        case VK_SPACE:
            togglePause();
            return true;
        case 'R':
            command(Id::restart, 0);
            return true;
        case 'N':
            command(Id::step, 0);
            return true;
        default:
            return false;
        }
    }

    bool App::pump() {
        MSG msg{};
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT)
                running = false;
            if (msg.message == WM_KEYDOWN && msg.hwnd == window && key(msg.wParam))
                continue; // simulation shortcuts apply while the canvas has focus
            if (IsDialogMessageW(window, &msg))
                continue; // Tab navigation between the panel controls
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        return running;
    }

    Viewport App::canvas() const {
        return { panelWidth, statusHeight, std::max(1, width - panelWidth), std::max(1, height - statusHeight) };
    }

    Viewport App::world() const {
        return { panelWidth, statusHeight + plotHeight, std::max(1, width - panelWidth),
                std::max(1, height - statusHeight - plotHeight) };
    }

    Viewport App::plot() const {
        return { panelWidth, statusHeight, std::max(1, width - panelWidth), plotHeight };
    }

    bool App::inCanvas(int x, int y) const {
        return x > panelWidth && x < width && y >= 0 && y < height - statusHeight - plotHeight;
    }

    void App::mouse(int x, int y) {
        if (orbiting) {
            camera.yaw += static_cast<float>(x - previous.x) * orbitSpeed;
            camera.pitch = std::clamp(camera.pitch + static_cast<float>(y - previous.y) * orbitSpeed, -maxPitch, maxPitch);
        }
        previous = { x, y };
    }

    void App::stopInteraction() {
        orbiting = false;
        if (GetCapture() == window)
            ReleaseCapture();
    }

    void App::draw() {
        glViewport(0, 0, width, height);
        glClearColor(.012f, .018f, .032f, 1);
        glClear(GL_COLOR_BUFFER_BIT);
        // The header above the settings panel is painted by OpenGL too.
        glEnable(GL_SCISSOR_TEST);
        glScissor(0, 0, panelWidth, height);
        glClearColor(246.f / 255, 248.f / 255, 252.f / 255, 1);
        glClear(GL_COLOR_BUFFER_BIT);
        glDisable(GL_SCISSOR_TEST);
        sim->draw(camera, world(), plot());
    }

    void App::updateStatus() {
        const V3 p = sim->parameters();
        const auto& s = sim->settings;
        const auto& state = sim->state;
        const bool imported = sim->imported();
        const std::wstring estimate =
            imported ? std::format(L"Imported CSV; truth unavailable\nk {:.4f}\nd {:.4f}\ng {:.4f}", p.x, p.y, p.z)
            : std::format(L"Fit / synthetic target\nk {:.4f} / {:.2f}\nd {:.4f} / {:.2f}\ng {:.4f} / {:.2f}",
                p.x, s.stiffness, p.y, s.damping, p.z, s.gravity);
        SetWindowTextW(values, estimate.c_str());
        constexpr std::array<const wchar_t*, 5> names{ L"Ready", L"Fitting", L"Converged", L"Stopped: inspect residual",
                                                       L"Insufficient sensitivity / invalid data" };
        const auto code = static_cast<size_t>(std::clamp(std::to_underlying(state.status()), 0, 4));
        const float horizontal = std::max(10.f, std::ceil(state.iterations() / 10) * 10);
        const auto text = std::format(
            L"{} | {} samples | iteration {:.0f} | MSE {:.7g} | GPU {:.2f} ms | {:.0f} fps\n"
            L"RMSE {:.5f} | clean-reference RMSE {} | normalized Gram determinant {:.3g}\n"
            L"Loss: log10 MSE [-10,0], iterations [0,{:.0f}] | {}",
            names[code], s.samples, state.iterations(), state.mse(), sim->gpuMs, fps, state.rmse(),
            imported ? std::wstring(L"n/a") : std::format(L"{:.5f}", state.referenceRmse()), state.gramDeterminant(),
            horizontal, imported ? L"Imported observations" : L"Synthetic observations");
        SetWindowTextW(status, text.c_str());
    }

    void App::fileDialog(Id id) {
        const bool import = id == Id::importCsv;
        wchar_t filename[MAX_PATH]{};
        if (id == Id::exportObservations)
            wcscpy_s(filename, L"Inverse-observations.csv");
        if (id == Id::exportFit)
            wcscpy_s(filename, L"Inverse-fit.csv");
        OPENFILENAMEW dialog{};
        dialog.lStructSize = sizeof(dialog);
        dialog.hwndOwner = window;
        dialog.lpstrFilter = L"CSV files\0*.csv\0All files\0*.*\0";
        dialog.lpstrFile = filename;
        dialog.nMaxFile = MAX_PATH;
        dialog.lpstrDefExt = L"csv";
        dialog.Flags = OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR | (import ? OFN_FILEMUSTEXIST : OFN_OVERWRITEPROMPT);
        if (!(import ? GetOpenFileNameW(&dialog) : GetSaveFileNameW(&dialog)))
            return;
        try {
            switch (id) {
            case Id::importCsv:
                sim->reset(readObservations(filename));
                syncDataSource();
                updateStatus();
                break;
            case Id::exportObservations:
                sim->exportObservations(filename);
                break;
            case Id::exportFit:
                sim->exportFit(filename);
                break;
            default:
                break;
            }
        }
        catch (const std::exception& e) {
            MessageBoxA(window, e.what(), "Inverse Physics CSV", MB_ICONERROR);
        }
    }
}
