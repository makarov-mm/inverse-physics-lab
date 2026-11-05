// Entry point: window setup and the main loop.
#include "App.hpp"
#include <commctrl.h>
#include <algorithm>
#include <chrono>
#include <exception>

namespace {
    using namespace inverse;
    using Clock = std::chrono::steady_clock;

    constexpr double tick = 1.0 / sampleRate; // one playback sample, and at most one optimizer iteration
    constexpr auto statusInterval = std::chrono::milliseconds(500);

    int run(App& app) {
        auto last = Clock::now(), report = last;
        double backlog = 0; // real seconds not yet played back, capped so a slow fit never bursts
        int framesSinceReport = 0;
        while (app.pump()) {
            const auto now = Clock::now();
            backlog = std::min(backlog + std::chrono::duration<double>(now - last).count(), 2 * tick);
            last = now;
            if (IsIconic(app.window)) {
                Sleep(30);
                backlog = 0;
                continue;
            }
            if (app.single) {
                app.advanceOne();
                backlog = 0;
            }
            else if (app.paused) {
                backlog = 0;
            }
            else if (backlog >= tick) {
                app.sim->tick(); // one iteration per frame at most: slow devices slow the schedule
                backlog -= tick;
            }
            app.draw();
            ++framesSinceReport;
            SwapBuffers(app.gpu.dc);
            if (now - report > statusInterval) {
                app.fps = static_cast<float>(framesSinceReport / std::chrono::duration<double>(now - report).count());
                framesSinceReport = 0;
                report = now;
                app.updateStatus();
            }
            if (!app.gpu.vsync)
                Sleep(1); // without vertical sync, avoid spinning a core at full speed
        }
        return 0;
    }
}

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int show) {
    try {
        SetProcessDPIAware();
        INITCOMMONCONTROLSEX controls{ sizeof(controls), ICC_BAR_CLASSES };
        InitCommonControlsEx(&controls);
        App app;
        app.create(instance);
        ShowWindow(app.window, show); // the window is created hidden; without this call it never appears
        return run(app);
    }
    catch (const std::exception& error) {
        MessageBoxA(nullptr, error.what(), "Inverse Physics Lab 3D", MB_ICONERROR);
        return 1;
    }
}
