#pragma once
// Native window, settings panel, camera interaction, CSV dialogs and the status line.
#include "GpuApi.hpp"
#include "Math3D.hpp"
#include "Model.hpp"
#include "Simulation.hpp"
#include <array>
#include <memory>

namespace inverse {
    constexpr int panelWidth = 320;   // settings column
    constexpr int statusHeight = 64;  // status strip under the plot
    constexpr int plotHeight = 170;   // convergence plot above the status strip
    constexpr int sliderCount = 4, comboCount = 4;
    constexpr int initialWidth = 1360, initialHeight = 1000, minimumWidth = 1080, minimumHeight = 760;
    constexpr const wchar_t* appTitle = L"Inverse Physics Lab", * appSubtitle = L"Recover parameters from motion";

    /// Control identifiers. Combos and sliders are consecutive so the index is id - first.
    enum class Id : int {
        experiment = 10, samples, substeps, node, //
        slider = 20,                              // slider + 0..3
        pause = 30, restart, step, fit, importCsv, exportObservations, exportFit,
        trails = 40, reference, seedSearch
    };

    /// One synthetic-data slider: linear map from the 0..100 trackbar position to [low, high].
    struct SliderSpec {
        const wchar_t* label;
        float low, high;
        int initial;             // trackbar position 0..100
        const wchar_t* format;   // std::format specification for the value
        float Settings::* target; // the Settings member driven by the slider
    };
    extern const std::array<SliderSpec, sliderCount> sliderSpecs;

    struct App {
        GpuApi gpu;
        std::unique_ptr<Simulation> sim;
        Camera camera;
        HWND window{}, status{}, values{}, pauseButton{};
        std::array<HWND, comboCount> combos{};
        std::array<HWND, sliderCount> sliders{}, labels{};
        HWND trailsBox{}, referenceBox{}, seedBox{};
        int width = initialWidth, height = initialHeight; // client size
        bool running = true, paused = false, single = false, orbiting = false;
        POINT previous{}; // last mouse position while orbiting
        float fps = 0;

        App() = default;
        App(const App&) = delete;
        App& operator=(const App&) = delete;
        ~App();

        void create(HINSTANCE instance);
        void createControls();
        void fit();
        void readSliders();
        void resize();
        void togglePause();
        void advanceOne();
        void command(Id id, int code);
        bool key(WPARAM pressed);
        /// Dispatches pending window messages; returns false once the application should exit.
        bool pump();
        Viewport canvas() const; // scene and plot together, for captures
        Viewport world() const;  // the 3D scene
        Viewport plot() const;   // the convergence plot
        bool inCanvas(int x, int y) const;
        void mouse(int x, int y);
        void stopInteraction();
        void syncDataSource(); // enables the synthetic-data controls only for synthetic data
        void draw();
        void updateStatus();
        void fileDialog(Id id);
    };
}
