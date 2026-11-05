#include "Lab.hpp"
#include <format>
namespace physics {
namespace {
class Inverse final : public Lab {
    static constexpr int count = 360;
    std::array<float, count> target{}, prediction{};
    float stiffness = 4, damping = .15f, loss = 1;
    int iteration{};
    float anim{};
    std::array<float, count> trajectory(float k, float d) const {
        std::array<float, count> result{};
        float x = .75f, v = 0;
        for (auto &value : result) {
            for (int sub = 0; sub < 4; ++sub) {
                constexpr float h = 1.f / 240;
                v += (-k * x - d * v) * h;
                x += v * h;
            }
            value = x;
        }
        return result;
    }
    float objective(float k, float d) const {
        auto y = trajectory(k, d);
        float sum = 0;
        for (int i = 0; i < count; ++i) {
            float e = y[i] - target[i];
            sum += e * e;
        }
        return sum / count;
    }
    void generateTarget() {
        float k = 6 + parameters[0] * 12, d = .2f + parameters[1] * 1.4f;
        target = trajectory(k, d);
        if (mode == 1) {
            std::mt19937 rng(333);
            for (float &v : target)
                v += (randomUnit(rng) - .5f) * .06f;
        }
        if (mode == 2)
            damping = d;
    }

  public:
    Inverse() {
        reset();
    }
    Description describe() const override {
        return {L"07 / Inverse Physics Lab",
                L"Recover spring stiffness and damping from a target motion trace.",
                {L"Fit two parameters", L"Noisy observations", L"Fit stiffness only"},
                {L"Target stiffness", L"Target damping"},
                L"Cyan: target observations\nOrange: fitted simulation\nChanging a slider updates the "
                L"target.\nReset restarts parameter estimation."};
    }
    void reset() override {
        stiffness = 4;
        damping = .15f;
        iteration = 0;
        anim = 0;
        generateTarget();
        prediction = trajectory(stiffness, damping);
        loss = objective(stiffness, damping);
    }
    void step(float dt, const Input &) override {
        generateTarget();
        anim += dt;
        for (int step = 0; step < 8; ++step) {
            constexpr float eps = .002f;
            auto base = trajectory(stiffness, damping), kp = trajectory(stiffness + eps, damping),
                 dp = trajectory(stiffness, damping + eps);
            double aa = 1e-5, ab = 0, bb = 1e-5, ga = 0, gb = 0;
            for (int i = 0; i < count; ++i) {
                double a = (kp[i] - base[i]) / eps, b = (dp[i] - base[i]) / eps, r = base[i] - target[i];
                aa += a * a;
                ab += a * b;
                bb += b * b;
                ga += a * r;
                gb += b * r;
            }
            double det = aa * bb - ab * ab;
            float dk = static_cast<float>((bb * ga - ab * gb) / det),
                  dd = static_cast<float>((aa * gb - ab * ga) / det);
            if (mode == 2) {
                dk = static_cast<float>(ga / aa);
                dd = 0;
            }
            float old = objective(stiffness, damping);
            for (float scale = 1; scale > .001f; scale *= .5f) {
                float k = std::clamp(stiffness - scale * dk, .2f, 30.f),
                      d = std::clamp(damping - scale * dd, .01f, 4.f);
                if (objective(k, d) < old) {
                    stiffness = k;
                    damping = d;
                    break;
                }
            }
            ++iteration;
        }
        prediction = trajectory(stiffness, damping);
        loss = objective(stiffness, damping);
    }
    void render(Canvas &c) const override {
        c.clear();
        for (int j = 1; j < 10; ++j)
            c.line({.05f, j * .08f + .1f}, {.95f, j * .08f + .1f}, {.06f, .09f, .13f});
        c.line({.05f, .5f}, {.95f, .5f}, {.2f, .25f, .3f});
        for (int i = 1; i < count; ++i) {
            float x = .05f + .9f * i / (count - 1), px = .05f + .9f * (i - 1) / (count - 1);
            c.line({px, .5f + target[i - 1] * .4f}, {x, .5f + target[i] * .4f}, {.2f, .85f, .95f});
            c.line({px, .5f + prediction[i - 1] * .4f}, {x, .5f + prediction[i] * .4f}, {1, .55f, .2f});
        }
        int frame = static_cast<int>(anim * 60) % count;
        float x = .05f + .9f * frame / (count - 1);
        c.line({x, .12f}, {x, .88f}, {.3f, .35f, .4f});
        c.dot({x, .5f + prediction[frame] * .4f}, .009f, {1, .7f, .3f});
    }
    std::wstring status() const override {
        return std::format(L"k {:.3f} / {:.3f} | damping {:.3f} / {:.3f} | MSE {:.7f}", stiffness,
                           6 + parameters[0] * 12, damping, .2f + parameters[1] * 1.4f, loss);
    }
    bool healthy() const override {
        return finite(loss) && finite(stiffness) && finite(damping) && stiffness > 0 && damping > 0;
    }
    double validationError() const override {
        return loss;
    }
};
} // namespace
std::unique_ptr<Lab> makeInverse() {
    return std::make_unique<Inverse>();
}
} // namespace physics
