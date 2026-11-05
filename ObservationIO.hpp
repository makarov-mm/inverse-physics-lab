#pragma once
#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>
namespace inverse {
using F4 = std::array<float, 4>;
struct Dataset {
    int samples = 0;
    std::vector<F4> observations;
};
inline std::string_view trim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r'))
        s.remove_prefix(1);
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r'))
        s.remove_suffix(1);
    return s;
}
template <class T> T number(std::string_view text) {
    T result{};
    text = trim(text);
    auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), result);
    if (error != std::errc{} || end != text.data() + text.size())
        throw std::runtime_error("Invalid CSV number.");
    return result;
}
inline Dataset readObservations(const std::filesystem::path &path) {
    if (std::filesystem::file_size(path) > 4 * 1024 * 1024)
        throw std::runtime_error("Observation CSV exceeds 4 MiB.");
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("Cannot open observation CSV.");
    Dataset data;
    std::vector<bool> present;
    bool header = false;
    std::string line;
    int lineNumber = 0, usable = 0, first = 10000, last = -1;
    while (std::getline(in, line)) {
        ++lineNumber;
        if (lineNumber == 1 && line.starts_with("\xEF\xBB\xBF"))
            line.erase(0, 3);
        auto text = trim(line);
        if (text.empty())
            continue;
        try {
            if (text.starts_with("# samples=")) {
                if (data.samples != 0)
                    throw std::runtime_error("Duplicate sample count.");
                data.samples = number<int>(text.substr(10));
                if (data.samples < 8 || data.samples > 360)
                    throw std::runtime_error("Sample count must be 8..360.");
                data.observations.resize(data.samples * 8);
                present.resize(data.samples * 8);
                continue;
            }
            if (text.front() == '#')
                continue;
            if (!header) {
                if (text != "sample,node,x,y,z,weight" || data.samples == 0)
                    throw std::runtime_error("Expected '# samples=N' and 'sample,node,x,y,z,weight'.");
                header = true;
                continue;
            }
            std::array<std::string_view, 6> fields{};
            for (int i = 0; i < 6; ++i) {
                auto comma = text.find(',');
                if ((i < 5 && comma == std::string_view::npos) || (i == 5 && comma != std::string_view::npos))
                    throw std::runtime_error("Expected six CSV columns.");
                fields[i] = text.substr(0, comma);
                if (i < 5)
                    text.remove_prefix(comma + 1);
            }
            int sample = number<int>(fields[0]), node = number<int>(fields[1]);
            if (sample < 0 || sample >= data.samples || node < 0 || node >= 8)
                throw std::runtime_error("Sample/node index outside the declared range.");
            int id = sample * 8 + node;
            if (present[id])
                throw std::runtime_error("Duplicate sample/node row.");
            present[id] = true;
            F4 p{};
            for (int i = 0; i < 4; ++i) {
                p[i] = number<float>(fields[i + 2]);
                if (!std::isfinite(p[i]))
                    throw std::runtime_error("All values must be finite.");
            }
            if (std::abs(p[0]) > 100 || std::abs(p[1]) > 100 || std::abs(p[2]) > 100 || p[3] < 0 || p[3] > 1)
                throw std::runtime_error("Coordinates must be within +/-100 and weights within 0..1.");
            if (node >= 6)
                p[3] = 0;
            data.observations[id] = p;
            if (p[3] > 0) {
                ++usable;
                first = std::min(first, sample);
                last = std::max(last, sample);
            }
        } catch (const std::exception &error) {
            throw std::runtime_error("CSV line " + std::to_string(lineNumber) + ": " + error.what());
        }
    }
    if (!header || usable < 12 || first == last)
        throw std::runtime_error(
            "At least twelve weighted free-node observations across multiple times are required.");
    return data;
}
} // namespace inverse
