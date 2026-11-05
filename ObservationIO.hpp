#pragma once
// Strict reader for the observation CSV format described in README.md:
//   # samples=N
//   sample,node,x,y,z,weight
//   0,0,-0.22,0.5,-0.21,1
// Parsing completes before anything is returned, so a rejected file never changes the dataset.
#include "Model.hpp"
#include <algorithm>
#include <charconv>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace inverse {
    constexpr std::uintmax_t maxObservationFileBytes = 4 * 1024 * 1024;
    constexpr std::string_view observationHeader = "sample,node,x,y,z,weight";
    constexpr std::string_view sampleCountPrefix = "# samples=";
    constexpr int minUsableObservations = 12;
    constexpr float maxCoordinate = 100;

    inline std::string_view trim(std::string_view s) {
        constexpr std::string_view blank = " \t\r";
        while (!s.empty() && blank.find(s.front()) != std::string_view::npos)
            s.remove_prefix(1);
        while (!s.empty() && blank.find(s.back()) != std::string_view::npos)
            s.remove_suffix(1);
        return s;
    }

    /// Parses a whole field as a number; anything left over (units, hex, quotes) is an error.
    template <class T> T number(std::string_view text) {
        T result{};
        text = trim(text);
        const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), result);
        if (error != std::errc{} || end != text.data() + text.size())
            throw std::runtime_error("Invalid CSV number.");
        return result;
    }

    inline Dataset readObservations(const std::filesystem::path& path) {
        if (std::filesystem::file_size(path) > maxObservationFileBytes)
            throw std::runtime_error("Observation CSV exceeds 4 MiB.");
        std::ifstream in(path);
        if (!in)
            throw std::runtime_error("Cannot open observation CSV.");
        Dataset data;
        std::vector<bool> present;
        bool header = false;
        std::string line;
        int lineNumber = 0, usable = 0, first = maxSamples, last = -1;
        while (std::getline(in, line)) {
            ++lineNumber;
            if (lineNumber == 1 && line.starts_with("\xEF\xBB\xBF"))
                line.erase(0, 3); // UTF-8 byte order mark
            std::string_view text = trim(line);
            if (text.empty())
                continue;
            try {
                if (text.starts_with(sampleCountPrefix)) {
                    if (data.samples != 0)
                        throw std::runtime_error("Duplicate sample count.");
                    data.samples = number<int>(text.substr(sampleCountPrefix.size()));
                    if (data.samples < minSamples || data.samples > maxSamples)
                        throw std::runtime_error("Sample count must be 8..360.");
                    data.observations.resize(static_cast<size_t>(data.samples) * nodes);
                    present.resize(data.observations.size());
                    continue;
                }
                if (text.front() == '#')
                    continue;
                if (!header) {
                    if (text != observationHeader || data.samples == 0)
                        throw std::runtime_error("Expected '# samples=N' and 'sample,node,x,y,z,weight'.");
                    header = true;
                    continue;
                }
                std::array<std::string_view, 6> fields{};
                for (size_t i = 0; i < fields.size(); ++i) {
                    const auto comma = text.find(',');
                    const bool lastField = i + 1 == fields.size();
                    if (lastField ? comma != std::string_view::npos : comma == std::string_view::npos)
                        throw std::runtime_error("Expected six CSV columns.");
                    fields[i] = text.substr(0, comma);
                    if (!lastField)
                        text.remove_prefix(comma + 1);
                }
                const int sample = number<int>(fields[0]), node = number<int>(fields[1]);
                if (sample < 0 || sample >= data.samples || node < 0 || node >= nodes)
                    throw std::runtime_error("Sample/node index outside the declared range.");
                const size_t id = static_cast<size_t>(sample) * nodes + static_cast<size_t>(node);
                if (present[id])
                    throw std::runtime_error("Duplicate sample/node row.");
                present[id] = true;
                F4 p{};
                for (size_t i = 0; i < p.size(); ++i) {
                    p[i] = number<float>(fields[i + 2]);
                    if (!std::isfinite(p[i]))
                        throw std::runtime_error("All values must be finite.");
                }
                if (std::abs(p[0]) > maxCoordinate || std::abs(p[1]) > maxCoordinate || std::abs(p[2]) > maxCoordinate ||
                    p[3] < 0 || p[3] > 1)
                    throw std::runtime_error("Coordinates must be within +/-100 and weights within 0..1.");
                if (node >= freeNodes)
                    p[3] = 0; // anchors never contribute residuals
                data.observations[id] = p;
                if (p[3] > 0) {
                    ++usable;
                    first = std::min(first, sample);
                    last = std::max(last, sample);
                }
            }
            catch (const std::exception& error) {
                throw std::runtime_error("CSV line " + std::to_string(lineNumber) + ": " + error.what());
            }
        }
        if (!header || usable < minUsableObservations || first == last)
            throw std::runtime_error("At least twelve weighted free-node observations across multiple times are required.");
        return data;
    }
}
