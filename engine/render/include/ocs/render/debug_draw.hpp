#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "ocs/math/vec3f.hpp"
#include "ocs/math/vec4f.hpp"

namespace ocs::render {

struct DebugLine {
    math::Vec3f start{};
    math::Vec3f end{};
    math::Vec4f color{1.0F, 1.0F, 1.0F, 1.0F};
};

class DebugDrawList {
public:
    void clear() noexcept { lines_.clear(); }
    void reserve(const std::size_t count) { lines_.reserve(count); }

    void line(const math::Vec3f start,
              const math::Vec3f end,
              const math::Vec4f color) {
        lines_.push_back({start, end, color});
    }

    void cross(const math::Vec3f center,
               const float half_size,
               const math::Vec4f color) {
        line(center + math::Vec3f{half_size, 0.0F, 0.0F},
             center - math::Vec3f{half_size, 0.0F, 0.0F}, color);
        line(center + math::Vec3f{0.0F, half_size, 0.0F},
             center - math::Vec3f{0.0F, half_size, 0.0F}, color);
        line(center + math::Vec3f{0.0F, 0.0F, half_size},
             center - math::Vec3f{0.0F, 0.0F, half_size}, color);
    }

    void box(const std::array<math::Vec3f, 8>& corners,
             const math::Vec4f color) {
        constexpr std::array<std::array<std::size_t, 2>, 12> kEdges{{
            {{0, 1}}, {{0, 2}}, {{0, 4}},
            {{1, 3}}, {{1, 5}},
            {{2, 3}}, {{2, 6}},
            {{3, 7}},
            {{4, 5}}, {{4, 6}},
            {{5, 7}}, {{6, 7}}
        }};
        for (const auto& edge : kEdges) {
            line(corners[edge[0]], corners[edge[1]], color);
        }
    }

    [[nodiscard]] const std::vector<DebugLine>& lines() const noexcept { return lines_; }
    [[nodiscard]] std::vector<DebugLine>& lines() noexcept { return lines_; }
    [[nodiscard]] std::size_t size() const noexcept { return lines_.size(); }
    [[nodiscard]] bool empty() const noexcept { return lines_.empty(); }

private:
    std::vector<DebugLine> lines_{};
};

} // namespace ocs::render
