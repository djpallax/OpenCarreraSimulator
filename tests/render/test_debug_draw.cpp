#include <gtest/gtest.h>

#include <array>

#include "ocs/render/debug_draw.hpp"

TEST(DebugDraw, BuildsCrossAndWireBoxAsIndependentLines) {
    ocs::render::DebugDrawList debug;
    debug.cross({0.0F, 0.0F, 0.0F}, 0.5F, {1.0F, 1.0F, 0.0F, 1.0F});

    const std::array<ocs::math::Vec3f, 8> corners{{
        {-1.0F, -1.0F, -1.0F}, {-1.0F, -1.0F, 1.0F},
        {-1.0F, 1.0F, -1.0F},  {-1.0F, 1.0F, 1.0F},
        {1.0F, -1.0F, -1.0F},  {1.0F, -1.0F, 1.0F},
        {1.0F, 1.0F, -1.0F},   {1.0F, 1.0F, 1.0F}
    }};
    debug.box(corners, {0.0F, 1.0F, 1.0F, 1.0F});

    EXPECT_EQ(debug.size(), 15U); // 3 cross axes + 12 box edges.
    EXPECT_FALSE(debug.empty());
}
