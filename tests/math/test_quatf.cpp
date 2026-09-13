#include <gtest/gtest.h>

#include <numbers>

#include "ocs/math/quatf.hpp"

TEST(Quatf, RotatesAroundPositiveZ) {
    const auto rotation = ocs::math::Quatf::from_axis_angle(
        {0.0F, 0.0F, 1.0F},
        std::numbers::pi_v<float> * 0.5F);
    const auto rotated = rotation.rotate({1.0F, 0.0F, 0.0F});

    EXPECT_NEAR(rotated.x, 0.0F, 1.0e-5F);
    EXPECT_NEAR(rotated.y, 1.0F, 1.0e-5F);
    EXPECT_NEAR(rotated.z, 0.0F, 1.0e-5F);
}
