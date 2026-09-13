#include <gtest/gtest.h>

#include "ocs/math/vec3f.hpp"

TEST(Vec3f, CrossProductUsesRightHandRule) {
    const ocs::math::Vec3f x{1.0F, 0.0F, 0.0F};
    const ocs::math::Vec3f y{0.0F, 1.0F, 0.0F};

    const ocs::math::Vec3f z = x.cross(y);

    EXPECT_FLOAT_EQ(z.x, 0.0F);
    EXPECT_FLOAT_EQ(z.y, 0.0F);
    EXPECT_FLOAT_EQ(z.z, 1.0F);
}

TEST(Vec3f, NormalizationProducesUnitLength) {
    const ocs::math::Vec3f value{3.0F, 4.0F, 0.0F};
    const ocs::math::Vec3f normalized = value.normalized();

    EXPECT_NEAR(normalized.length(), 1.0F, 1.0e-6F);
}
