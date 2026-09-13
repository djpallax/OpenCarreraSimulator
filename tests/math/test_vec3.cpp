#include <gtest/gtest.h>

#include "ocs/math/vec3.hpp"

TEST(Vec3, DotProduct) {
    const ocs::math::Vec3 a{1.0, 2.0, 3.0};
    const ocs::math::Vec3 b{4.0, 5.0, 6.0};
    EXPECT_DOUBLE_EQ(a.dot(b), 32.0);
}

TEST(Vec3, Length) {
    const ocs::math::Vec3 v{3.0, 4.0, 0.0};
    EXPECT_DOUBLE_EQ(v.length(), 5.0);
}
