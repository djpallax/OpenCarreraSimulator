#include <gtest/gtest.h>

#include "ocs/math/mat4f.hpp"

TEST(Mat4f, IdentityPreservesVector) {
    const ocs::math::Mat4f identity = ocs::math::Mat4f::identity();
    const ocs::math::Vec4f input{1.0F, 2.0F, 3.0F, 1.0F};

    const ocs::math::Vec4f output = identity * input;

    EXPECT_FLOAT_EQ(output.x, input.x);
    EXPECT_FLOAT_EQ(output.y, input.y);
    EXPECT_FLOAT_EQ(output.z, input.z);
    EXPECT_FLOAT_EQ(output.w, input.w);
}

TEST(Mat4f, TranslationMovesPoint) {
    const ocs::math::Mat4f translation =
        ocs::math::Mat4f::translation({2.0F, -3.0F, 5.0F});

    const ocs::math::Vec4f result =
        translation * ocs::math::Vec4f{1.0F, 2.0F, 3.0F, 1.0F};

    EXPECT_FLOAT_EQ(result.x, 3.0F);
    EXPECT_FLOAT_EQ(result.y, -1.0F);
    EXPECT_FLOAT_EQ(result.z, 8.0F);
    EXPECT_FLOAT_EQ(result.w, 1.0F);
}

TEST(Mat4f, VulkanPerspectiveMapsNearAndFarToZeroAndOne) {
    const auto projection =
        ocs::math::Mat4f::perspective_vulkan(
            ocs::math::radians(60.0F),
            16.0F / 9.0F,
            0.1F,
            100.0F);

    const auto near_clip =
        projection * ocs::math::Vec4f{0.0F, 0.0F, -0.1F, 1.0F};
    const auto far_clip =
        projection * ocs::math::Vec4f{0.0F, 0.0F, -100.0F, 1.0F};

    EXPECT_NEAR(near_clip.z / near_clip.w, 0.0F, 1.0e-5F);
    EXPECT_NEAR(far_clip.z / far_clip.w, 1.0F, 1.0e-5F);
}

TEST(Mat4f, ScaleChangesVectorComponents) {
    const auto matrix = ocs::math::Mat4f::scale({2.0F, 3.0F, 4.0F});
    const auto result = matrix * ocs::math::Vec4f{1.0F, 2.0F, 3.0F, 1.0F};

    EXPECT_FLOAT_EQ(result.x, 2.0F);
    EXPECT_FLOAT_EQ(result.y, 6.0F);
    EXPECT_FLOAT_EQ(result.z, 12.0F);
    EXPECT_FLOAT_EQ(result.w, 1.0F);
}
