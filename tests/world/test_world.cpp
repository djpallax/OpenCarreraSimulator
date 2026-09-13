#include <gtest/gtest.h>

#include "ocs/math/mat4f.hpp"
#include "ocs/render/handles.hpp"
#include "ocs/world/world.hpp"
#include "ocs/world/world_position.hpp"

TEST(WorldPosition, CameraRelativeConversionPreservesLocalPrecision) {
    const ocs::world::WorldPosition camera{1'000'000'000.0, -2'000'000'000.0, 500.0};
    const ocs::world::WorldPosition object{1'000'000'001.25, -1'999'999'997.5, 503.75};
    const ocs::math::Vec3f relative = object.relative_to(camera);

    EXPECT_FLOAT_EQ(relative.x, 1.25F);
    EXPECT_FLOAT_EQ(relative.y, 2.5F);
    EXPECT_FLOAT_EQ(relative.z, 3.75F);
}

TEST(World, SpawnsAndRejectsStaleObjectHandles) {
    ocs::world::World world;
    const ocs::render::ModelHandle model{0U, 1U};
    const auto first = world.spawn("first", model);
    ASSERT_TRUE(first);
    EXPECT_EQ(world.object_count(), 1U);
    EXPECT_TRUE(world.destroy(first));
    EXPECT_EQ(world.get(first), nullptr);

    const auto second = world.spawn("second", model);
    ASSERT_TRUE(second);
    EXPECT_EQ(first.index, second.index);
    EXPECT_NE(first.generation, second.generation);
    EXPECT_EQ(world.get(first), nullptr);
}

TEST(World, ExtractsCameraRelativeRenderScene) {
    ocs::world::World world;
    const ocs::render::ModelHandle model{4U, 2U};
    ocs::world::Transform transform{};
    transform.position = {1010.0, 2020.0, 35.0};
    transform.scale = {2.0F, 3.0F, 4.0F};
    ASSERT_TRUE(world.spawn("object", model, transform));

    const auto scene = world.extract_render_scene({1000.0, 2000.0, 30.0});
    ASSERT_EQ(scene.instances.size(), 1U);
    EXPECT_EQ(scene.instances[0].model, model);

    const auto& matrix = scene.instances[0].model_matrix;
    EXPECT_FLOAT_EQ(matrix(0, 3), 10.0F);
    EXPECT_FLOAT_EQ(matrix(1, 3), 20.0F);
    EXPECT_FLOAT_EQ(matrix(2, 3), 5.0F);
    EXPECT_FLOAT_EQ(matrix(0, 0), 2.0F);
    EXPECT_FLOAT_EQ(matrix(1, 1), 3.0F);
    EXPECT_FLOAT_EQ(matrix(2, 2), 4.0F);
}

TEST(World, HiddenObjectsAreNotExtracted) {
    ocs::world::World world;
    const ocs::render::ModelHandle model{1U, 1U};
    const auto object = world.spawn("hidden", model);
    ASSERT_TRUE(object);
    world.get(object)->visible = false;

    const auto scene = world.extract_render_scene({});
    EXPECT_TRUE(scene.instances.empty());
}
