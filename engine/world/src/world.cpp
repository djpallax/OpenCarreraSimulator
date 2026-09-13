#include "ocs/world/world.hpp"

#include <utility>

#include "ocs/math/mat4f.hpp"

namespace ocs::world {

ObjectHandle World::spawn(std::string name,
                          const render::ModelHandle model,
                          const Transform& transform,
                          const bool movable) {
    if (!model) {
        return {};
    }
    return objects_.emplace(WorldObject{
        .name = std::move(name),
        .model = model,
        .transform = transform,
        .visible = true,
        .movable = movable
    });
}

bool World::destroy(const ObjectHandle handle) noexcept {
    return objects_.erase(handle);
}

WorldObject* World::get(const ObjectHandle handle) noexcept {
    return objects_.get(handle);
}

const WorldObject* World::get(const ObjectHandle handle) const noexcept {
    return objects_.get(handle);
}

render::RenderScene World::extract_render_scene(const WorldPosition& camera_origin) const {
    render::RenderScene result{};
    result.instances.reserve(objects_.size());

    objects_.for_each([&](const ObjectHandle, const WorldObject& object) {
        if (!object.visible || !object.model) {
            return;
        }

        const math::Vec3f relative_position = object.transform.position.relative_to(camera_origin);
        const math::Mat4f model_matrix =
            math::Mat4f::translation(relative_position) *
            math::Mat4f::rotation(object.transform.rotation) *
            math::Mat4f::scale(object.transform.scale);
        result.instances.push_back({
            .model = object.model,
            .model_matrix = model_matrix
        });
    });

    return result;
}

} // namespace ocs::world
